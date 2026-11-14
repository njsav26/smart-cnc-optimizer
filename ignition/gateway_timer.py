# Ignition Gateway Timer Script: CNC01 spindle simulator
# Location: Project > Scripting > Gateway Events > Timer
# Settings: 500 ms, Fixed Delay, Dedicated thread
# Runtime: Jython 2.7 inside Ignition 8.3 (not standalone Python)
#
# Fault_Mode: 0=healthy, 1=chipped tool, 2=worn tool (ramps over ~2 min), 3=coolant failure

def handleTimerEvent():
	# Simulates CNC01 spindle physics, writes tags, publishes JSON to MQTT.
	import random, math, time

	BASE = "[default]TestPlant/Line1/CNC01/"
	TOPIC = "plant/test/cnc01/telemetry"
	MQTT_SERVER = "Chariot SCADA"

	RPM0, CUR0, VIB0 = 10000.0, 14.5, 0.35

	HEALTHY, CHIPPED, WORN, COOLANT_FAIL = 0, 1, 2, 3


	# 1. Read inputs
	paths = [BASE + "Fault_Mode", BASE + "Feed_Rate_Override_Pct", BASE + "Coolant_Temp_C", BASE + "Tool_Wear_Pct"]
	fault, feed, last_temp, wear = [qv.value for qv in system.tag.readBlocking(paths)]
	k = feed / 100.0

	# 2. Healthy baseline with sensor noise
	rpm = random.gauss(RPM0, 50)
	cur = random.gauss(CUR0, 0.25)
	vib = random.gauss(VIB0, 0.03)
	temp_target = 26.25 + 2.25 * math.sin(time.time() / 100.0)

	# 3. Wear Progression
	WEAR_TIME_S = 120								# time to fully wear out at 100% feed
	TICK_S = 0.5 									# timer period

	if fault == WORN:
		# higher feed = faster wear
		wear = min(wear + (100.0 / WEAR_TIME_S) * TICK_S * k, 100.0)	# lower feed, lower wear
	elif fault == HEALTHY:							# reset on HEALTHY, operator changed tool
		wear = 0.0

	w = wear / 100.0
	w_effect = w ** 2								# slow wear at first, then increases over time

	# 4. Fault physics (effects reduce as feed decreases)
	if fault == CHIPPED:							# CHIPPED END MILL --> impacts --> vibration-dominated
		vib += 1.75 * k * k + random.gauss(0, 0.1)  # uneven impact causes more vibration
		cur += 8.0 * k								# poor cutting/rubbing require more force to remove material; therefore more force; torque directly proportional to current
		rpm -= 80 * k								# more load torque slows the spindle until drive compensates; speed droop
		temp_target += 0.5
	elif fault == WORN:								# WORN TOOL --> friction --> current and heat-dominated
		vib += 0.6 * w_effect * k					# milder version of chipped conditions
		cur += 3.0 * w_effect * k					# cutting force ~= specific cutting force x depth of cut x chip thickness
		rpm -= 30 * w_effect * k
		temp_target += 2.0 * w_effect				# extra heat from friction

	if fault == COOLANT_FAIL:						# COOLANT FAILURE --> heat cannot escape the tool and material as efficiently
		temp = min(last_temp + 0.05, 45.0)
	else:
		temp = last_temp + 0.1 * (temp_target - last_temp) + random.gauss(0, 0.05)

	vib = max(vib, 0)

	# 5. Write simulated values to tags
	system.tag.writeBlocking(
		tagPaths = [BASE + "Spindle_RPM", BASE + "Motor_Current_Amps",
		BASE + "Vibration_RMS_g", BASE + "Coolant_Temp_C", BASE + "Tool_Wear_Pct"],
		values = [round(rpm, 0), round(cur, 2), round(vib, 3), round(temp, 2), round(wear, 1)])

	# 6. Publish JSON
	payload = system.util.jsonEncode({
		"machine": "CNC01",
		"ts": system.date.toMillis(system.date.now()),
		"rpm": round(rpm, 0),
		"current": round(cur, 2),
		"temp": round(temp, 2),
		"vibration": round(vib, 3),
		"wear": round(wear, 1),
		"feed_override": feed,
		"fault_mode": fault,
	})
	system.cirruslink.transmission.publish(MQTT_SERVER, TOPIC, payload, 0, False)
