import json
import paho.mqtt.client as mqtt


def on_connect(client, userdata, flags, reason_code, properties):
    print("Connected:", reason_code)
    client.subscribe("plant/test/cnc01/telemetry")


def on_message(client, userdata, msg):
    print(json.loads(msg.payload))


client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
client.on_connect = on_connect
client.on_message = on_message
client.connect("localhost", 1883)
client.loop_forever()
