import json
import paho.mqtt.client as mqtt_client

x = y = yaw = 0.0
velocity = angular_velocity = 0.0

def on_message(client, userdata, msg):
    global x, y, yaw, velocity, angular_velocity
    data = json.loads(msg.payload.decode('utf-8'))

    if 'x' in data:
        s = data['x']
        x, y, yaw = s.get('x', 0), s.get('y', 0), s.get('yaw', 0)
        velocity, angular_velocity = s.get('velocity', 0), s.get('angular_velocity', 0)
    elif 'position' in data:
        x, y = data['position'].get('x', 0), data['position'].get('y', 0)
        yaw = data.get('orientation', {}).get('yaw', 0)
        velocity = data.get('velocity', {}).get('linear', 0)
        angular_velocity = data.get('velocity', {}).get('angular', 0)

client = mqtt_client.Client(client_id="pose-reader")
client.on_message = on_message
client.connect('localhost', 1883)
client.subscribe("robobot/kalman/state", qos=1)
client.loop_start()
