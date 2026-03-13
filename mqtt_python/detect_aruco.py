import cv2
import numpy as np
import os
import json
import paho.mqtt.client as mqtt

CALIBRATION_FILE = "calibration/camera_params.npz"
MQTT_BROKER = "localhost"
MQTT_PORT = 1883
MQTT_TOPIC = "robobot/drive/cmd"

def load_calibration():
    if not os.path.exists(CALIBRATION_FILE):
        print(f"Error: Calibration file not found at {CALIBRATION_FILE}")
        return None, None
        
    with np.load(CALIBRATION_FILE) as data:
        return data['camera_matrix'], data['dist_coeffs']

def main():
    camera_matrix, dist_coeffs = load_calibration()
    if camera_matrix is None:
        return

    mqtt_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    mqtt_connected = False
    
    try:
        mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)
        mqtt_client.loop_start()
        mqtt_connected = True
        print("Connected to MQTT Broker.")
    except Exception as e:
        print("Running in local testing mode without MQTT communication.")

    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    aruco_params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
    
    marker_length = 0.05 
    cap = cv2.VideoCapture(0)
    
    if not cap.isOpened():
        print("Error: Cannot open camera")
        return

    window_name = "Robobot Vision: ArUco Detection"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.setWindowProperty(window_name, cv2.WND_PROP_TOPMOST, 1)

    print("Vision system running. Press 'q' to stop.")

    while True:
        ret, frame = cap.read()
        if not ret:
            break
            
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = detector.detectMarkers(gray)
        
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            
            obj_points = np.array([
                [-marker_length / 2, marker_length / 2, 0],
                [marker_length / 2, marker_length / 2, 0],
                [marker_length / 2, -marker_length / 2, 0],
                [-marker_length / 2, -marker_length / 2, 0]
            ], dtype=np.float32)
            
            success, rvec, tvec = cv2.solvePnP(
                obj_points, corners[0][0], camera_matrix, dist_coeffs
            )
            
            if success:
                cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec, marker_length)
                z_distance = float(tvec[2][0])
                x_offset = float(tvec[0][0])
                
                cv2.putText(frame, f"Z: {z_distance:.2f}m", (10, 30), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
                payload = {
                    "target_id": int(ids[0][0]),
                    "distance_z": z_distance,
                    "offset_x": x_offset
                }
                
                if mqtt_connected:
                    mqtt_client.publish(MQTT_TOPIC, json.dumps(payload))
        
        cv2.imshow(window_name, frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
            
    cap.release()
    cv2.destroyAllWindows()
    if mqtt_connected:
        mqtt_client.loop_stop()
        mqtt_client.disconnect()

if __name__ == "__main__":
    main()