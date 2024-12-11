import cv2
import mediapipe as mp
import numpy as np
import socket
import struct
import time
import math

# Initialize MediaPipe
mp_drawing = mp.solutions.drawing_utils
mp_holistic = mp.solutions.holistic

# UDP Socket Setup
UDP_IP = "127.0.0.1"
UDP_PORT = 5005
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Start webcam
cap = cv2.VideoCapture(0)

# Adjustable transmission delay in seconds
i = 0
i_max = 10  # Sliding window size
transmission_delay = 0.1  # Transmission delay
last_transmission_time = time.time()  # Track the last transmission time

# Sliding window arrays
s_raw = [[0] * i_max for _ in range(3)]
e_raw = [[0] * i_max for _ in range(3)]
w_raw = [[0] * i_max for _ in range(3)]
ef_raw = [[0] * i_max for _ in range(3)]

# Define constant lengths for the links
link1_length = 5.0  # Shoulder to elboqw
link2_length = 4.0  # Elbow to wrist
link3_length = 1.0  # Wrist to fingertip

# Coordinate storage
elbow_coords, wrist_coords, endf_coords = (0, 0, 0), (0, 0, 0), (0, 0, 0)

# Visualization frame
output_frame = np.zeros((500, 500, 3), dtype=np.uint8)


# Define scale and origin for drawing
scale = 25  # Increased scale for larger visualization
origin = (250, 250)  # Origin remains in the center of the frame



def calculate_ik(x, y, phi, l1, l2, l3):
    try:
        # คิดตำแหน่ง ข้อมือ เป็นจุดปลายใน inverse/workspace
        target_x = x - l3 * math.cos(phi)
        target_y = y - l3 * math.sin(phi)

        # คิดระยะห่างจาก 0 ถึง ข้อมือ
        distance = math.sqrt(target_x**2 + target_y**2)
        if distance > (l1 + l2) or distance < abs(l1 - l2):  # Check reachable range
            print("Target is out of reach.")
            return None, None, None

        # Use the law of cosines to find theta2
        cos_theta2 = (target_x**2 + target_y**2 - l1**2 - l2**2) / (2 * l1 * l2)
        if abs(cos_theta2) > 1:  # Numerical precision check
            print("Target is out of reach due to numerical issues.")
            return None, None, None

        sin_theta2 = math.sqrt(max(0, 1 - cos_theta2**2))  # Positive root to avoid math domain error
        theta2 = math.atan2(sin_theta2, cos_theta2)

        # Calculate theta1
        k1 = l1 + l2 * cos_theta2
        k2 = l2 * sin_theta2
        theta1 = math.atan2(target_y, target_x) - math.atan2(k2, k1)

        # Calculate theta3
        theta3 = phi - (theta1 + theta2)

        return theta1, theta2, theta3
    except Exception as e:
        print(f"Error in IK calculation: {e}")
        return None, None, None

    
# Helper Functions
def normalize_vector(vector):
    norm = np.linalg.norm(vector)
    if norm == 0:
        return np.array([0, 0])  # Avoid division by zero
    return vector / norm

def smooth(data, alpha=0.5):
    smoothed = [data[0]]
    for point in data[1:]:
        smoothed.append(alpha * point + (1 - alpha) * smoothed[-1])
    return smoothed

def draw_point(image, coords, color, scale=50, origin=(250, 250)):
    x, y = int(coords[0] * scale + origin[0]), int(coords[1] * scale + origin[1])
    cv2.circle(image, (x, y), 5, color, -1)

def draw_line(image, start, end, color, scale=50, origin=(250, 250)):
    start_x, start_y = int(start[0] * scale + origin[0]), int(start[1] * scale + origin[1])
    end_x, end_y = int(end[0] * scale + origin[0]), int(end[1] * scale + origin[1])
    cv2.line(image, (start_x, start_y), (end_x, end_y), color, 2)

# def jacobian_determinant(theta2, l1, l2):
#     # Compute determinant of Jacobian
#     return l1 * l2 * np.sin(theta2)


# # Check Singularity
# l1 = 5
# l2 = 4
# theta1_range = np.linspace(0, 2 * np.pi, 20)  # Coarse granularity
# theta2_range = np.linspace(0, 2 * np.pi, 200)  # Finer granularity for key joint
# singularities = []

# for theta2 in theta2_range:
#     det_J = jacobian_determinant(theta2, l1, l2)
#     if np.abs(det_J) < 1e-6:
#         rounded_singularity = (round(theta2, 2))
#         if rounded_singularity not in singularities:
#             singularities.append(rounded_singularity)

# print(f"Total unique singularities: {len(singularities)}")
# print("Singular configurations:", singularities)

# Initialize MediaPipe Holistic Model
with mp_holistic.Holistic(min_detection_confidence=0.5, min_tracking_confidence=0.5) as holistic:
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        # Flip and process the frame
        frame = cv2.flip(frame, 1)
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = holistic.process(frame_rgb)

        # Draw landmarks
        frame.flags.writeable = True
        if results.pose_landmarks:
            mp_drawing.draw_landmarks(frame, results.pose_landmarks, mp_holistic.POSE_CONNECTIONS)
        if results.left_hand_landmarks:
            mp_drawing.draw_landmarks(frame, results.left_hand_landmarks, mp_holistic.HAND_CONNECTIONS)
        if results.right_hand_landmarks:
            mp_drawing.draw_landmarks(frame, results.right_hand_landmarks, mp_holistic.HAND_CONNECTIONS)

        # Process landmarks for shoulder, elbow, wrist, and fingertip
        if results.pose_landmarks:
            landmarks = results.pose_landmarks.landmark
            shoulder_raw = landmarks[mp_holistic.PoseLandmark.RIGHT_SHOULDER]
            elbow_raw = landmarks[mp_holistic.PoseLandmark.RIGHT_ELBOW]
            wrist_raw = landmarks[mp_holistic.PoseLandmark.RIGHT_WRIST]
            endf_raw = landmarks[mp_holistic.PoseLandmark.RIGHT_INDEX]

            # Update sliding window arrays
            s_raw[0][i], s_raw[1][i], s_raw[2][i] = shoulder_raw.x, shoulder_raw.y, shoulder_raw.z
            e_raw[0][i], e_raw[1][i], e_raw[2][i] = elbow_raw.x, elbow_raw.y, elbow_raw.z
            w_raw[0][i], w_raw[1][i], w_raw[2][i] = wrist_raw.x, wrist_raw.y, wrist_raw.z
            ef_raw[0][i], ef_raw[1][i], ef_raw[2][i] = endf_raw.x, endf_raw.y, endf_raw.z
            i += 1

            if i >= i_max:
                i = 0

            # Transmit data based on delay
            current_time = time.time()
            if current_time - last_transmission_time >= transmission_delay:
                shoulder = (sum(s_raw[0])/i_max, sum(s_raw[1])/i_max, sum(s_raw[2])/i_max)
                elbow = (sum(e_raw[0])/i_max, sum(e_raw[1])/i_max, sum(e_raw[2])/i_max)
                wrist = (sum(w_raw[0])/i_max, sum(w_raw[1])/i_max, sum(w_raw[2])/i_max)
                endf = (sum(ef_raw[0])/i_max, sum(ef_raw[1])/i_max, sum(ef_raw[2])/i_max)

                # Calculate vectors and normalize them in X-Y plane
                elbow_vector = np.array([elbow[0] - shoulder[0], elbow[1] - shoulder[1]])
                wrist_vector = np.array([wrist[0] - elbow[0], wrist[1] - elbow[1]])
                endf_vector = np.array([endf[0] - wrist[0], endf[1] - wrist[1]])

                elbow_unit_xy = normalize_vector(elbow_vector)
                wrist_unit_xy = normalize_vector(wrist_vector)
                endf_unit_xy = normalize_vector(endf_vector)

                # Scale the unit vectors to constant lengths
                elbow_coords = (
                    link1_length * elbow_unit_xy[0],
                    link1_length * elbow_unit_xy[1],
                    elbow[2]
                )
                wrist_coords = (
                    elbow_coords[0] + link2_length * wrist_unit_xy[0],
                    elbow_coords[1] + link2_length * wrist_unit_xy[1],
                    wrist[2]
                )
                endf_coords = (
                    wrist_coords[0] + link3_length * endf_unit_xy[0],
                    wrist_coords[1] + link3_length * endf_unit_xy[1],
                    endf[2]
                )

                # UDP transmission
                data = struct.pack('<ffffffffffff',
                                   0, 0, 0,  # Shoulder (relative origin)
                                   elbow_coords[0], elbow_coords[1], elbow_coords[2],
                                   wrist_coords[0], wrist_coords[1], wrist_coords[2],
                                   endf_coords[0], endf_coords[1], endf_coords[2])
                sock.sendto(data, (UDP_IP, UDP_PORT))
                #print(f"End Effector: X={endf_coords[0]}, Y={endf_coords[1]}, Z={endf_coords[2]}")
                last_transmission_time = current_time

 # Visualize the output
        output_frame.fill(0)
        draw_point(output_frame, (0, 0, 0), (255, 255, 255), scale=scale, origin=origin)  # Origin (shoulder)
        draw_line(output_frame, (0, 0, 0), elbow_coords, (0, 255, 0), scale=scale, origin=origin)
        draw_line(output_frame, elbow_coords, wrist_coords, (255, 0, 0), scale=scale, origin=origin)
        draw_line(output_frame, wrist_coords, endf_coords, (0, 0, 255), scale=scale, origin=origin)
        draw_point(output_frame, elbow_coords, (0, 255, 0), scale=scale, origin=origin)
        draw_point(output_frame, wrist_coords, (255, 0, 0), scale=scale, origin=origin)
        draw_point(output_frame, endf_coords, (0, 0, 255), scale=scale, origin=origin)

        # Show both frames
        cv2.imshow('Hand Gesture Robot Arm - Input', frame)
        cv2.imshow('Hand Gesture Robot Arm - Output', output_frame)
        
        dx = endf_coords[0] - wrist_coords[0]
        dy = endf_coords[1] - wrist_coords[1]
        # ใช้ atan2 จะได้ครอบคลุม quadrant
        phi = math.atan2(dy, dx) if dx != 0 else 0
        
        theta1, theta2, theta3 = calculate_ik(endf_coords[0], endf_coords[1], phi, 5, 4, 1)
        if theta1 is not None and theta2 is not None and theta3 is not None:
            # print(f"Angles (in degrees):")
            # print(f"Theta1: {math.degrees(theta1):.2f}°")
            # print(f"Theta2: {math.degrees(theta2):.2f}°")
            # print(f"Theta3: {math.degrees(theta3):.2f}°")
            pass

        

        # Exit on 'q'
        if cv2.waitKey(10) & 0xFF == ord('q'):
            break

# Release resources
cap.release()
cv2.destroyAllWindows()
sock.close()
