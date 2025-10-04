import cv2
from enum import Enum
import mediapipe as mp

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist

class FingerState(Enum):
    FOLDED = 0
    EXTENDED = 1

class Direction(Enum):
    FORWARD = 0
    STOP = 1
    LEFT = 2
    RIGHT = 3
    NONE = 4

class GestureDetector(Node):

    def __init__(self):
        super().__init__('gesture_detector')

        self.vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        
        self.mp_hands = mp.solutions.hands
        self.mp_drawing = mp.solutions.drawing_utils

        # Define parameters for MediaPipe Hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=2,
            min_detection_confidence=0.7,
            min_tracking_confidence=0.5
        )

        self.detect_gestures()

    def index_finger_status(self, hand_landmarks, image):
        h, w, _ = image.shape
        lm = hand_landmarks.landmark

        # Convert to pixel coordinates
        wrist = (int(lm[0].x * w), int(lm[0].y * h))
        index_pip = (int(lm[6].x * w), int(lm[6].y * h))  # joint before tip
        index_tip = (int(lm[8].x * w), int(lm[8].y * h))

        # Check if extended
        state = FingerState.EXTENDED if index_tip[1] < index_pip[1] else FingerState.FOLDED

        direction = Direction.NONE
        if state == FingerState.EXTENDED:
            dx = index_tip[0] - wrist[0]
            dy = index_tip[1] - wrist[1]

            if abs(dx) > abs(dy):  # Horizontal motion dominates
                direction = Direction.RIGHT if dx > 0 else Direction.LEFT
            else:  # Vertical motion dominates
                direction = Direction.DOWN if dy > 0 else Direction.UP

        return state, direction
    
    def detect_commands(self, hand_landmarks):
        lm = hand_landmarks.landmark
        fingers = []
        direction = Direction.NONE

        if lm[4].x > lm[3].x:
            fingers.append(1)
        else:
            fingers.append(0)
        
        for finger in [8, 12, 16, 20]:
            if lm[finger].y < lm[finger -1].y:
                fingers.append(1)
            else:
                fingers.append(0)

        # [thumb, index, middle, ring, pinky]
        if fingers == [0, 1, 0, 0, 0]:
            direction = Direction.FORWARD
        elif fingers == [0, 0, 0, 0, 0]:
            direction = Direction.STOP
        elif fingers == [0, 1, 1, 1, 1]:
            direction = Direction.LEFT
        elif fingers == [1, 0, 0, 0, 0]:
            direction = Direction.RIGHT

        return direction

    def detect_gestures(self):
        cap = cv2.VideoCapture(0)

        while cap.isOpened():
            success, frame = cap.read()
            if not success:
                self.get_logger().info("Camera could not be opened.")
                break

            # Flip horizontally for a mirror view
            frame = cv2.flip(frame, 1)

            # Convert BGR (OpenCV) to RGB (MediaPipe)
            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            # Process frame with MediaPipe
            result = self.hands.process(rgb_frame)

            # Draw landmarks
            if result.multi_hand_landmarks:
                for hand_landmarks in result.multi_hand_landmarks:
                    self.mp_drawing.draw_landmarks(
                        frame, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)
                    # Call gesture detection function
                    direction = self.detect_commands(hand_landmarks)
                    self.get_logger().info('Hand gesture detected: %s' % direction)

                    # Publish velocity command based on gesture
                    vel_msg = Twist()
                    if direction == Direction.FORWARD:
                        vel_msg.linear.x = 0.20
                    elif direction == Direction.STOP:
                        vel_msg.linear.x = 0.0
                    elif direction == Direction.LEFT:
                        vel_msg.angular.z = 0.5
                    elif direction == Direction.RIGHT:
                        vel_msg.angular.z = -0.5
                    
                    self.vel_publisher.publish(vel_msg)

            # Show frame
            cv2.imshow("MediaPipe Hands", frame)

            if cv2.waitKey(1) & 0xFF == 27:  # ESC key
                break

        cap.release()
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)

    gesture_detector = GestureDetector()

    rclpy.spin(gesture_detector)

    gesture_detector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
