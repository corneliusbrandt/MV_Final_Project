import cv2
from enum import Enum
import mediapipe as mp

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

class FingerState(Enum):
    FOLDED = 0
    EXTENDED = 1

class Direction(Enum):
    LEFT = 0
    RIGHT = 1
    UP = 2
    DOWN = 3
    STOP = 4
    NONE = 5

class GestureDetector(Node):

    def __init__(self):
        super().__init__('gesture_detector')
        
        self.mp_hands = mp.solutions.hands
        self.mp_drawing = mp.solutions.drawing_utils

        self.hands = mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
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

    def detect_gestures(self):
        # Open webcam
        cap = cv2.VideoCapture(0)  # 0 = default webcam

        while cap.isOpened():
            success, frame = cap.read()
            if not success:
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
                    state, direction = self.index_finger_status(hand_landmarks, frame)
                    self.get_logger().info('Publishing: "%s"' % direction)

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
