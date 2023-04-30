#!/usr/bin/env python3

from depthai_hand_tracker.HandTrackerRenderer import HandTrackerRenderer
import argparse
import threading
import time
from queue import Queue
import sys
from time import sleep
from typing import Any
from akari_client import AkariClient

sys.path.append("depthai_hand_tracker")

VIDEO_WIDTH, VIDEO_HEIGHT = 1152, 648

pan_target_angle = 0.0
tilt_target_angle = 0.0


def get_args() -> Any:
    parser = argparse.ArgumentParser()
    parser_tracker = parser.add_argument_group("Tracker arguments")
    parser_tracker.add_argument(
        '-g',
        '--gesture',
        choices=["ONE", "TWO", "THREE", "FOUR", "FIVE", "OK", "PEACE", "FIST"],
        help="Specify gestures to track"
    )
    return parser.parse_args()


# 手追従するクラス
class HandTracker:

    def __init__(self) -> None:
        global pan_target_angle
        global tilt_target_angle

        # AkariClientのインスタンスを作成する
        self.akari = AkariClient()
        # 関節制御用のインスタンスを取得する
        self.joints = self.akari.joints

        # サーボトルクON
        self.joints.enable_all_servo()
        # モータ速度設定
        self.joints.set_joint_velocities(pan=10, tilt=10)
        # モータ加速度設定
        self.joints.set_joint_accelerations(pan=30, tilt=30)

        time.sleep(0.5)

        # Initialize motor position
        self.joints.move_joint_positions(sync=True, pan=0, tilt=0)
        self.currentMotorAngle = self.joints.get_joint_positions()

        # Dynamixel Input Value
        pan_target_angle = self.currentMotorAngle["pan"]
        tilt_target_angle = self.currentMotorAngle["tilt"]

    def _tracker(self) -> None:
        global pan_target_angle
        global tilt_target_angle
        while True:
            self.joints.move_joint_positions(
                pan=pan_target_angle, tilt=tilt_target_angle
            )
            sleep(0.01)


# モーター可動角を更新するクラス
class DirectionUpdater:

    _H_PIX_WIDTH = VIDEO_WIDTH
    _H_PIX_HEIGHT = VIDEO_HEIGHT
    _PAN_THRESHOLD = 0.1
    _TILT_THRESHOLD = 0.1
    _pan_dev = 0
    _tilt_dev = 0
    # モータゲインの最大幅。追従性の最大はここで変更
    _MAX_PAN_GAIN = 0.1
    _MAX_TILT_GAIN = 0.1
    # モータゲインの最小幅。追従性の最小はここで変更
    _MIN_PAN_GAIN = 0.07
    _MIN_TILT_GAIN = 0.07
    # 顔の距離によってモータゲインを変化させる係数。上げると早い動きについていきやすいが、オーバーシュートしやすくなる。
    _GAIN_COEF_PAN = 0.0001
    _GAIN_COEF_TILT = 0.0001

    _pan_p_gain = _MIN_PAN_GAIN
    _tilt_p_gain = _MIN_TILT_GAIN

    _PAN_POS_MAX = 1.047
    _PAN_POS_MIN = -1.047
    _TILT_POS_MAX = 0.523
    _TILT_POS_MIN = -0.523
    _pan_offset = 0
    _tilt_offset = 0.02

    def __init__(self) -> None:
        self._bbox_x = 0
        self._bbox_y = 0
        self._bbox_width = 0
        self._bbox_height = 0
        self._old_bbox_x: float = 0
        self._old_bbox_y: float = 0

    def _calc_p_gain(self) -> None:
        self._pan_p_gain = self._GAIN_COEF_PAN * self._bbox_width
        if self._pan_p_gain > self._MAX_PAN_GAIN:
            self._pan_p_gain = self._MAX_PAN_GAIN
        elif self._pan_p_gain < self._MIN_PAN_GAIN:
            self._pan_p_gain = self._MIN_PAN_GAIN
        self._tilt_p_gain = self._GAIN_COEF_TILT * self._bbox_width
        if self._tilt_p_gain > self._MAX_TILT_GAIN:
            self._tilt_p_gain = self._MAX_TILT_GAIN
        elif self._tilt_p_gain < self._MIN_TILT_GAIN:
            self._tilt_p_gain = self._MIN_TILT_GAIN

    def _bbox_info_cb(self, q_detection: Any, target_gesture: str) -> None:
        while True:
            detections = q_detection.get()
            print(detections)
            dect_x, dect_y, dect_w, dect_h = detections[0]
            if not target_gesture:
                gesture_result = None
            else:
                gesture_result = detections[1]

            if (gesture_result == target_gesture):
                self._bbox_x = dect_x
                self._bbox_y = dect_y

                self._bbox_width = dect_w
                self._bbox_height = dect_h

                self._set_goal_pos(
                    self._bbox_x + self._bbox_width / 2,
                    self._bbox_y + self._bbox_height / 2,
                )
                self._calc_p_gain()

    def _set_goal_pos(self, bbox_x: float, bbox_y: float) -> None:
        global pan_target_angle
        global tilt_target_angle
        if bbox_x >= 1000:
            bbox_x = 0
        if bbox_y >= 1000:
            bbox_y = 0
        pan_error = -(bbox_x + self._pan_dev - self._H_PIX_WIDTH / 2.0) / (
            self._H_PIX_WIDTH / 2.0
        )  # -1 ~ 1
        tilt_error = -(bbox_y + self._tilt_dev - self._H_PIX_HEIGHT / 2.0) / (
            self._H_PIX_HEIGHT / 2.0
        )  # -1 ~ 1

        if abs(pan_error) > self._PAN_THRESHOLD and not (bbox_x == self._old_bbox_x):
            pan_target_angle += self._pan_p_gain * pan_error + self._pan_offset
        if pan_target_angle < self._PAN_POS_MIN:
            pan_target_angle = self._PAN_POS_MIN
        elif pan_target_angle > self._PAN_POS_MAX:
            pan_target_angle = self._PAN_POS_MAX
        if abs(tilt_error) > self._TILT_THRESHOLD and not (bbox_y == self._old_bbox_y):
            tilt_target_angle += self._tilt_p_gain * tilt_error + self._tilt_offset
        if tilt_target_angle < self._TILT_POS_MIN:
            tilt_target_angle = self._TILT_POS_MIN
        elif tilt_target_angle > self._TILT_POS_MAX:
            tilt_target_angle = self._TILT_POS_MAX

        self._old_bbox_x = bbox_x
        self._old_bbox_y = bbox_y


# 手のひら検出をする関数
def HandRecognition(q_detection: Any) -> None:
    from depthai_hand_tracker.HandTrackerEdge import HandTracker

    tracker = HandTracker(use_gesture=True)

    renderer = HandTrackerRenderer(tracker=tracker)

    palm_detection = (0, 0, 0, 0)
    gesture_result = ""

    while True:
        frame, hands, bag = tracker.next_frame()
        if frame is None:
            break

        for hand in hands:
            palm_detection = (
                int(hand.landmarks[0][0] + (hand.landmarks[9]
                    [0] - hand.landmarks[0][0]) / 2),
                int(hand.landmarks[0][1] + (hand.landmarks[9]
                    [1] - hand.landmarks[0][1]) / 2),
                abs(hand.landmarks[9][0] - hand.landmarks[0][0]),
                abs(hand.landmarks[9][0] - hand.landmarks[0][0])
            )
            gesture_result = hand.gesture
            q_detection.put((palm_detection, gesture_result))

        # Draw hands
        frame = renderer.draw(frame, hands, bag)
        key = renderer.waitKey(delay=1)
        if key == 27 or key == ord('q'):
            break
    renderer.exit()
    tracker.exit()


def main() -> None:
    args = get_args()
    target_gesture = args.gesture

    q_detection: Any = Queue()
    hand_tracker = HandTracker()
    direction_updater = DirectionUpdater()

    # Threadの設定
    t1 = threading.Thread(target=HandRecognition, args=(q_detection,))
    t2 = threading.Thread(target=direction_updater._bbox_info_cb,
                          daemon=True, args=(q_detection, target_gesture))
    t3 = threading.Thread(target=hand_tracker._tracker, daemon=True)

    # Threadの動作
    t1.start()
    t2.start()
    t3.start()
    t1.join(timeout=1)
    t2.join(timeout=1)
    t3.join(timeout=1)


if __name__ == "__main__":
    main()
