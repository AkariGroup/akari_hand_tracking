#!/usr/bin/env python3

from depthai_hand_tracker.HandTrackerRenderer import HandTrackerRenderer
import argparse
import threading
import time
from queue import Queue
from time import sleep
from typing import Any
from akari_client import AkariClient
import sys

sys.path.append("depthai_hand_tracker")

VIDEO_WIDTH, VIDEO_HEIGHT = 1152, 648

pan_target_angle = 0.0
tilt_target_angle = 0.0


def get_args() -> Any:
    parser = argparse.ArgumentParser()
    parser.add_argument('-e', '--edge', action="store_true",
                        help="Use Edge mode (postprocessing runs on the device)")
    parser_tracker = parser.add_argument_group("Tracker arguments")
    parser_tracker.add_argument('-i', '--input', type=str,
                        help="Path to video or image file to use as input (if not specified, use OAK color camera)")
    parser_tracker.add_argument("--pd_model", type=str,
                        help="Path to a blob file for palm detection model")
    parser_tracker.add_argument('--no_lm', action="store_true",
                        help="Only the palm detection model is run (no hand landmark model)")
    parser_tracker.add_argument("--lm_model", type=str,
                        help="Landmark model 'full', 'lite', 'sparse' or path to a blob file")
    parser_tracker.add_argument('--use_world_landmarks', action="store_true",
                        help="Fetch landmark 3D coordinates in meter")
    parser_tracker.add_argument('-s', '--solo', action="store_true",
                        help="Solo mode: detect one hand max. If not used, detect 2 hands max (Duo mode)")
    parser_tracker.add_argument('-xyz', "--xyz", action="store_true",
                        help="Enable spatial location measure of palm centers")
    parser_tracker.add_argument('-g', '--gesture',
                        choices=["ONE", "TWO", "THREE", "FOUR", "FIVE", "OK", "PEACE", "FIST"], help="Specify gestures to track")
    parser_tracker.add_argument('-c', '--crop', action="store_true",
                        help="Center crop frames to a square shape")
    parser_tracker.add_argument('-f', '--internal_fps', type=int,
                        help="Fps of internal color camera. Too high value lower NN fps (default= depends on the model)")
    parser_tracker.add_argument("-r", "--resolution", choices=['full', 'ultra'], default='full',
                        help="Sensor resolution: 'full' (1920x1080) or 'ultra' (3840x2160) (default=%(default)s)")
    parser_tracker.add_argument('--internal_frame_height', type=int,
                        help="Internal color camera frame height in pixels")
    parser_tracker.add_argument("-lh", "--use_last_handedness", action="store_true",
                        help="Use last inferred handedness. Otherwise use handedness average (more robust)")
    parser_tracker.add_argument('--single_hand_tolerance_thresh', type=int, default=10,
                        help="(Duo mode only) Number of frames after only one hand is detected before calling palm detection (default=%(default)s)")
    parser_tracker.add_argument('--dont_force_same_image', action="store_true",
                        help="(Edge Duo mode only) Don't force the use the same image when inferring the landmarks of the 2 hands (slower but skeleton less shifted)")
    parser_tracker.add_argument('-lmt', '--lm_nb_threads', type=int, choices=[1, 2], default=2,
                        help="Number of the landmark model inference threads (default=%(default)i)")
    parser_tracker.add_argument('-t', '--trace', type=int, nargs="?", const=1, default=0,
                        help="Print some debug infos. The type of info depends on the optional argument.")
    parser_renderer = parser.add_argument_group("Renderer arguments")
    parser_renderer.add_argument('-o', '--output', help="Path to output video file")
    return parser.parse_args()


# 顔追従するクラス
class FaceTracker:
    """face tracking class"""

    def __init__(self) -> None:
        global pan_target_angle
        global tilt_target_angle

        # AkariClientのインスタンスを作成する
        self.akari = AkariClient()
        # 関節制御用のインスタンスを取得する
        self.joints = self.akari.joints

        self._default_x = 0
        self._default_y = 0

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


class DirectionUpdater:
    """Update direction from face info"""

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

    def __init__(self) -> None:
        global prev_time
        global cur_time
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
            pan_target_angle += self._pan_p_gain * pan_error
        if pan_target_angle < self._PAN_POS_MIN:
            pan_target_angle = self._PAN_POS_MIN
        elif pan_target_angle > self._PAN_POS_MAX:
            pan_target_angle = self._PAN_POS_MAX
        if abs(tilt_error) > self._TILT_THRESHOLD and not (bbox_y == self._old_bbox_y):
            tilt_target_angle += self._tilt_p_gain * tilt_error
        if tilt_target_angle < self._TILT_POS_MIN:
            tilt_target_angle = self._TILT_POS_MIN
        elif tilt_target_angle > self._TILT_POS_MAX:
            tilt_target_angle = self._TILT_POS_MAX

        self._old_bbox_x = bbox_x
        self._old_bbox_y = bbox_y


def HandRecognition(q_detection: Any, args: Any) -> None:
    dargs = vars(args)
    tracker_args = {a: dargs[a] for a in ['pd_model', 'lm_model', 'internal_fps', 'internal_frame_height'] if dargs[a] is not None}

    if args.edge:
        from depthai_hand_tracker.HandTrackerEdge import HandTracker
        tracker_args['use_same_image'] = not args.dont_force_same_image
    else:
        from depthai_hand_tracker.HandTracker import HandTracker

    tracker = HandTracker(
        input_src=args.input,
        use_lm=not args.no_lm,
        use_world_landmarks=args.use_world_landmarks,
        use_gesture=True,
        xyz=args.xyz,
        solo=args.solo,
        crop=args.crop,
        resolution=args.resolution,
        stats=True,
        trace=args.trace,
        use_handedness_average=not args.use_last_handedness,
        single_hand_tolerance_thresh=args.single_hand_tolerance_thresh,
        lm_nb_threads=args.lm_nb_threads,
        **tracker_args
        )

    renderer = HandTrackerRenderer(
            tracker=tracker,
            output=args.output)

    palm_detection = (0, 0, 0, 0)
    gesture_result = ""

    while True:
        # Run hand tracker on next frame
        # 'bag' contains some information related to the frame
        # and not related to a particular hand like body keypoints in Body Pre Focusing mode
        # Currently 'bag' contains meaningful information only when Body Pre Focusing is used
        frame, hands, bag = tracker.next_frame()
        if frame is None:
            break

        for hand in hands:
            palm_detection = (
                int(hand.landmarks[0][0] + (hand.landmarks[9][0] - hand.landmarks[0][0]) / 2),
                int(hand.landmarks[0][1] + (hand.landmarks[9][1] - hand.landmarks[0][1]) / 2),
                abs(hand.landmarks[9][0] - hand.landmarks[0][0]),
                abs(hand.landmarks[9][0] - hand.landmarks[0][0])
                )
            # wrist_point = (hand.landmarks[0][0], hand.landmarks[0][1])
            gesture_result = hand.gesture
            # print(wrist_point)
            # print(gesture_result)
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

    face_tracker = FaceTracker()
    direction_updater = DirectionUpdater()

    t1 = threading.Thread(
        target=HandRecognition,
        args=(
            q_detection,
            args,
        ),
    )
    t2 = threading.Thread(target=direction_updater._bbox_info_cb, args=(q_detection, target_gesture))
    t3 = threading.Thread(target=face_tracker._tracker)
    t1.start()
    t2.start()
    t3.start()
    t1.join()
    t2.join()
    t3.join()


if __name__ == "__main__":
    main()
