# Picar-x-BIP

Autonomous driving prototype for a `Picarx` robot car with:
- camera-based lane following
- ultrasonic obstacle handling
- optional street-sign actions for `left`, `right`, and `stop`

The current main entry point is [PID_camera_lane_angle_following.py]

## What It Does

The car runs a continuous control loop that:
- reads the ultrasonic sensor and applies obstacle safety logic first
- optionally reacts to detected street signs
- reads a camera frame
- detects the lane from white borders on a dark track
- computes a steering error from lane offset and lane angle
- uses PID to steer smoothly
- chooses speed based on turn sharpness and lane confidence

Priority order:
- obstacle override
- active street-sign maneuver
- normal lane following

## Repo Layout

- [PID_camera_lane_angle_following.py]
  Main driving script.
- [object_avoidance/object_avoidance.py]
  Ultrasonic obstacle state machine.
- [object_avoidance/object_avoidance_config.py]
  All tunable config values.
- [street_sign_controller.py]
  Optional sign-detector adapter plus sign action state machine.
- [signRecognitionPi.py]
  Sign recognition module for `left`, `right`, and `stop`.
- [PSEUDOCODE.md]
  Algorithmic walkthrough.
- [object_avoidance/test_obstacle_avoidance.py]
  Lightweight obstacle behavior test.

## Requirements

You need:
- a `Picarx` car with working motor/servo/ultrasonic access
- Python 3
- `picarx`
- `opencv-python` / `cv2`
- `numpy`
- a working camera available to OpenCV

If you want the included sign-recognition module, you also need:
- `picamera2`
- `pyserial`

## Running

Start the car with:

```bash
python3 PID_camera_lane_angle_following.py
```

Stop it with:

```bash
Ctrl+C
```

On shutdown the script stops the car and releases the camera.

## Camera Notes

The script currently opens:

```python
CameraReader(camera_index=0)
```

That means OpenCV will try to use camera device `0`. If the wrong camera opens, change the index in [PID_camera_lane_angle_following.py]

## Optional Street Sign Detector

Street-sign handling is optional.

If no detector module exists, the car still runs and simply ignores signs.

The adapter in [street_sign_controller.py] supports either:

```python
def detect_sign(frame) -> str | None
```

or:

```python
class StreetSignDetector:
    def detect(self, frame) -> str | None
```

Valid outputs are:
- `"left"`
- `"right"`
- `"stop"`
- `None`

This repo now includes [signRecognitionPi.py], and the adapter will automatically try to load it.

That module exposes:

```python
def detect_sign(frame)
```

and may return:
- `"left"`
- `"right"`
- `"stop"`
- `"left_far"`
- `"right_far"`
- `"stop_far"`
- `"none"`

The adapter normalizes those values for the main car controller:
- `"_far"` suffixes are stripped
- `"none"` becomes `None`

Important behavior:
- when imported by the main driving script, `signRecognitionPi.py` no longer starts its own camera/serial loop
- when run directly with `python3 signRecognitionPi.py`, it still runs as a standalone detector process

## Current Tuning Assumption

The latest config is tuned as a cautious first pass for:
- a dark track mat
- bright white lane borders
- narrow indoor course geometry
- sharp corners
- side-mounted street signs near the thick boundary lines

You will still need real-world calibration.

## Most Important Config Values

Lane and steering:
- `WHITE_HLS_LIGHTNESS_MIN`
- `WHITE_HSV_VALUE_MIN`
- `EXPECTED_LANE_WIDTH`
- `KP`
- `KD`
- `CORNER_ERROR_THRESHOLD`
- `CORNER_ANGLE_THRESHOLD_DEG`

Signs:
- `SIGN_ACTION_DELAY`
- `SIGN_TURN_TIME`
- `SIGN_TURN_SPEED`

Obstacle handling:
- `STOP_DISTANCE`
- `BRAKE_DISTANCE`
- `SLOW_DISTANCE`

## Known Limitations

- Final behavior still depends heavily on track lighting and camera placement.
- Sign timing is approximate until tested on the real course.
- The lane model is still a simple line fit, not a full curve model.
- Obstacle recovery is intentionally conservative after close readings.
- Hardware startup still depends on the `Picarx` stack and camera working correctly on the target machine.
- The included sign detector still requires `picamera2` and `serial` imports to be available on the target machine.
