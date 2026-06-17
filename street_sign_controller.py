import importlib


ALLOWED_SIGNS = {"left", "right", "stop"}


class StreetSignDetectorAdapter:
    """
    Optional adapter for an external sign detector.

    Supported detector shapes:
    - module function: detect_sign(frame) -> "left" | "right" | "stop" | None
    - class instance: StreetSignDetector().detect(frame) -> same values
    """

    def __init__(self, enabled=True):
        self.enabled = enabled
        self.available = False
        self._callable = None
        self.load_error = None

        if not enabled:
            return

        candidate_modules = [
            "street_sign_detector",
            "street_sign.street_sign_detector",
            "signRecognitionPi",
        ]

        for module_name in candidate_modules:
            try:
                module = importlib.import_module(module_name)
            except ImportError:
                continue
            except Exception as exc:
                self.load_error = f"{module_name} import failed: {exc}"
                continue

            detect_fn = getattr(module, "detect_sign", None)
            if callable(detect_fn):
                self._callable = detect_fn
                self.available = True
                return

            detector_cls = getattr(module, "StreetSignDetector", None)
            if detector_cls is not None:
                try:
                    detector = detector_cls()
                except Exception as exc:
                    self.load_error = f"{module_name}.StreetSignDetector failed: {exc}"
                    continue
                detect_method = getattr(detector, "detect", None)
                if callable(detect_method):
                    self._callable = detect_method
                    self.available = True
                    return

    def detect(self, frame):
        if not self.available or self._callable is None:
            return None

        try:
            result = self._callable(frame)
        except Exception:
            return None
        if not isinstance(result, str):
            return None

        normalized = result.strip().lower()
        if normalized == "none":
            return None

        if normalized.endswith("_far"):
            normalized = normalized[:-4]

        if normalized in ALLOWED_SIGNS:
            return normalized

        return None


class StreetSignController:
    def __init__(self, settings):
        self.cfg = settings
        self.state = "idle"
        self.state_started_at = 0.0
        self.pending_sign = None
        self.pending_started_at = 0.0
        self.pending_last_seen_at = 0.0
        self.pending_confirmations = 0
        self.pending_ready = False
        self.last_completed_at = -1e9

    def _in_cooldown(self, now):
        return (now - self.last_completed_at) < self.cfg["SIGN_COOLDOWN"]

    def _transition(self, state, now):
        self.state = state
        self.state_started_at = now

    def _clear_pending(self):
        self.pending_sign = None
        self.pending_started_at = 0.0
        self.pending_last_seen_at = 0.0
        self.pending_confirmations = 0
        self.pending_ready = False

    def _build_response(self, action, override, speed=None, steering_angle=None):
        return {
            "action": action,
            "override": override,
            "speed": speed,
            "steering_angle": steering_angle,
        }

    def _execution_response(self, action):
        if action == "left":
            return self._build_response(
                action="turn_left_sign",
                override=True,
                speed=self.cfg["SIGN_TURN_SPEED"],
                steering_angle=self.cfg["SIGN_LEFT_ANGLE"],
            )

        if action == "right":
            return self._build_response(
                action="turn_right_sign",
                override=True,
                speed=self.cfg["SIGN_TURN_SPEED"],
                steering_angle=self.cfg["SIGN_RIGHT_ANGLE"],
            )

        return self._build_response(
            action="stop_sign",
            override=True,
            speed=0,
            steering_angle=0,
        )

    def update(self, now, detected_sign=None):
        if self.state == "executing_left":
            if now - self.state_started_at >= self.cfg["SIGN_TURN_TIME"]:
                self._transition("recovering", now)
            else:
                return self._execution_response("left")

        if self.state == "executing_right":
            if now - self.state_started_at >= self.cfg["SIGN_TURN_TIME"]:
                self._transition("recovering", now)
            else:
                return self._execution_response("right")

        if self.state == "executing_stop":
            if now - self.state_started_at >= self.cfg["SIGN_STOP_HOLD_TIME"]:
                self.last_completed_at = now
                self._transition("idle", now)
                self._clear_pending()
            else:
                return self._execution_response("stop")

        if self.state == "recovering":
            if now - self.state_started_at >= 0.25:
                self.last_completed_at = now
                self._transition("idle", now)
                self._clear_pending()
            else:
                return self._build_response(
                    action="recover_sign_turn",
                    override=True,
                    speed=self.cfg["SIGN_TURN_SPEED"],
                    steering_angle=0,
                )

        if self._in_cooldown(now):
            self._clear_pending()
            return self._build_response(action="clear", override=False)

        if detected_sign is not None and detected_sign not in ALLOWED_SIGNS:
            detected_sign = None

        if (
            detected_sign is None
            and self.pending_sign is not None
            and (now - self.pending_last_seen_at) > self.cfg["SIGN_PENDING_TIMEOUT"]
        ):
            self._clear_pending()
            return self._build_response(action="clear", override=False)

        if detected_sign is not None and detected_sign != self.pending_sign:
            self.pending_sign = detected_sign
            self.pending_started_at = now
            self.pending_last_seen_at = now
            self.pending_confirmations = 1
            self.pending_ready = False
            return self._build_response(action=f"pending_{detected_sign}", override=False)

        if detected_sign is not None and detected_sign == self.pending_sign:
            self.pending_last_seen_at = now
            self.pending_confirmations += 1
            if self.pending_confirmations >= self.cfg["SIGN_CONFIRM_FRAMES"]:
                self.pending_ready = True

        if self.pending_sign is None:
            return self._build_response(action="clear", override=False)

        if not self.pending_ready:
            return self._build_response(action=f"pending_{self.pending_sign}", override=False)

        if now - self.pending_started_at < self.cfg["SIGN_ACTION_DELAY"]:
            return self._build_response(action=f"pending_{self.pending_sign}", override=False)

        latched_sign = self.pending_sign

        if latched_sign == "left":
            self._transition("executing_left", now)
            return self._execution_response("left")

        if latched_sign == "right":
            self._transition("executing_right", now)
            return self._execution_response("right")

        self._transition("executing_stop", now)
        return self._execution_response("stop")
