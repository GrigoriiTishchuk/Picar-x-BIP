# Pseudocode

## Main Loop

```text
initialize car hardware
initialize camera
initialize PID controller
initialize lane detector
initialize obstacle controller
initialize street sign controller
initialize optional street sign detector

loop forever:
    now = current_time()

    obstacle_control = handle_obstacle(now)
    if obstacle_control.override:
        continue

    sign_control = handle_street_sign(now, detected_sign=None)
    if sign_control.override:
        continue

    frame = camera.get_frame()
    if frame is missing:
        stop car
        continue

    detected_sign = street_sign_detector.detect(frame)
    sign_control = handle_street_sign(now, detected_sign)
    if sign_control.override:
        continue

    lane_result = lane_detector.detect(frame)
    if lane_result.detected is false:
        reset PID
        search for lane
        continue

    speed = choose_speed(lane_result, obstacle_control.speed_limit)
    steering_angle = update_steering(lane_result.error)
    drive forward at speed
```

## Obstacle Handling

```text
read ultrasonic distance
sanitize invalid readings
append valid readings to short history
filtered_distance = median(history)

if currently executing avoid-left:
    keep steering left until avoid turn timer ends
    then switch to avoid-right

if currently executing avoid-right:
    keep steering right until avoid turn timer ends
    then switch to return-to-lane

if currently returning to lane:
    hold straight steering briefly
    then switch to clear

if raw distance is extremely close:
    emergency stop immediately

otherwise decide using filtered distance:
    if distance <= stop threshold for enough frames:
        start avoid-left maneuver
    else if distance <= brake threshold:
        limit speed to slow speed
    else if distance <= slow threshold:
        limit speed to slow speed
    else:
        clear obstacle state
```

## Street Sign Handling

```text
possible detector outputs:
    "left"
    "right"
    "stop"
    none

if sign controller is already executing a sign action:
    if executing left:
        steer hard left at sign turn speed for sign turn time
        then go to short recovery state

    if executing right:
        steer hard right at sign turn speed for sign turn time
        then go to short recovery state

    if executing stop:
        keep car stopped for sign stop hold time
        then enter cooldown

    if recovering:
        hold straight steering briefly
        then enter cooldown

if in cooldown:
    ignore new sign detections

if new sign is detected:
    if it is different from the pending sign:
        start a new pending sign
        record pending start time
        set confirmation count to 1
    else:
        increment confirmation count

    if confirmation count is high enough
    and pending delay has expired:
        if sign == left:
            execute 90-degree left sign turn
        if sign == right:
            execute 90-degree right sign turn
        if sign == stop:
            execute stop sign behavior

if no sign is seen for too long while pending:
    clear pending sign
```

## Lane Detection

```text
input = camera frame

convert frame to a binary white-lane mask:
    blur image
    convert to HLS and HSV
    threshold for bright, low-saturation white regions
    combine masks
    apply morphology to remove noise and fill gaps

crop to lower region of interest

for each scanline inside the ROI:
    find white pixels
    split into left-half and right-half candidates
    choose inner left border and inner right border

    if one border is missing:
        estimate it using previous lane width

    if lane width looks valid:
        compute lane center point for that scanline

if fewer than 2 center points exist:
    report lane not detected

fit a line through the center points
predict lane center at bottom of frame

offset_error = image_center - predicted_lane_center
angle_error = lane direction angle from line slope

combined_error =
    offset_weight * offset_error
    - angle_weight * angle_error

smooth combined_error over time

compute confidence from:
    how many scanlines produced valid points
    how well those points fit the line

if too few points and confidence is too low:
    report lane not detected

otherwise return:
    detected = true
    error = combined_error
    offset_error
    angle_error
    confidence
```

## PID Steering

```text
dt = time since previous frame
integral += error * dt
clamp integral to safe limit

derivative = (error - previous_error) / dt
smooth derivative

raw_output =
    kp * error
    + ki * integral
    + kd * derivative

clamp steering to steering limits
smooth steering with previous steering angle
limit how far steering can change in one loop
send steering command to servo
```

## Speed Selection

```text
if lane is not detected:
    use search speed
else if lane confidence is low:
    use low-confidence speed
else if lane error or lane angle implies a corner:
    use corner speed
else:
    use normal speed

if obstacle controller provides a lower speed limit:
    cap speed at that lower value
```

## Priority Order

```text
1. Obstacle emergency / obstacle override
2. Active street sign maneuver
3. Normal lane following
```
