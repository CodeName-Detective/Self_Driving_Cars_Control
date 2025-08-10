# CARLA 2D Waypoint Follower Controller
![CARLA Output GIF](media/cover.gif)

This repository contains a 2D longitudinal and lateral controller implementation for a vehicle simulated in the [CARLA Simulator](https://carla.org/). The controller is designed to follow a predefined set of waypoints with associated target speeds, and serves as a core component of a waypoint-following autonomous driving demo.

## Controller Overview

The `Controller2D` class handles two key aspects of vehicle control:

* **Longitudinal control**: Adjusts throttle and brake to match desired speeds.
* **Lateral control**: Adjusts the steering angle to minimize path deviation.

### 🔧 Technologies Used

* Python 3
* NumPy
* CARLA Simulator

---

## 🔁 Longitudinal Control: PID Controller

The throttle and brake commands are computed using a **PID controller** based on the velocity error between the current speed and the desired speed extracted from the closest waypoint.

PID formula:

```
a_desired = Kp * error + Ki * integral_error + Kd * derivative_error
```

* **Throttle** is applied if desired acceleration is positive.
* **Brake** is applied if deceleration is needed.
* Outputs are smoothed using `tanh` to prevent aggressive spikes.

### Tunable Parameters:

* `Kp = 1.0`
* `Ki = 0.2`
* `Kd = 0.01`

---

## 🛞 Lateral Control: Stanley Controller

The steering control is based on the **Stanley method**, using two components:

* **Heading Error**: Angular difference between vehicle yaw and path direction.
* **Cross-Track Error**: Lateral distance from the vehicle to the path.

The total steering angle is computed as:

```
steer = heading_error + arctan(K_gain * cross_track_error / velocity)
```

### Tunable Parameters:

* `K_gain = 0.3` (Stanley control gain)
* Output steering is clamped to vehicle limits: `[-1.22, 1.22]` radians

---

## 📌 Waypoint Format

Waypoints should be passed as a list of `[x, y, v_desired]`, where:

* `x`, `y`: global coordinates of the waypoint
* `v_desired`: desired speed (m/s) at that point


---

## Ouputs
[CARLA Video](media/video.mp4)

