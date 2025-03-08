# PID class

```cpp
PID::PID(double init_kP, double init_kI, double init_kD);
```

Takes initial P, I, and D constants.  
Creates a PID object for using PID control loops.

---

```cpp
double PID::pid_adjust(double setpoint, double current_value);
```

Takes a target value and the current value.  
Returns the modifier for a PID control loop.

**P(roportional)**: distance to target (error). Basically, if setpoint is far away, change the current value faster.  
**I(ntegral)**: Error buildup. If error keeps getting bigger, change current value faster.  
**D(erivative)**: Rate of change of error. If error is oscillating around the setpoint, change current value slower.

---

```cpp
void PID::tune_X(float stick_mod);
```

Takes the scaling value for controller (so that a human can change the values slow enough to use).  
Tunes the X (either kP, kI, or kD) value.

---

```cpp
float PID::get_const(char constant);
```

Takes the character of the constant.  
Returns the current value of the constant.

# drive_straight()

```cpp
void drive_straight(float inches, float target_ips, float ips_per_sec);
```

Takes inches to travel, maximum inches/second, and acceleration in inches/sec/sec.  
Moves the drive along one heading according to those values.

### Explanation

```cpp
float dir_mod = (inches > 0) ? 1 : -1;
```

Fancy way of saying `if (inches > 0) dir_mod = 1; else dir_mod = -1;`

Used to make sure the robot travels in the same direction as distance. The same thing could be accomplished by passing negative velocity and acceleration values.

---

```cpp
if (std::abs(pos) + stop_dist(ips, ips_per_sec) >= std::abs(inches))
    ips -= ips_per_sec / 50;    // 50 hz
else if (ips < target_ips)
    ips += target_ips / 50;
else ips = target_ips;
```

Handles acceleration

If the current distance travelled plus the distance needed to stop exceeds the target distance, slow down. Otherwise, if the current speed is less than target speed, speed up. Otherwise, set speed to the target speed (to prevent moving faster than the target).

---

```cpp
pos += ips / 50 * dir_mod;    // divide by 50 bc loop occurs 50 times a second; dir_mod adjusts for fwd/bwd
```

If speed is 50 inches per second, then after 1 second, position should have increased 50 inches. Likewise, after a 50th of a second, speed should have increased by a 50th of 50 inches. This tracks the target position, accounting for the fact that the loop iterates every 20 milliseconds.

---

```cpp
pid_adjustment_l = pid_drive_l.pid_adjust(pos, pos_l);
pid_adjustment_r = pid_drive_r.pid_adjust(pos, pos_r);
```

Figure out how we should change what values are sent to the motors, based on the target position and actual position of the motors.

---

```cpp
pid_adjustment_dir = pid_dir.pid_adjust(target_heading, ROTATION * GYRO_CORRECTION);
```

Same as prior, but based on the direction the robot is facing. The value of the inertial sensor is multiplied by some constant, because when turning 360 degrees, it actually reads something like 358 degrees. The correction constant will be different for every sensor (I think).

---

```cpp
target_time += 20;
while (sands_of_time.time(vex::msec) < target_time);   // wait for next iteration;
```

Tate used this because he thought it was more accurate than using `vex::wait()`. Either he was wrong, or the function has since been updated, because there doesn't seem to be difference.

# drive_turn()

```cpp
void drive_turn(float degrees, float outer_radius, float target_ips, float ips_per_sec, bool reversed);
```

Takes the degrees to turn, the outer radius of the turn (inches), the speed of the turn (in/sec), the acceleration of the turn (in/sec/sec), and whether or not to arc backwards.
Turns in an arc with radius `outer_radius`.

### Explanation

```cpp
target_heading += degrees;
```

To maintain the target heading between function calls, it is stored as a global variable. This adds to the heading the relative number of degrees needed to turn.

---

```cpp
int dir_mod = (degrees > 0) ? 1 : -1;
```

Used instead of `std::abs()` because I forgot that function existed when I wrote this. Could easily be replaced.

---

```cpp
degrees_remaining = target_heading - ROTATION * GYRO_CORRECTION;
```

The number of degrees left in the turn. If I want to go 90°, and I've turned 30° so far, this is 60°.

---

```cpp
if (std::abs(degrees_remaining / RAD__DEG * outer_radius) - stop_dist(ips, ips_per_sec) <= 0)
    ips -= ips_per_sec / 50.0;
else if (ips < target_ips)
    ips += ips_per_sec / 50.0;   // 50 cycles per second
else ips = target_ips;
```

Handles acceleration.

First, `degrees_remaining` is converted to radians to get the number of radii left in the turn. Then it gets multiplied by the length of one radius to get the number of inches left in the turn. Other than that, it follows the logic of drive_straight().

---

```cpp
outer_vel_rpm = ips / DRIVE_REV__IN * 60 * ((reversed) ? -1 : 1);         // fixes the stupid thing;
inner_vel_rpm = outer_vel_rpm * radius_ratio * ((reversed) ? -1 : 1);     // try not to touch
```

Converts inches per second to rpm.  
The conditional checks if the robot should be moving backwards, and if it should be, it makes the rpm negative.

---

```cpp
outer_pos += ips / 50;
inner_pos = outer_pos * radius_ratio;
```

Updates the distance each side should have traveled. Same concept as `drive_straight()`.  
The inner circle of the turn is a fraction of the outer circle, represented by the ratio of their radii. To convert the distance travelled as a percentage along one circle, we take the inches travelled and multiply by the radius ratio.

---

```cpp
if ((reversed && degrees > 0) || (!reversed && !(degrees > 0))) {      // left is inner side
    pid_adjustment_l = -1 * pid_drive_l.pid_adjust(inner_pos, pos_l);
    pid_adjustment_r = pid_drive_r.pid_adjust(outer_pos, pos_r);

    drive_l.spin(DIR_FWD, dir_mod * inner_vel_rpm + pid_adjustment_l, VEL_RPM);
    drive_r.spin(DIR_FWD, dir_mod * outer_vel_rpm + pid_adjustment_r, VEL_RPM);
}
else {      // right is inner side
    pid_adjustment_l = pid_drive_l.pid_adjust(outer_pos, pos_l);
    pid_adjustment_r = -1 * pid_drive_r.pid_adjust(inner_pos, pos_r);

    drive_l.spin(DIR_FWD, outer_vel_rpm + pid_adjustment_l, VEL_RPM);
    drive_r.spin(DIR_FWD, inner_vel_rpm + pid_adjustment_r, VEL_RPM);
}
```

The if statement decides which side of the drive is on the outside.  
To determine the drive speeds and PID adjustments, it basically follows the same logic as `drive_straight()`.
