# within_range()

```cpp
bool within_range(double value, double base, double range);
```

Takes a value to check, a base (center of the range), and a range (extending on either side of the base).  
Returns `true` if the `value` is within `range` units from the `base`.

# stop_dist()

```cpp
float stop_dist(float current_vel, float accel, float target_vel);
```

Takes a speed, acceleration, and a speed to try to reach.  
Returns the distance it will take to reach the `target_vel` if we start at `current_vel` and accelerate by `accel`.

# side_pressed()

```cpp
int *side_pressed(void);
```

Returns an array. Index zero returns which horizontal (left-right) side was pressed. Index 1 returns which vertical (up-down) side was pressed.

# tune_gyro()

```cpp
void tune_gyro(void);
```

Debug function. Should not be called during match programs.

Calibrates the gyro and prints the current heading to the brain.
