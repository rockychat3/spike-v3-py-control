This is a library for Spike v3 App users building a two-drive-motor, two-front-optical-sensor robot (a common FLL design) to be able to more efficiently utilize their sensors and focus team time on completing missions without directly dealing with sensor code on every student action (which generally leads to either no use of sensors or code monopolization by 1-2 students).  The code here is fairly short and very possible to dive deeper into with students.  It also mirrors real software practice (use of reusable, shareable libraries) and gives all students a chance to learn about, and even participate in, open source development.

The initial working library was fully developed with GPT5 in a standard chatGPT thread, and later migrated into VS Code with the Continue extension to more efficiently address known bugs and unhelpful conventions.  This was guided by a FLL mentor with very mediocre software development skills, user beware.  It is hosted on Github with a MIT license, which essentially means that any team can take this code and use it as they wish.  It would be greatly appreciated if, after using and learning the code, other teams (students and mentors) can contribute back improvements, bug fixes, and new ideas to make this an awesome control library for driving and mechanisms.

To push the code to the Spike Hub, use uploader.llsp3 -- this is Python code that runs on slot 19 to simply upload the text of the robot_control.py code.  To do this, you need to manually copy and paste the text of the main code where indicated in this file in the triple quotes, then click the run button in the Spike app.

## Public API (Spike App v3, Python)

All calls are async and should be awaited. Use runloop.run(main) to run your async main.

- Import: `import robot_control as rc`
- Typical ports (adjust to your build):
  - Left drive motor: 'B'
  - Right drive motor: 'A'
  - Left front color sensor: 'F'
  - Right front color sensor: 'E'

### Installation to the SPIKE Hub

Option A: One-time installer (recommended for classroom use)
1) Open uploader.llsp3 in the SPIKE App (slot 19 works well).
2) Copy the entire contents of robot_control.py into the triple-quoted robot_control_code string inside uploader.llsp3.
3) Run uploader.llsp3. You should see "Installed /flash/robot_control.py ...".
4) After that, any new program can simply `import robot_control as rc` and use the library.

Option B: Copy robot_control.py directly
- If your workflow allows, save robot_control.py into the hub's /flash folder as robot_control.py.

### Quick start example

```python example.llsp3
import runloop
import robot_control as rc

async def main():
    drive = rc.DriveLib('B', 'A', 'F', 'E', track_width_cm=15.0)
    mech  = rc.Mechanism('C')

    rc.set_status('yellow')
    await drive.straight(0.5, base_velocity=420)
    await drive.turn('left', 90)
    await drive.arc('right', 45, radius_cm=20)
    result = await drive.align(1.0, base_velocity=420, creep_velocity=150, center_after_square=False, report=True)
    print('Align result:', result)

    await mech.rotate('right', 120)
    await mech.home_stall(direction=-1)

runloop.run(main())
```

---

### Module-level helpers (rc)

- rc.wait_button(side='left', poll_ms=10)
  - Waits until a hub button is pressed. side can be 'left' or 'right'.
  - Example: `await rc.wait_button('right')`

- rc.set_status(color)
  - Sets the hub power LED to a color. Accepts strings like 'red','green','yellow','blue','purple','white' (case-insensitive) or a color constant.
  - Example: `rc.set_status('red')`

- rc.yaw_raw()
  - Returns the current yaw in degrees (relative to the world), using the hub motion sensor.
  - Example: `print('Yaw:', rc.yaw_raw())`

### DriveLib class

Constructor
```python robot_control.py
rc.DriveLib(left_motor_port, right_motor_port, left_sensor_port, right_sensor_port,
            track_width_cm=10.0, black_threshold=0.22, white_threshold=0.90,
            left_polarity=-1, right_polarity=1)
```
- Ports can be strings like 'A','B','E','F' or hub.port values. track_width_cm is the wheel-to-wheel distance.
- Polarity lets you flip a motor direction without rewiring.

Methods (all async unless noted)
- await straight(target_total_rot, base_velocity=None, Kp=4.0, report=False)
  - Drive straight for N wheel rotations (positive forward, negative backward) with gyro trim.
  - base_velocity is in deg/sec per motor. If None, uses a tuned default.

- await turn(direction, degrees, turn_velocity=None, tol_deg=1.0, report=False)
  - Gyro-based point turn. direction: 'left' or 'right'. degrees 0..180.
  - turn_velocity caps the turn speed; tol_deg controls settle tolerance.

- await arc(direction, degrees, radius_cm, base_velocity=None, use_gyro_trim=True, report=False)
  - Drive an arc of given radius (cm) and angle. direction: 'left' or 'right'.
  - Uses simple kinematics plus optional yaw trim to stay on target.

- await align(target_rotations, base_velocity=None, creep_velocity=None, Kp=4.0, center_after_square=False, report=False)
  - High-reliability line acquire and square routine:
    1) Go straight close to target, 2) creep and detect the black line, 3) long slow back-off, 4) independent ultra-slow re-entry until both sensors rest on black, 5) optional edge-centering.
  - Returns dict: {'found': bool, 'first_hit': 'L'|'R'|None, 'rotations_to_line': float}
  - center_after_square=True attempts a final white/black edge center.

- await stop_hard()
  - Immediate stop of both motors.

### Mechanism class (single motor axes)

Constructor
```python robot_control.py
rc.Mechanism(port, hold_at_rest=True, decel_window_deg=90.0, creep_min_vel=120)
```
- For arms/claws/etc. Uses simple trapezoidal-like profiling.

Methods
- position_deg() -> float
  - Current relative position in degrees.

- await rotate(direction, degrees, velocity=None, report=False)
  - Move by a relative angle. direction: 'left' or 'right'.

- await home_stall(direction=-1, velocity=350, min_delta_deg=1.0, window_ms=300, timeout_ms=4000, zero_at_end=True, report=False)
  - Drive toward a hard stop until movement within a time window is below min_delta_deg, then optionally zero the encoder.
  - Returns True if homed before timeout.

### Notes and tips
- All motions are async: call within an async def and await them.
- Units
  - Rotations: 1.0 = one full wheel revolution per wheel.
  - Degrees (turn): robot yaw degrees (0..180 supported per call).
  - Velocity: deg/sec sent to each motor.
- Sensors
  - align expects two color sensors mounted left and right at the front, looking down.
- Button pauses are helpful when live-testing: use rc.wait_button('left'|'right').

### Troubleshooting
- If you see import errors, make sure robot_control.py exists in /flash on the hub (run uploader.llsp3).
- If your robot drives backward, flip left_polarity/right_polarity in DriveLib(...).
- If line detect is unreliable on your mat, tweak the black/white thresholds in DriveLib constructor.
