// ============================================================================
//  MiniHexa Robot - Complete API Reference
//  IEEE SoutheastCon 2026 Competition Code
// ============================================================================
//
//  This file is a reference guide — it contains ONLY comments, no code.
//  Open it alongside your competition sketches as a cheat sheet.
//
//  All functions documented here come from the library files in each
//  sketch folder (hiwonder_robot.h, hiwonder_sensor.h, etc.)
//


// ============================================================================
//  1. QUICK START TEMPLATE
// ============================================================================
//
//  Minimal boilerplate for any new competition sketch:
//
//    #include "hiwonder_robot.h"
//
//    Robot minihexa;
//
//    Velocity_t vel = {0.0f, 0.0f, 0.0f};
//    Vector_t   pos = {0.0f, 0.0f, 0.0f};
//    Euler_t    att = {0.0f, 0.0f, 0.0f};
//
//    void setup() {
//        Serial.begin(115200);
//        minihexa.begin();   // inits servos, I2C, IMU, ADC, SPIFFS
//        delay(2000);        // let servos settle
//    }
//
//    void loop() {
//        // your code here
//    }
//
//  Arduino IDE Settings:
//    Board:            ESP32 Dev Module
//    Partition Scheme: No OTA (2MB APP / 2MB SPIFFS)
//    ESP32 Core:       3.3.7
//


// ============================================================================
//  2. CORE MOVEMENT: move()
// ============================================================================
//
//  Signature:
//    void minihexa.move(Velocity_t *vel, Vector_t *pos, Euler_t *att,
//                       uint32_t time = 600, int step_num = -1);
//
//  Parameters:
//    vel      - Body velocity (walking direction & speed)
//               .vx    = lateral (+ = right, - = left)
//               .vy    = forward/back (+ = forward, - = backward)
//               .omega = rotation (+ = CCW/left turn, - = CW/right turn)
//               Typical range: -3.0 to +3.0 for vx/vy, -2.5 to +2.5 for omega
//               Set all to 0.0 to stop walking
//
//    pos      - Body position offset from rest stance (cm)
//               .x = lateral shift (range: -4.0 to +4.0)
//               .y = forward shift (range: -4.0 to +4.0)
//               .z = height (range: -4.0 to +4.0, 0=normal, +=raise, -=lower)
//
//    att      - Body orientation (degrees)
//               .roll  = side tilt (unclamped)
//               .pitch = forward tilt (unclamped)
//               .yaw   = rotation (clamped: -20.0 to +20.0)
//
//    time     - Gait cycle period in ms (default: 600)
//               Minimum: 400ms when moving, 10ms when stopped
//               Lower = faster walking but less stable
//
//    step_num - Number of gait cycles to execute
//               -1 = run continuously until next move() call
//                0 = immediate stop
//               >0 = execute exactly N steps then auto-stop
//
//  IMPORTANT: After a finite move (step_num > 0), you MUST delay:
//    delay(step_num * time + 200);  // +200ms safety margin
//
//  Example:
//    vel = {0.0f, 3.0f, 0.0f};
//    minihexa.move(&vel, &pos, &att, 600, 5);
//    delay(5 * 600 + 200);  // wait for 5 steps to finish
//


// ============================================================================
//  3. VELOCITY REFERENCE TABLE
// ============================================================================
//
//  Tested values from omnidirectional_movement example:
//
//  Direction          | vx    | vy    | omega | Notes
//  -------------------|-------|-------|-------|---------------------------
//  Forward            |  0.0  | +3.0  |  0.0  | Straight ahead
//  Backward           |  0.0  | -3.0  |  0.0  | Straight back
//  Strafe Right       | +3.0  |  0.0  |  0.0  | Sideways right
//  Strafe Left        | -3.0  |  0.0  |  0.0  | Sideways left
//  Diagonal Fwd-Right | +2.0  | +2.0  |  0.0  | 45-degree forward-right
//  Diagonal Fwd-Left  | -2.0  | +2.0  |  0.0  | 45-degree forward-left
//  Diagonal Back-Right| +2.0  | -2.0  |  0.0  | 45-degree backward-right
//  Diagonal Back-Left | -2.0  | -2.0  |  0.0  | 45-degree backward-left
//  Turn Left (CCW)    |  0.0  |  0.0  | +2.0  | Rotate in place
//  Turn Right (CW)    |  0.0  |  0.0  | -2.0  | Rotate in place
//  Stop               |  0.0  |  0.0  |  0.0  | Halt all movement
//
//  The omnidirectional example uses: time=1800, step_num=3 per direction
//  The avoid() function uses:        time=800,  vy=2.0 for forward/back
//                                    time=1000, omega=1.8 for turning
//


// ============================================================================
//  4. POSE CONTROL (body position & tilt without walking)
// ============================================================================
//
//  Set vel to {0,0,0} and use pos/att to pose the body:
//
//  Tested values from pose_adjust example:
//
//  Position Offsets (pos):
//    pos = {+3.0,  0.0,  0.0}  — shift body right 3cm
//    pos = { 0.0, +3.0,  0.0}  — shift body forward 3cm
//    pos = {-3.0,  0.0,  0.0}  — shift body left 3cm
//    pos = { 0.0, -3.0,  0.0}  — shift body backward 3cm
//    pos = { 0.0,  0.0, +3.0}  — raise body 3cm
//    pos = { 0.0, -2.0, -1.0}  — lower and shift back
//
//  Euler Angles (att):
//    att = {+8.0,   0.0,   0.0}  — roll right 8 degrees
//    att = {-8.0,   0.0,   0.0}  — roll left 8 degrees
//    att = { 0.0, +12.0,   0.0}  — pitch forward 12 degrees
//    att = { 0.0, -12.0,   0.0}  — pitch backward 12 degrees
//    att = { 0.0,   0.0, +12.0}  — yaw left 12 degrees
//    att = { 0.0,   0.0, -12.0}  — yaw right 12 degrees
//
//  Range limits: pos x/y/z each clamped to [-4, +4] cm
//                att yaw clamped to [-20, +20] degrees
//                att roll/pitch are unclamped but keep reasonable (<20)
//
//  Example — look up and to the right:
//    vel = {0, 0, 0};
//    pos = {0, 0, 1.0};          // raise body slightly
//    att = {5.0, -8.0, -10.0};   // roll right, pitch back, yaw right
//    minihexa.move(&vel, &pos, &att, 600);
//


// ============================================================================
//  5. TIMING & STEP CONTROL RULES
// ============================================================================
//
//  - time (ms): period of one full gait cycle
//    Minimum 400ms when robot is walking (vel != 0)
//    Minimum 10ms when robot is stopped (pose-only)
//    If step amplitude exceeds max_half_step_length (3.0cm), time is
//    automatically shortened (floored at 400ms)
//
//  - step_num: number of gait cycles
//    -1 = continuous (default) — runs until next move() overrides
//     0 = immediate stop
//    >0 = finite — robot auto-stops after N cycles
//
//  - Blocking pattern for finite moves:
//    minihexa.move(&vel, &pos, &att, TIME, STEPS);
//    delay(STEPS * TIME + 200);   // MUST wait before next move()
//
//  - Stopping cleanly:
//    vel = {0, 0, 0};
//    minihexa.move(&vel, &pos, &att);
//    delay(500);   // allow stop transition
//
//  - The gait uses a tripod pattern:
//    Group A: legs 1, 3, 5 (right-front, right-back, left-mid)
//    Group B: legs 2, 4, 6 (right-mid, left-back, left-front)
//    Groups alternate swing phases
//
//  - Internal update runs on a FreeRTOS timer every 10ms (100Hz)
//


// ============================================================================
//  6. PREPROGRAMMED MOVES
// ============================================================================
//
//  --- twist() ---
//  Signature: void minihexa.twist(float radius, uint16_t circles,
//                                  uint16_t steps_per_circle,
//                                  RotationDirection dir);
//  What it does: Rocks/sways the body in a circular tilting path.
//                Traces a circle in (roll, pitch) euler space.
//  Parameters:
//    radius           — amplitude of sway in degrees (e.g. 15.0)
//    circles          — number of full revolutions (e.g. 3)
//    steps_per_circle — points per circle (e.g. 15)
//    dir              — CLOCKWISE or COUNTER_CLOCKWISE
//  Duration: ~(circles * steps_per_circle * 100ms) + 1000ms
//  Blocking: YES
//  Example:
//    minihexa.twist(15.0f, 3, 15, COUNTER_CLOCKWISE);  // ~5.5 seconds
//    minihexa.twist(15.0f, 3, 15, CLOCKWISE);           // ~5.5 seconds
//
//
//  --- acting_cute() ---
//  Signature: void minihexa.acting_cute();
//  What it does: Two-phase personality animation:
//    Phase 1: Nods forward/back 2 times (pitch +/-10 degrees, 300ms each)
//    Phase 2: Sways body left/right 2 times (x +/-2cm, 200ms each)
//  Duration: ~2500ms
//  Blocking: YES
//  Example:
//    minihexa.acting_cute();
//
//
//  --- wake_up() ---
//  Signature: void minihexa.wake_up();
//  What it does: Dramatic "waking up" sequence:
//    1. Tilt left + yaw left (hold 1 second)
//    2. Tilt left + yaw right (hold 1 second)
//    3. Return upright
//    4. Walk forward 10 steps at vy=1.5
//  Duration: ~9+ seconds
//  Blocking: YES
//  Example:
//    minihexa.wake_up();
//
//
//  --- _wake_up() ---
//  Signature: void minihexa._wake_up();
//  What it does: Short version of wake_up — just the stretch, no walking.
//    1. Tilt left + yaw left (hold 1 second)
//    2. Tilt left + yaw right (hold 1 second)
//    3. Return to neutral
//  Duration: ~2400ms
//  Blocking: YES
//  Example:
//    minihexa._wake_up();
//
//
//  --- reset() ---
//  Signature: void minihexa.reset();
//  What it does: Returns robot to neutral standing pose at pos={0,0,1}
//                (slightly elevated) over 600ms.
//  Duration: ~1200ms
//  Blocking: YES
//  Example:
//    minihexa.reset();
//
//
//  --- crawl_state() ---
//  Signature: void minihexa.crawl_state();
//  What it does: Transitions from ACTION_GROUP mode back to CRAWL (walking)
//                mode. Sends all 6 legs to rest position over 1 second.
//  Duration: ~1000ms
//  Blocking: YES (delay inside)
//  Example:
//    minihexa.crawl_state();
//


// ============================================================================
//  7. OBSTACLE AVOIDANCE: avoid()
// ============================================================================
//
//  Signature: void minihexa.avoid(uint16_t dis);
//
//  Call in a loop with the ultrasonic distance reading (mm).
//  Runs a 4-state finite state machine internally:
//
//  State    | Behavior                          | Transition
//  ---------|-----------------------------------|---------------------------
//  FORWARD  | Walk forward (vy=2.0, time=800)   | dis < 200mm → TURN
//           |                                   | dis < 100mm → BACK
//  BACK     | Walk backward (vy=-2.0, time=800) | dis 100-200mm → TURN
//           |                                   | dis > 200mm → FORWARD
//  TURN     | Turn left (omega=1.8, 4 steps)    | always → WAIT
//  WAIT     | Hold until turn finishes           | dis > 200mm → FORWARD
//           |                                   | dis 100-200mm → TURN
//           |                                   | dis < 100mm → BACK
//
//  Distance thresholds: all in millimeters
//    > 200mm  = clear, walk forward
//    100-200mm = approaching, turn away
//    < 100mm  = too close, reverse
//
//  Usage pattern:
//    void loop() {
//        uint16_t dis = minihexa.sensor.get_distance();
//        minihexa.avoid(dis);
//        delay(50);  // 20Hz polling
//    }
//
//  NOTE: avoid() has its own internal state (avoid_state). Do not mix
//  with your own move() calls — they will conflict. Use one or the other.
//


// ============================================================================
//  8. SELF-BALANCING: balance()
// ============================================================================
//
//  Signature: void minihexa.balance(bool state);
//
//  When state=true:
//    - Enables IMU timer (10ms updates)
//    - Reads roll/pitch from QMI8658 IMU via Madgwick filter
//    - Applies negated angles to body euler to counteract tilt
//    - Angles clamped to +/-18 degrees
//    - Updates every 50ms
//
//  When state=false:
//    - Disables IMU timer
//    - Stops balancing corrections
//
//  Example:
//    minihexa.balance(true);   // start balancing
//    delay(10000);             // balance for 10 seconds
//    minihexa.balance(false);  // stop balancing
//


// ============================================================================
//  9. ULTRASONIC SENSOR
// ============================================================================
//
//  I2C address: 0x77
//
//  --- get_distance() ---
//  Signature: uint16_t minihexa.sensor.get_distance();
//  Returns:   Distance in millimeters (filtered, 3-sample moving average)
//  Example:
//    uint16_t dis = minihexa.sensor.get_distance();
//    Serial.printf("Distance: %d mm\n", dis);
//
//  --- _get_distance() ---
//  Signature: uint16_t minihexa.sensor._get_distance();
//  Returns:   Raw unfiltered distance in mm (single reading)
//
//  --- set_ultrasound_rgb() ---
//  Signature: void minihexa.sensor.set_ultrasound_rgb(uint8_t mode,
//                                                      uint8_t *rgb1,
//                                                      uint8_t *rgb2);
//  Parameters:
//    mode  — 0 = solid color, 1 = breathing/pulsing
//    rgb1  — 3-byte array {R, G, B} for right LED (0-255 each)
//    rgb2  — 3-byte array {R, G, B} for left LED (0-255 each)
//
//  Example — set both LEDs to solid green:
//    uint8_t rgb1[3] = {0, 255, 0};
//    uint8_t rgb2[3] = {0, 255, 0};
//    minihexa.sensor.set_ultrasound_rgb(0, rgb1, rgb2);
//
//  Example — breathing red:
//    uint8_t rgb[3] = {255, 0, 0};
//    minihexa.sensor.set_ultrasound_rgb(1, rgb, rgb);
//


// ============================================================================
//  10. IR SENSORS (cliff/edge detection)
// ============================================================================
//
//  Two downward-facing IR sensors for detecting floor/cliff edges.
//
//  --- get_ir1_state() ---
//  Signature: uint8_t minihexa.sensor.get_ir1_state();
//  Returns:   0 = floor detected (safe), 1 = no floor (cliff/edge!)
//  GPIO: 18
//
//  --- get_ir2_state() ---
//  Signature: uint8_t minihexa.sensor.get_ir2_state();
//  Returns:   0 = floor detected (safe), 1 = no floor (cliff/edge!)
//  GPIO: 32
//
//  Usage pattern (fall prevention):
//    uint8_t ir1 = minihexa.sensor.get_ir1_state();
//    uint8_t ir2 = minihexa.sensor.get_ir2_state();
//    if (ir1 == 1 || ir2 == 1) {
//        // CLIFF DETECTED — back up immediately!
//        vel = {0.0f, -3.0f, 0.0f};
//        minihexa.move(&vel, &pos, &att, 600, 3);
//        delay(2400);
//    }
//


// ============================================================================
//  11. TOUCH SENSOR
// ============================================================================
//
//  --- get_touch_state() ---
//  Signature: uint8_t minihexa.sensor.get_touch_state();
//  Returns:   0 = pressed/touched, 1 = released/not touched
//  GPIO: 18 (shared pin with IR1)
//
//  Example:
//    if (minihexa.sensor.get_touch_state() == 0) {
//        Serial.println("Touched!");
//    }
//


// ============================================================================
//  12. BOARD PERIPHERALS
// ============================================================================
//
//  All accessed through minihexa.board
//
//  --- IMU (QMI8658 6-axis) ---
//
//  get_imu_euler(float *val)
//    Writes Euler angles to val[0..2]: [roll, pitch, yaw] in degrees
//    Uses Madgwick AHRS filter internally
//    Example:
//      float euler[3];
//      minihexa.board.get_imu_euler(euler);
//      Serial.printf("Roll=%.1f Pitch=%.1f Yaw=%.1f\n",
//                    euler[0], euler[1], euler[2]);
//
//  get_imu_acc(float *val)
//    Writes accelerometer to val[0..2]: [x, y, z] in g
//
//  get_imu_gyro(float *val)
//    Writes gyroscope to val[0..2]: [x, y, z] in degrees/sec
//
//  imu_update(bool state)
//    true = start 10ms IMU update timer, false = stop
//    (Usually managed internally by balance())
//
//
//  --- Button (GPIO 0) ---
//
//  get_button_state()
//    Returns: true = NOT pressed, false = pressed
//    (Inverted logic — GPIO0 has a pull-up)
//    Example:
//      if (!minihexa.board.get_button_state()) {
//          Serial.println("Button pressed!");
//      }
//
//
//  --- Sound Sensor (GPIO 34) ---
//
//  get_sound_val()
//    Returns: raw ADC value (0-4095) from sound/microphone sensor
//    Example:
//      int sound = minihexa.board.get_sound_val();
//      if (sound > 2000) { /* loud sound detected */ }
//
//
//  --- Battery Voltage ---
//
//  minihexa.board.bat_voltage
//    Type: uint16_t, unit: millivolts
//    Updated automatically every 1 second by internal timer
//    Normal range: ~7000-8400 mV (2S LiPo)
//    Low battery buzzer activates below 7000 mV
//    Buzzer deactivates above 7400 mV
//    Example:
//      Serial.printf("Battery: %d mV\n", minihexa.board.bat_voltage);
//
//  Also available: minihexa.sensor.get_bat_voltage() (I2C path, v1.0 boards)
//


// ============================================================================
//  13. ACTION GROUPS (SPIFFS file playback)
// ============================================================================
//
//  Action groups are recorded servo position sequences stored as binary
//  files on SPIFFS flash (/ActionGroup{id}.rob). Each file contains
//  frame-by-frame servo positions that can be played back.
//
//  --- action_group_run(uint8_t id) ---
//  Plays stored action group. Blocking until complete.
//  Max ID: 49 (up to 50 action groups)
//
//  --- action_group_stop() ---
//  Interrupts a running action group.
//
//  --- action_group_download(uint8_t id, uint8_t *buf, size_t length) ---
//  Writes action group binary data to SPIFFS.
//  Frame index 1 = create/overwrite file
//  Frame index >1 = append to existing file
//
//  --- action_group_erase(uint8_t id) ---
//  Deletes action group file for given ID.
//
//  --- list_action_group_dir() ---
//  Prints all stored .rob files to Serial (via ESP_LOGI).
//
//  Built-in named actions (triggered by action group command with data[0]):
//    ID 1 = twist CCW:     minihexa.twist(15.0f, 3, 15, COUNTER_CLOCKWISE)
//    ID 2 = twist CW:      minihexa.twist(15.0f, 3, 15, CLOCKWISE)
//    ID 3 = short stretch:  minihexa._wake_up()
//    ID 4 = wake up+walk:   minihexa.wake_up()
//    ID 5 = acting cute:    minihexa.acting_cute()
//    ID 6+ = play SPIFFS file
//


// ============================================================================
//  14. SERVO DIRECT CONTROL
// ============================================================================
//
//  --- multi_servo_control() ---
//  Signature: void minihexa.multi_servo_control(ServoArg_t* arg,
//                                                uint16_t servo_num,
//                                                uint16_t time);
//  Sends raw PWM pulse-width values to servos, bypassing kinematics.
//
//  ServoArg_t struct:
//    uint8_t  id;    // servo ID (1-21)
//    uint16_t duty;  // pulse width in microseconds (500-2500)
//
//  Example — move servo 1 to center position:
//    ServoArg_t servo = {1, 1500};
//    minihexa.multi_servo_control(&servo, 1, 500);  // 1 servo, 500ms
//
//
//  SERVO ID MAP:
//
//  Leg 1 (front-right):  joint_a=1,  joint_b=2,  joint_c=3
//  Leg 2 (mid-right):    joint_a=4,  joint_b=5,  joint_c=6
//  Leg 3 (back-right):   joint_a=7,  joint_b=8,  joint_c=9
//  Leg 4 (back-left):    joint_a=10, joint_b=11, joint_c=12
//  Leg 5 (mid-left):     joint_a=13, joint_b=14, joint_c=15
//  Leg 6 (front-left):   joint_a=16, joint_b=17, joint_c=18
//  Arm:                  joint_a=19, joint_b=20, joint_c=21
//
//  Joint roles:
//    joint_a = coxa (hip rotation, rotates leg around Z axis)
//    joint_b = femur (upper leg, lifts leg)
//    joint_c = tibia (lower leg, extends/curls)
//
//  Servo types:
//    IDs 1-18:  280-degree servos (factor: 7.143 us/degree)
//    IDs 19-21: 180-degree servos (factor: 11.11 us/degree)
//
//  Duty range: 500-2500 microseconds
//  Midpoint: ~1500 us (SERVO_MIDPOINT = 140.0 degrees in library units)
//
//
//  SERVO CALIBRATION (trim offsets):
//
//  set_deviation(uint8_t id, int8_t val)
//    Set trim offset for servo id (range: -100 to +100)
//
//  read_deviation(int8_t *val)
//    Read all 18 offsets into val[0..17]
//
//  download_deviation()
//    Save all offsets to NVS flash (persists across reboots)
//


// ============================================================================
//  15. KEY STRUCTS
// ============================================================================
//
//  Velocity_t (defined in kinematics.h):
//    float vx;     // lateral speed (+ = right)
//    float vy;     // forward speed (+ = forward)
//    float omega;  // rotation speed (+ = CCW/left)
//
//  Vector_t (defined in kinematics.h):
//    volatile float x;  // lateral position offset (cm)
//    volatile float y;  // forward position offset (cm)
//    volatile float z;  // height offset (cm)
//
//  Euler_t (defined in kinematics.h):
//    float pitch;  // forward/back tilt (degrees)
//    float roll;   // side tilt (degrees)
//    float yaw;    // rotation (degrees, clamped -20 to +20)
//
//  ServoArg_t (defined in hiwonder_servo.h):
//    uint8_t  id;    // servo ID (1-21)
//    uint16_t duty;  // PWM pulse width (500-2500 us)
//
//  CircularPath (defined in hiwonder_robot.h):
//    float r;                      // circle radius (cm)
//    uint16_t steps_per_circle;    // points per revolution
//    uint16_t total_circles;       // number of circles
//    uint16_t current_step;        // current index
//    bool is_completed;            // done flag
//    RotationDirection direction;  // CLOCKWISE or COUNTER_CLOCKWISE
//
//  RotationDirection enum:
//    CLOCKWISE = 1
//    COUNTER_CLOCKWISE = -1
//
//  ModeState enum (from global.h — used by remote control protocol):
//    MINIHEXA_NULL = 0
//    MINIHEXA_RESET = 1
//    MINIHEXA_VOLTAGE_GET = 2
//    MINIHEXA_CRAWL_STATE = 3
//    MINIHEXA_MOVING_CONTROL = 4
//    MINIHEXA_POSE_CONTROL = 5
//    MINIHEXA_AVOID = 6
//    MINIHEXA_BALANCE = 7
//    MINIHEXA_RGB_ADJUST = 8
//    MINIHEXA_OFFSET_STATE = 9
//    MINIHEXA_OFFSET_READ = 10
//    MINIHEXA_OFFSET_SAVE = 11
//    MINIHEXA_OFFSET_VERIFY = 12
//    MINIHEXA_LEG_OFFSET_SET = 13
//    MINIHEXA_ACTION_GROUP_RUN = 14
//    MINIHEXA_ACTION_GROUP_STOP = 15
//    MINIHEXA_ACTION_GROUP_DOWNLOAD = 16
//    MINIHEXA_ACTION_GROUP_ERASE = 17
//    MINIHEXA_ACTION_GROUP_ALL_ERASE = 18
//    MINIHEXA_SERVO_CONTROL = 19
//    MINIHEXA_SERVO_DUTY_READ = 20
//    MINIHEXA_ARM_CONTROL = 21
//
//  Robot internal states:
//    Func_State:  CALIBRATE, CRAWL, ACTION_GROUP
//    Move_State:  MOVING, STOP, REST
//    Avoid_State: FORWARD, BACK, TURN, WAIT
//


// ============================================================================
//  16. GPIO PIN MAP
// ============================================================================
//
//  Pin  | Function              | Notes
//  -----|---------------------- |----------------------------
//  0    | Button                | Pull-up, LOW = pressed
//  2    | Onboard LED           | Built-in blue LED
//  16   | UART2 RX (RXD2)      | Servo bus receive
//  17   | UART2 TX (TXD2)      | Servo bus transmit
//  18   | IR sensor 1 / Touch   | INPUT, shared pin
//  19   | IR sensor 2           | INPUT
//  21   | Buzzer                | LEDC PWM output
//  22   | I2C SDA               | Shared I2C data
//  23   | I2C SCL               | Shared I2C clock
//  32   | IR sensor 2 (alt)     | INPUT (sensor.io3_pin)
//  33   | Battery ADC (v1.1)    | ADC1_CH5, voltage divider
//  34   | Sound sensor           | ADC input, analog mic
//


// ============================================================================
//  17. I2C ADDRESS MAP
// ============================================================================
//
//  Address | Device                | Used For
//  --------|-----------------------|--------------------------------
//  0x77    | Ultrasonic module     | Distance sensing + RGB LEDs
//  0x52    | ESP32-S3 Camera       | Color/face detection (AI projects)
//  0x34    | WonderEcho ASR        | Voice recognition (AI projects)
//  0x46    | Battery decoder (v1.0)| Battery voltage (v1.0 boards only)
//  0x6B    | QMI8658 IMU           | Accelerometer + Gyroscope
//


// ============================================================================
//  18. UTILITY FUNCTIONS & MACROS
// ============================================================================
//
//  From kinematics.h:
//    float fmap(float x, float in_min, float in_max, float out_min, float out_max)
//      Linear map — like Arduino map() but for floats
//
//  From global.h:
//    GET_LOW_BYTE(A)        — extract low byte: (uint8_t)(A)
//    GET_HIGH_BYTE(A)       — extract high byte: (uint8_t)((A) >> 8)
//    BYTE_TO_HW(A, B)       — combine bytes: ((uint16_t)(A) << 8) | (uint8_t)(B)
//    LIMIT(x, min, max)     — clamp value between min and max
//
//  From kinematics.h (advanced — for custom IK work):
//    Vector_t fkine(Theta_t *theta)
//      Forward kinematics: joint angles → foot position
//
//    Theta_t ikine(Vector_t *vector)
//      Inverse kinematics: foot position → joint angles
//
//    Vector_t rotation_trans(Vector_t *vec, float theta, Axis_state axis)
//      Rotate a 3D vector by theta degrees around AXIS_X/AXIS_Y/AXIS_Z
//
//    Vector_t vector_arg_ops(Vector_t *v1, Vector_t *v2, Ops_state ops)
//      Add (ADD) or subtract (SUB) two 3D vectors
//


// ============================================================================
//  19. LEG GEOMETRY & KINEMATICS CONSTANTS
// ============================================================================
//
//  Leg segment lengths (cm):
//    BODY_TOP_LEN    = 2.85   (coxa — hip to first joint)
//    TOP_MIDDLE_LEN  = 3.70   (femur — first to second joint)
//    MIDDLE_BASE_LEN = 7.20   (tibia — second joint to foot tip)
//
//  Body dimensions (cm):
//    body_width  = 7.6
//    body_length = 14.0
//
//  Default leg rest position: {6.5, 0.0, -4.0} cm
//    (6.5cm out from hip, 4cm below body)
//
//  Leg hip attachment offsets from body center (cm):
//    Leg 1 (front-right): x=+3.15, y=-5.65
//    Leg 2 (mid-right):   x=+4.34, y= 0.00
//    Leg 3 (back-right):  x=+3.15, y=+5.65
//    Leg 4 (back-left):   x=-3.15, y=+5.65
//    Leg 5 (mid-left):    x=-4.34, y= 0.00
//    Leg 6 (front-left):  x=-3.15, y=-5.65
//
//  Max half-step length: 3.0 cm (limits max stride per gait cycle)
//  Default leg lift height: 3.0 cm (dynamically adjusted by speed/height)
//


// ============================================================================
//  20. CAMERA MODULE (ESP32-S3) — for reference only
// ============================================================================
//
//  Not needed for basic competition code, but available if camera is mounted.
//  All accessed through minihexa.sensor.camera
//  I2C address: 0x52
//
//  Color detection (full frame):
//    camera.red_block_detection(buf, buf_len)
//    camera.green_block_detection(buf, buf_len)
//    camera.blue_block_detection(buf, buf_len)
//    camera.purple_block_detection(buf, buf_len)
//    camera.color_id_detection(buf, buf_len)
//
//  Face detection:
//    camera.face_data_receive(buf, buf_len)
//
//  Line following (region-based):
//    camera.region1_red_block_detection(buf, buf_len)
//    camera.region2_red_block_detection(buf, buf_len)
//    (same for green, blue, purple)
//


// ============================================================================
//  21. VOICE MODULE (WonderEcho ASR) — for reference only
// ============================================================================
//
//  Not needed for basic competition code.
//  Accessed through minihexa.sensor.asr
//  I2C address: 0x34
//
//  rec_recognition()    — returns voice recognition result byte
//  speak(cmd, id)       — trigger speech output
//    cmd: ASR_COMMAND (0x00) or ASR_ANNOUNCER (0xFF)
//    id:  phrase ID number
//


// ============================================================================
//  END OF REFERENCE
// ============================================================================

// This file compiles as-is (empty — all comments). No setup() or loop() needed
// since it is just a reference document. Arduino IDE requires these for .ino
// files to compile, so they are provided below as empty stubs:

void setup() {}
void loop() {}
