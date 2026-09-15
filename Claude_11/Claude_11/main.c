/*
  Self-Driving RC Car - bare-metal AVR C (ATmega328P), no Arduino core.
  Target: Arduino Nano board / ATmega328P @ 16 MHz
  Toolchain: avr-gcc (Microchip Studio "GCC C Executable Project")

  Arduino-pin -> AVR port/bit mapping (from schematic "Bil_1_kretsschema_Nano"):
    Sensor 1 - Left   : TRIG D2  = PD2   / ECHO D10 = PB2   (HY-SRF05)
    Sensor 2 - Middle : TRIG D4  = PD4   / ECHO D7  = PD7   (HY-SRF05)
    Sensor 3 - Right  : TRIG D8  = PB0   / ECHO D5  = PD5   (HY-SRF05)
    Steering servo    : D9  = PB1 (OC1A, Timer1, 16-bit)
    Drive motor DRV8871: IN1 = D3 = PD3 (OC2B), IN2 = D11 = PB3 (OC2A) (Timer2, 8-bit)
    Start trigger     : A0  = PC0 (remote/start module drives this HIGH to start)

  NOTE: the rear (4th) HY-SRF05 sensor (was TRIG D12=PB4 / ECHO D13=PB5) has
  been physically removed. There is now no way to check whether the rear is
  clear before reversing - all recovery reversing below is done "blind",
  i.e. it just assumes the rear is clear. If your track/pen has obstacles
  close behind the car this is riskier than the old rear-checked version.

  Behavior: waits for the start signal, then drives in one of two states:

    DRIVE - the normal state. Centers itself between the two side boards
    with a PID on the left/right sensor difference, and slows down as the
    front sensor sees a wall/corner getting close.

    TURN - triggered the instant the front sensor sees a corner wall inside
    CORNER_TRIGGER_DISTANCE_CM. A gentle PID nudge can't turn 90 degrees in
    a 1m-wide corridor in time, so instead the car commits: full steering
    lock, reduced fixed speed, held for TURN_DURATION_MS (extended a bit if
    the front is still blocked once that expires), then it hands back
    control to the DRIVE state/PID.

  Recovery (reverse + wiggle): without a rear sensor, the car can get stuck
  in two ways that previously relied on the rear sensor's "back off" step:

    1. Front emergency stop - the front sensor sees something inside
       STOP_DISTANCE_CM. Same trigger as before.
    2. Stuck detection (new) - all three front sensors report essentially
       unchanged distances for STUCK_CHECK_WINDOW_MS while the car is in
       DRIVE and commanding forward motion. This catches cases like wheels
       spinning against a wall/edge at an angle where the front sensor
       never actually reads "close" (e.g. wedged sideways), so the normal
       STOP_DISTANCE_CM check never fires.

  Either trigger runs the same recovery: stop, then reverse for
  REVERSE_TIME_MS while continuously wiggling the steering hard left/right
  (instead of holding one lock), which gives the car a much better chance
  of un-wedging itself than reversing in a straight line. Control then
  returns to DRIVE.

  Physical setup this was tuned for: 3 front HY-SRF05 sensors on a 45-degree
  bracket (left/right angled outward, middle straight ahead), mounted
  ~6-7cm off the ground. Track is a 1m-wide, 4m-long rectangular loop with
  90-degree corners, bounded by ~15cm boards. The board height is well
  above the sensor height, so a level beam hits them fine all the way
  round. The 45-degree mount means left/right read a diagonal distance to
  the side wall rather than a perpendicular one - since both sensors are
  angled the same way, this is just a constant scale factor on the PID's
  error signal, which STEER_KP already absorbs.
*/

#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdint.h>
#include <stdlib.h> // labs()

// ---------- Tuning constants ----------
// MAX_DISTANCE_CM/ECHO_TIMEOUT_US are shortened from the open-ground version
// since a 1m x 4m track never needs 5m of range - this also shortens the
// worst-case wait when a ping doesn't come back, keeping the loop faster.
#define MAX_DISTANCE_CM        150u    // ignore/clamp anything farther than this
#define ECHO_TIMEOUT_US        9000UL  // ~150cm round-trip timeout

#define STOP_DISTANCE_CM       10u     // genuine imminent collision - stop/steer/reverse
#define CORNER_SLOW_DISTANCE_CM 80u    // front wall closer than this -> start slowing, still in DRIVE state
#define CORNER_TRIGGER_DISTANCE_CM 60u // front wall closer than this -> commit to a hard-lock TURN

// Set to match which way your track loops - a fixed rectangular track
// always turns the same way, so hardcoding this is far more reliable than
// trying to infer it from noisy sensor readings right at the corner.
// 1 = always turn right (clockwise), 0 = always turn left (counter-clockwise).
#define TURN_DIRECTION_RIGHT    0u

#define TURN_SPEED              150u    // fixed, slow speed while executing a hard-lock turn
#define TURN_DURATION_MS        600u   // how long to hold full lock through a corner - THE main knob to tune
#define TURN_MAX_EXTRA_MS       800u   // if still blocked after TURN_DURATION_MS, keep turning up to this much longer

#define SERVO_MIN_US      1000u  // full-left pulse width  - calibrate to your linkage
#define SERVO_MAX_US      2000u  // full-right pulse width - calibrate to your linkage
#define SERVO_CENTER_DEG  90u
#define SERVO_MIN_DEG     45u    // maps to SERVO_MIN_US
#define SERVO_MAX_DEG     135u   // maps to SERVO_MAX_US

#define DRIVE_SPEED       200u   // 0-255 forward PWM speed on a clear straight
#define MIN_SPEED         150u    // speed floor so the car doesn't stall approaching a corner
#define TURN_SPEED_REDUCTION 50u // max PWM cut for hard PID steering corrections on a straight
#define REVERSE_SPEED     255u   // 0-255 reverse PWM speed used during recovery

// ---------- Recovery (reverse + steering wiggle) tuning ----------
// No rear sensor anymore, so recovery just reverses blind for a fixed time
// while sweeping the steering lock-to-lock, rather than backing straight up.
#define REVERSE_TIME_MS        600u    // total time spent reversing during a recovery
#define WIGGLE_HALF_PERIOD_MS  150u    // how often the steering flips side while reversing

// ---------- Stuck detection tuning ----------
// If, while in STATE_DRIVE and commanding forward motion, all three front
// sensors report essentially the same distance for this long, the car
// isn't actually going anywhere (wheels spinning / wedged) even though
// nothing tripped STOP_DISTANCE_CM. Only checked in STATE_DRIVE - STATE_TURN
// legitimately holds a lock for a while and has its own timeout escape, so
// checking there too would risk a false trigger mid-corner.
#define STUCK_CHECK_WINDOW_MS   1000u   // how long with no real change before calling it "stuck"
#define STUCK_DISTANCE_DELTA_CM 5u      // combined left+mid+right movement below this = not moving

// ---------- micros() via Timer0 (normal mode, prescaler 64 -> 4us/tick) ----------
static volatile uint32_t timer0_overflow_count = 0;

ISR(TIMER0_OVF_vect) {
    timer0_overflow_count++;
}

static void timer0_micros_init(void) {
    TCCR0A = 0;                          // normal mode
    TCCR0B = (1 << CS01) | (1 << CS00);  // prescaler 64
    TIMSK0 = (1 << TOIE0);
}

static uint32_t micros(void) {
    uint8_t oldSREG = SREG;
    cli();
    uint32_t m = timer0_overflow_count;
    uint8_t t = TCNT0;
    if ((TIFR0 & (1 << TOV0)) && (t < 255)) {
        m++;
    }
    SREG = oldSREG;
    return ((m << 8) + t) * 4UL; // 4us per timer tick at 16MHz/64
}

// ---------- Timer1: servo PWM on D9 / PB1 (OC1A) ----------
static void servo_timer_init(void) {
    TCCR1A = (1 << COM1A1) | (1 << WGM11);              // Fast PWM, non-inverting OC1A
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11);  // TOP=ICR1, prescaler 8
    ICR1   = 39999; // 20ms period @ 2MHz timer clock (16MHz/8)
    OCR1A  = 3000;  // 1500us = center, until first set_servo_angle() call
}

static void set_servo_angle(uint8_t angle_deg) {
    if (angle_deg < SERVO_MIN_DEG) angle_deg = SERVO_MIN_DEG;
    if (angle_deg > SERVO_MAX_DEG) angle_deg = SERVO_MAX_DEG;

    uint32_t span_deg = SERVO_MAX_DEG - SERVO_MIN_DEG;
    uint32_t span_us  = SERVO_MAX_US - SERVO_MIN_US;
    uint16_t pulse_us = SERVO_MIN_US + (uint16_t)((uint32_t)(angle_deg - SERVO_MIN_DEG) * span_us / span_deg);

    OCR1A = pulse_us * 2; // 2 timer ticks per microsecond at 2MHz timer clock
}

// ---------- Timer2: motor PWM, IN1=D3/PD3 (OC2B), IN2=D11/PB3 (OC2A) ----------
static void motor_timer_init(void) {
    TCCR2A = (1 << COM2A1) | (1 << COM2B1) | (1 << WGM21) | (1 << WGM20); // Fast PWM, non-inverting
    TCCR2B = (1 << CS22); // prescaler 64 -> ~976 Hz PWM
    OCR2A = 0;
    OCR2B = 0;
}

static void drive_forward(uint8_t speed) {
    OCR2A = 0;      // IN2 = 0
    OCR2B = speed;  // IN1 = PWM
}

static void drive_reverse(uint8_t speed) {
    OCR2B = 0;      // IN1 = 0
    OCR2A = speed;  // IN2 = PWM
}

static void motor_stop(void) {
    OCR2A = 0;
    OCR2B = 0;
}

// ---------- GPIO setup ----------
static void gpio_init(void) {
    // Port D: PD2 trig_left, PD3 motor_in1(OC2B), PD4 trig_mid -> outputs
    //         PD5 echo_right, PD7 echo_mid -> inputs
    DDRD |= (1 << PD2) | (1 << PD3) | (1 << PD4);
    DDRD &= ~((1 << PD5) | (1 << PD7));

    // Port B: PB0 trig_right, PB1 servo(OC1A), PB3 motor_in2(OC2A) -> outputs
    //         PB2 echo_left -> input
    // (rear sensor's PB4/PB5 removed along with the hardware)
    DDRB |= (1 << PB0) | (1 << PB1) | (1 << PB3);
    DDRB &= ~(1 << PB2);

    // Port C: PC0 (A0) start-trigger input
    DDRC &= ~(1 << PC0);
}

// ---------- Ultrasonic sensor: trigger + echo timing ----------
static void trigger_pulse(volatile uint8_t *port_reg, uint8_t bit) {
    *port_reg &= ~(1 << bit);
    _delay_us(2);
    *port_reg |= (1 << bit);
    _delay_us(10);
    *port_reg &= ~(1 << bit);
}

// Waits for echo to go HIGH then LOW, returns pulse width in microseconds
// (0 if it times out, meaning "no echo" / out of range).
static uint32_t measure_echo_pulse(volatile uint8_t *pin_reg, uint8_t bit) {
    uint32_t t0 = micros();
    while (!(*pin_reg & (1 << bit))) {
        if (micros() - t0 > ECHO_TIMEOUT_US) return 0;
    }
    uint32_t start = micros();
    while (*pin_reg & (1 << bit)) {
        if (micros() - start > ECHO_TIMEOUT_US) return 0;
    }
    return micros() - start;
}

static long read_distance_cm(volatile uint8_t *trig_port, uint8_t trig_bit,
                              volatile uint8_t *echo_pin, uint8_t echo_bit) {
    trigger_pulse(trig_port, trig_bit);
    uint32_t duration = measure_echo_pulse(echo_pin, echo_bit);

    if (duration == 0) {
        return MAX_DISTANCE_CM; // no echo = treat as clear / out of range
    }

    long distance = (long)(duration / 58);
    if (distance > MAX_DISTANCE_CM) distance = MAX_DISTANCE_CM;
    return distance;
}

// ---------- small helpers (map / constrain, since avr-libc has no Arduino equivalents) ----------
static long constrain_long(long x, long lo, long hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static long map_long(long x, long in_min, long in_max, long out_min, long out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

static float constrain_float(float x, float lo, float hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

// ---------- Steering PID ----------
// Error = (right sensor distance) - (left sensor distance), in cm.
// Positive error => more room on the right => steer right.
// Output is added directly to SERVO_CENTER_DEG as a degree offset.
#define STEER_KP  0.3f   // degrees of steering per cm of left/right imbalance
#define STEER_KI  0.02f   // corrects any steady drift/bias - keep small
#define STEER_KD  0.1f   // damps oscillation from sudden sensor jumps
#define STEER_INTEGRAL_LIMIT 150.0f // anti-windup clamp on the integral term (cm*s)

typedef struct {
    float kp, ki, kd;
    float integral;
    float prev_error;
} SteeringPID;

static SteeringPID steer_pid = {
    .kp = STEER_KP,
    .ki = STEER_KI,
    .kd = STEER_KD,
    .integral = 0.0f,
    .prev_error = 0.0f
};

static void pid_reset(SteeringPID *pid) {
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
}

// dt is in seconds. Returns a steering angle offset in degrees.
static float pid_update(SteeringPID *pid, float error, float dt) {
    if (dt <= 0.0f) dt = 0.001f; // guard against a zero/negative dt on the first call

    pid->integral += error * dt;
    pid->integral = constrain_float(pid->integral, -STEER_INTEGRAL_LIMIT, STEER_INTEGRAL_LIMIT);

    float derivative = (error - pid->prev_error) / dt;
    pid->prev_error = error;

    return (pid->kp * error) + (pid->ki * pid->integral) + (pid->kd * derivative);
}

// ---------- Sensor smoothing ----------
// Ultrasonic readings jitter by a few cm between pings just from measurement
// noise. Left undamped, that jitter looks like a huge "rate of change" to
// the D term (a 3cm jump over one ~30ms loop is a ~100cm/s "velocity"),
// which the derivative gain then amplifies straight into servo twitch. This
// exponential filter smooths the raw left/right difference before it ever
// reaches the PID, so STEER_KD can damp real oscillation instead of noise.
// Lower alpha = smoother but slower to respond; 1.0 = no filtering at all.
#define SENSOR_FILTER_ALPHA 0.3f

static float filtered_diff = 0.0f;
static uint8_t filtered_diff_valid = 0;

static float filter_update(float raw_value) {
    if (!filtered_diff_valid) {
        filtered_diff = raw_value;
        filtered_diff_valid = 1;
    } else {
        filtered_diff = (SENSOR_FILTER_ALPHA * raw_value) + ((1.0f - SENSOR_FILTER_ALPHA) * filtered_diff);
    }
    return filtered_diff;
}

// ---------- Turn PID (experimental) ----------
// A separate, much more aggressive PID used only during STATE_TURN. Same
// error signal (right - left, in cm) as the drive PID, but tuned to snap
// toward full steering lock quickly instead of gently nudging - a 90-degree
// corner in a 1m corridor needs decisive steering, not a small correction.
// It reads the RAW (unfiltered) diff, since the smoothing that helps
// straight-line wobble would only blunt the fast reaction needed here.
#define TURN_KP  0.5f    // aggressive: diff of ~90cm should already be close to full lock
#define TURN_KI  0.0f    // no integral - this is a short, discrete maneuver, not sustained tracking
#define TURN_KD  0.0f    // start with none; a sudden "lost the wall" jump would otherwise cause a huge derivative spike

// Safety net: if 1, ignore the turn PID's sign and always steer toward
// TURN_DIRECTION_RIGHT, using only the PID's magnitude for how hard to turn.
// Flip this to 1 if testing shows it occasionally turns the wrong way -
// sensor readings right at a sharp corner are the least reliable moment to
// trust for direction, even though the magnitude they give is still useful.
#define TURN_FORCE_DIRECTION 0

static SteeringPID turn_pid = {
    .kp = TURN_KP,
    .ki = TURN_KI,
    .kd = TURN_KD,
    .integral = 0.0f,
    .prev_error = 0.0f
};

// Resets the drive PID, the turn PID, and the sensor filter together, so
// state carried over from before a corner/emergency stop never leaks into
// whatever comes next.
static void reset_steering(void) {
    pid_reset(&steer_pid);
    pid_reset(&turn_pid);
    filtered_diff_valid = 0;
}

// ---------- Stuck detector ----------
// Compares current left/mid/right readings against a snapshot taken
// STUCK_CHECK_WINDOW_MS ago. If they've barely moved, the car is commanding
// forward motion but isn't actually progressing.
static uint32_t stuck_window_start_us = 0;
static long stuck_ref_left = 0, stuck_ref_mid = 0, stuck_ref_right = 0;
static uint8_t stuck_window_valid = 0;

static void reset_stuck_detector(void) {
    stuck_window_valid = 0;
}

// Call once per loop iteration with the freshest readings while in
// STATE_DRIVE. Returns 1 the moment a full window has elapsed with
// essentially no combined change across all three sensors, 0 otherwise.
// Also transparently rolls the window forward, so it doesn't need to be
// called at any particular rate.
static uint8_t stuck_check_and_update(uint32_t now_us, long dist_left, long dist_mid, long dist_right) {
    if (!stuck_window_valid) {
        stuck_window_start_us = now_us;
        stuck_ref_left = dist_left;
        stuck_ref_mid = dist_mid;
        stuck_ref_right = dist_right;
        stuck_window_valid = 1;
        return 0;
    }

    if ((now_us - stuck_window_start_us) < ((uint32_t)STUCK_CHECK_WINDOW_MS * 1000UL)) {
        return 0; // window not up yet
    }

    long total_change = labs(dist_left - stuck_ref_left)
                       + labs(dist_mid  - stuck_ref_mid)
                       + labs(dist_right - stuck_ref_right);

    // Roll the window forward regardless of the outcome, using the current
    // readings as the new baseline for the next window.
    stuck_window_start_us = now_us;
    stuck_ref_left = dist_left;
    stuck_ref_mid = dist_mid;
    stuck_ref_right = dist_right;

    return (total_change < (long)STUCK_DISTANCE_DELTA_CM) ? 1 : 0;
}

// ---------- Recovery: reverse while wiggling the steering ----------
// Used both for the front-emergency stop and for stuck detection. With no
// rear sensor there's no way to confirm the rear is clear, so this reverses
// blind. Wiggling the steering lock-to-lock while backing up (rather than
// holding one lock, or going straight) gives a wedged/spinning car a much
// better chance of freeing itself.
static void recover_reverse_and_wiggle(void) {
    motor_stop();
    drive_reverse(REVERSE_SPEED);

    uint32_t recover_start_us = micros();
    uint8_t steer_right = (uint8_t)TURN_DIRECTION_RIGHT; // arbitrary starting side

    while ((micros() - recover_start_us) < ((uint32_t)REVERSE_TIME_MS * 1000UL)) {
        set_servo_angle(steer_right ? SERVO_MAX_DEG : SERVO_MIN_DEG);
        _delay_ms(WIGGLE_HALF_PERIOD_MS);
        steer_right = !steer_right;
    }

    motor_stop();
    set_servo_angle(SERVO_CENTER_DEG);

    reset_steering();
    reset_stuck_detector();
}

// ---------- Drive state machine ----------
typedef enum {
    STATE_DRIVE, // PID wall-centering on a straight
    STATE_TURN   // PID-driven, aggressively-tuned turn through a corner
} DriveState;

int main(void) {
    gpio_init();
    timer0_micros_init();
    servo_timer_init();
    motor_timer_init();
    motor_stop();
    set_servo_angle(SERVO_CENTER_DEG);

    sei(); // enable interrupts (needed for micros())

    // Wait for the remote/start module to drive A0 (PC0) HIGH
    while (!(PINC & (1 << PC0))) {
        _delay_ms(20);
    }

    uint32_t last_pid_us = micros();
    DriveState state = STATE_DRIVE;
    uint32_t turn_start_us = 0;
    reset_stuck_detector();

    while (1) {
        // ---- Stop/resume: A0 going LOW pauses the car; it resumes once A0
        // goes HIGH again, starting fresh (motor stopped, wheels centered,
        // steering state reset while paused so nothing stale carries over).
        if (!(PINC & (1 << PC0))) {
            motor_stop();
            set_servo_angle(SERVO_CENTER_DEG);
            reset_steering();
            reset_stuck_detector();
            state = STATE_DRIVE;

            while (!(PINC & (1 << PC0))) {
                _delay_ms(20);
            }

            last_pid_us = micros(); // fresh timing reference for dt after resuming
            continue;
        }

        long dist_left  = read_distance_cm(&PORTD, PD2, &PINB, PB2);
        _delay_ms(10); // let the previous ping's echoes die down (avoid cross-talk)

        long dist_mid   = read_distance_cm(&PORTD, PD4, &PIND, PD7);
        _delay_ms(10);

        long dist_right = read_distance_cm(&PORTB, PB0, &PIND, PD5);
        _delay_ms(10);

        // ---- Safety net: applies in either state ----
        if (dist_mid < STOP_DISTANCE_CM) {
            // Front wall came up faster than steering could react. No rear
            // sensor to check anymore, so just reverse blind while wiggling
            // the steering to help clear whatever's ahead.
            recover_reverse_and_wiggle();
            last_pid_us = micros();
            state = STATE_DRIVE;
            continue;
        }

        if (state == STATE_DRIVE) {
            // Stuck check: only meaningful here, since STATE_TURN
            // legitimately holds readings still for a while on purpose.
            if (stuck_check_and_update(micros(), dist_left, dist_mid, dist_right)) {
                recover_reverse_and_wiggle();
                last_pid_us = micros();
                continue;
            }

            if (dist_mid < CORNER_TRIGGER_DISTANCE_CM) {
                // Committing to a hard-lock turn - a gradual PID correction
                // can't make a 90-degree turn in a 1m-wide corridor in time.
                state = STATE_TURN;
                turn_start_us = micros();
                reset_steering();
                reset_stuck_detector();
                continue;
            }

            // Continuous wall-centering PID: keeps the car centered between
            // the two side boards on the straights. error > 0 = more room
            // on the right = steer right.
            uint32_t now = micros();
            float dt = (float)(now - last_pid_us) / 1000000.0f;
            last_pid_us = now;

            long diff = constrain_long(dist_right - dist_left, -(long)MAX_DISTANCE_CM, (long)MAX_DISTANCE_CM);
            float smoothed_diff = filter_update((float)diff);
            float offset = pid_update(&steer_pid, smoothed_diff, dt);

            int angle = (int)SERVO_CENTER_DEG + (int)offset;
            if (angle < (int)SERVO_MIN_DEG) angle = (int)SERVO_MIN_DEG;
            if (angle > (int)SERVO_MAX_DEG) angle = (int)SERVO_MAX_DEG;
            set_servo_angle((uint8_t)angle);

            // Slow down for two independent reasons, and use whichever wants
            // the lower speed: (1) how hard we're currently steering, and
            // (2) how close the front wall/corner is getting.
            long steer_amount = (offset < 0.0f) ? (long)(-offset) : (long)offset;
            long turn_speed = (long)DRIVE_SPEED - map_long(
                constrain_long(steer_amount, 0, (long)(SERVO_MAX_DEG - SERVO_CENTER_DEG)),
                0, (long)(SERVO_MAX_DEG - SERVO_CENTER_DEG),
                0, (long)TURN_SPEED_REDUCTION);

            long front_speed = (dist_mid < CORNER_SLOW_DISTANCE_CM)
                ? map_long(constrain_long(dist_mid, STOP_DISTANCE_CM, CORNER_SLOW_DISTANCE_CM),
                           STOP_DISTANCE_CM, CORNER_SLOW_DISTANCE_CM,
                           MIN_SPEED, DRIVE_SPEED)
                : (long)DRIVE_SPEED;

            long speed = (turn_speed < front_speed) ? turn_speed : front_speed;
            if (speed < MIN_SPEED) speed = MIN_SPEED;

            drive_forward((uint8_t)speed);

        } else { // STATE_TURN
            uint32_t now = micros();
            float dt = (float)(now - last_pid_us) / 1000000.0f;
            last_pid_us = now;

            // Raw, unfiltered diff - we want a fast reaction here, not the
            // noise smoothing used on the straights.
            long diff = constrain_long(dist_right - dist_left, -(long)MAX_DISTANCE_CM, (long)MAX_DISTANCE_CM);
            float offset = pid_update(&turn_pid, (float)diff, dt);

#if TURN_FORCE_DIRECTION
            float magnitude = (offset < 0.0f) ? -offset : offset;
            offset = TURN_DIRECTION_RIGHT ? magnitude : -magnitude;
#endif

            int angle = (int)SERVO_CENTER_DEG + (int)offset;
            if (angle < (int)SERVO_MIN_DEG) angle = (int)SERVO_MIN_DEG;
            if (angle > (int)SERVO_MAX_DEG) angle = (int)SERVO_MAX_DEG;
            set_servo_angle((uint8_t)angle);

            drive_forward(TURN_SPEED);

            uint32_t elapsed_ms = (micros() - turn_start_us) / 1000UL;

            if (elapsed_ms >= TURN_DURATION_MS) {
                // Nominal turn time is up. If the front is clear again, the
                // corner's behind us - hand back to the drive PID. If it's
                // still blocked, keep the turn PID running a bit longer
                // rather than snapping back to straight-line driving into a wall.
                if (dist_mid >= CORNER_TRIGGER_DISTANCE_CM ||
                    elapsed_ms >= (TURN_DURATION_MS + TURN_MAX_EXTRA_MS)) {
                    state = STATE_DRIVE;
                    reset_steering();
                    reset_stuck_detector();
                    last_pid_us = micros();
                }
            }
        }
    }
}
