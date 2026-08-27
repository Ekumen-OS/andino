/// PID computation rate \[Hz\].
pub const PID_RATE: u32 = 30;
/// PID computation period \[ms\].
pub const PID_PERIOD_MS: u32 = 1000 / PID_RATE;
/// Time window to automatically stop the robot if no command has been received \[ms\].
pub const AUTO_STOP_WINDOW: u32 = 3000;

/// Serial port baud rate.
pub const BAUDRATE: u32 = 9600;

/// PID default tuning proportional gain.
pub const PID_KP: i32 = 30;
/// PID default tuning derivative gain.
pub const PID_KD: i32 = 10;
/// PID default tuning integral gain.
pub const PID_KI: i32 = 0;
/// PID default tuning output gain.
pub const PID_KO: i32 = 10;

/// Minimum PWM wave duty cycle (0%) (see
/// <https://www.arduino.cc/reference/en/language/functions/analog-io/analogwrite/>).
pub const PWM_MIN: i32 = 0;
/// Maximum PWM wave duty cycle (100%) (see
/// <https://www.arduino.cc/reference/en/language/functions/analog-io/analogwrite/>).
pub const PWM_MAX: i32 = 255;
