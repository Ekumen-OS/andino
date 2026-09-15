//! Motor control module for the Andino robot.

use arduino_hal::{
    port::{
        mode::{Output, PwmOutput},
        Pin, PinOps,
    },
    simple_pwm::PwmPinOps,
};
use embedded_hal::pwm::SetDutyCycle;

/// Minimum allowed motor speed value (0-255 scale).            
const MIN_MOTOR_SPEED: u8 = 0;
/// Maximum allowed motor speed value (0-255 scale).            
///                                                             
/// This corresponds to the maximum 8-bit PWM value for Arduino.
const MAX_MOTOR_SPEED: u8 = 255;

/// DC motor controller for the Andino robot.
///
/// It handles motor direction, speed control via PWM,
/// and motor enabling/disabling.
///
/// # Example
///
/// ```rust
/// use arduino_hal::simple_pwm::IntoPwmPin;
///
/// // Initialize pins and timers
/// let left_enable_motor_pin = pins.d13;
/// let left_forward_pwm = pins.d10.into_output().into_pwm(&timer1);
/// let left_backward_pwm = pins.d6.into_output().into_pwm(&timer0);
///
/// // Create a new motor instance
/// let mut left_motor = Motor::new(
///     left_enable_motor_pin,
///     left_forward_pwm,
///     left_backward_pwm,
/// );
///
/// // Enable the motor
/// left_motor.enable();
///
/// // Set motor speed (positive for forward, negative for backward)
/// left_motor.set_speed(150);  // Forward at ~59% speed
/// // left_motor.set_speed(-200);  // Backward at ~78% speed
/// // left_motor.set_speed(0);     // Stop the motor
/// ```
pub struct Motor<EPIN, FPIN, BPIN, FT, BT> {
    /// Minimum allowed speed value (typically 0).
    min_speed: u8,
    /// Maximum allowed speed value (typically 255 for 8-bit PWM).
    max_speed: u8,
    /// Digital output connected to motor enable pin.
    motor_enable_pin: Pin<Output, EPIN>,
    /// PWM output connected to motor forward pin.
    forward_pwm: Pin<PwmOutput<FT>, FPIN>,
    /// PWM output connected to motor backward pin.
    backward_pwm: Pin<PwmOutput<BT>, BPIN>,
}

impl<EPIN, FPIN, BPIN, FT, BT> Motor<EPIN, FPIN, BPIN, FT, BT>
where
    EPIN: PinOps,
    FPIN: PwmPinOps<FT>,
    BPIN: PwmPinOps<BT>,
    Pin<PwmOutput<FT>, FPIN>: SetDutyCycle,
    Pin<PwmOutput<BT>, BPIN>: SetDutyCycle,
{
    /// Creates a new Motor instance with default speed limits.
    ///
    /// This constructor initializes a motor with the default speed range of 0-255,
    /// which corresponds to the standard 8-bit PWM range on Arduino.
    ///
    /// # Arguments
    /// * `motor_enable_pin` - Digital output pin that enables/disables the motor driver.
    /// * `forward_pwm` - PWM pin that controls forward motion.
    /// * `backward_pwm` - PWM pin that controls backward motion.
    ///
    /// # Returns
    /// A new Motor instance.
    pub fn new(
        motor_enable_pin: Pin<Output, EPIN>,
        forward_pwm: Pin<PwmOutput<FT>, FPIN>,
        backward_pwm: Pin<PwmOutput<BT>, BPIN>,
    ) -> Self {
        Self {
            min_speed: MIN_MOTOR_SPEED,
            max_speed: MAX_MOTOR_SPEED,
            motor_enable_pin,
            forward_pwm,
            backward_pwm,
        }
    }

    /// Enables the motor driver.
    pub fn enable(&mut self) {
        self.motor_enable_pin.set_high();
        self.forward_pwm.enable();
        self.backward_pwm.enable();
    }

    /// Disables the motor driver.
    pub fn disable(&mut self) {
        self.motor_enable_pin.set_low();
        self.forward_pwm.disable();
        self.backward_pwm.disable();
    }

    /// Sets the speed and direction of the motor.
    ///
    /// This method controls both the speed and direction of the motor:
    /// - Positive values make the motor rotate forward.
    /// - Negative values make the motor rotate backward.
    /// - Zero stops the motor.
    ///
    /// # Arguments
    /// * `speed` - The desired speed value, typically between -255 and 255.
    pub fn set_speed(&mut self, speed: i32) {
        let abs_speed = speed.abs().min(self.max_speed as i32);

        let pct = ((abs_speed * 100) / self.max_speed as i32) as u8;

        let (fwd_val, bwd_val) = if speed >= 0 {
            (pct, self.min_speed)
        } else {
            (self.min_speed, pct)
        };

        self.forward_pwm.set_duty_cycle_percent(fwd_val).unwrap();
        self.backward_pwm.set_duty_cycle_percent(bwd_val).unwrap();
    }
}
