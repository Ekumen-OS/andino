//! PID controller module for closed-loop control.

/// PID controller.
///
/// The controller uses "derivative on measurement" to avoid derivative kick when the
/// setpoint changes abruptly, making it suitable for motor speed control applications.
///
/// # Example
///
/// ```rust
/// // Create a PID controller with output range [-255, 255]
/// let mut pid = Pid::new(-255, 255);
///
/// // Configure PID gains (Kp, Ki, Kd, Ko)
/// pid.set_tunings(10, 5, 2, 1);
///
/// // Set target value
/// pid.set_setpoint(100);
///
/// // In your control loop:
/// loop {
///     let sensor_value = read_encoder();
///
///     if let Some(output) = pid.compute(sensor_value) {
///         motor.set_speed(output as i16);
///     }
/// }
/// ```
pub struct Pid {
    /// Proportional gain.
    kp: i32,
    /// Integral gain.
    ki: i32,
    /// Derivative gain.
    kd: i32,
    /// Output scaling factor.
    ko: i32,

    /// Maximum allowed output value.
    output_max: i32,
    /// Minimum allowed output value.
    output_min: i32,

    /// Desired target value.
    setpoint: i32,
    /// Previous feedback/measurement value.
    last_feedback: i32,
    /// Previous input value.
    last_input: i32,
    /// Previous output value.
    last_output: i32,
    /// Accumulated integral term with anti-windup clamping.
    integral_term: i32,

    /// Enable pid
    enabled: bool,
}

impl Pid {
    /// Creates a new PID controller with specified output limits.
    ///
    /// The controller is initialized with all gains set to zero. Use `set_tunings()`
    /// to configure the PID gains before use.
    ///
    /// # Arguments
    /// * `output_min` - Minimum allowed output value.
    /// * `output_max` - Maximum allowed output value.
    /// * `kp` - Proportional gain.
    /// * `ki` - Integral gain.
    /// * `kd` - Derivative gain.
    /// * `ko` - Output scaling divisor (divides final output).
    ///
    /// # Returns
    /// A new Pid instance with gains set to zero.
    pub fn new(output_min: i32, output_max: i32, kp: i32, ki: i32, kd: i32, ko: i32) -> Self {
        Self {
            ko,
            kp,
            ki,
            kd,
            output_max,
            output_min,
            setpoint: 0,
            last_feedback: 0,
            last_input: 0,
            last_output: 0,
            integral_term: 0,
            enabled: false,
        }
    }

    /// Enable PID
    pub fn enable(&mut self) {
        self.enabled = true;
    }

    /// Disable PID
    pub fn disable(&mut self) {
        self.enabled = false;
    }

    /// Indicate if the PID is enabled
    pub fn is_enabled(&self) -> bool {
        self.enabled
    }

    /// Configures the PID controller gains.
    ///
    /// # Arguments
    /// * `kp` - Proportional gain.
    /// * `ki` - Integral gain.
    /// * `kd` - Derivative gain.
    /// * `ko` - Output scaling divisor (divides final output).
    pub fn set_tunings(&mut self, kp: i32, ki: i32, kd: i32, ko: i32) {
        self.ko = ko;
        self.kp = kp;
        self.ki = ki;
        self.kd = kd;
    }

    /// Sets the desired target value.
    ///
    /// # Arguments
    /// * `setpoint` - The target value the controller should reach.
    pub fn set_setpoint(&mut self, setpoint: i32) {
        self.setpoint = setpoint;
    }

    /// Resets the controller state to initial conditions.
    ///
    /// # Arguments
    /// * `feedback` - Current measured value from sensor.
    pub fn reset(&mut self, feedback: i32) {
        self.setpoint = 0;
        self.last_feedback = feedback;
        self.last_input = 0;
        self.last_output = 0;
        self.integral_term = 0;
    }

    /// Computes the PID output based on current feedback.
    ///
    /// # Arguments
    /// * `feedback` - Current measured value from sensor.
    ///
    /// # Returns
    /// * `Some(output)` - Control output clamped to [output_min, output_max].
    pub fn compute(&mut self, feedback: i32) -> Option<i32> {
        if !self.enabled {
            // Reset PID once to prevent startup spikes.
            if self.last_input != 0 {
                self.reset(feedback);
            }
            return None;
        }

        let input = feedback - self.last_feedback;
        let error = self.setpoint - input;

        let proportional_term = self.kp * error;
        let derivative_term = -self.kd * (input - self.last_input);

        // Calculate output value and apply scaling
        let mut output = (proportional_term + derivative_term + self.integral_term) / self.ko;
        output += self.last_output;
        output = output.clamp(self.output_min, self.output_max);

        if output.abs() != self.output_max {
            self.integral_term += self.ki * error;
        }

        // Update state variables
        self.last_output = output;
        self.last_feedback = feedback;
        self.last_input = input;

        Some(output)
    }
}
