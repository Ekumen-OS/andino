//! The main application controller for the Andino robot.

use core::cell::RefCell;

use arduino_hal::{
    simple_pwm::{IntoPwmPin, Prescaler, Timer0Pwm, Timer1Pwm},
    Peripherals,
};
use avr_device::interrupt::{self, Mutex};

use crate::{
    components::{
        encoder::Encoder, motor::Motor, shell::Shell, Command, CommandArgs, CommandError,
        CommandErrorResult,
    },
    util::{
        interrupt::IntoInterrupt,
        millis::{millis, millis_init},
        serial::SerialStream,
    },
};

use super::{
    constants::{
        AUTO_STOP_WINDOW, BAUDRATE, PID_KD, PID_KI, PID_KO, PID_KP, PID_PERIOD_MS, PID_RATE,
        PWM_MAX,
    },
    Commands, EncoderLeftMotorType, EncoderRightMotorType, LeftMotorType, RightMotorType,
};
use crate::components::pid::Pid;

/// Static storage for the LED pin so it can be accessed from ISR
static LEFT_ENCODER: Mutex<RefCell<Option<EncoderLeftMotorType>>> = Mutex::new(RefCell::new(None));
static RIGHT_ENCODER: Mutex<RefCell<Option<EncoderRightMotorType>>> =
    Mutex::new(RefCell::new(None));

/// Attach ISR callback to encoder
fn left_encoder_isr(pci_group: u8) {
    interrupt::free(|cs| {
        if let Some(ref mut encoder) = LEFT_ENCODER.borrow(cs).borrow_mut().as_mut() {
            encoder.handle_interrupt(pci_group);
        }
    });
}

fn right_encoder_isr(pci_group: u8) {
    interrupt::free(|cs| {
        if let Some(ref mut encoder) = RIGHT_ENCODER.borrow(cs).borrow_mut().as_mut() {
            encoder.handle_interrupt(pci_group);
        }
    });
}

/// Reset the encoders count
fn reset_encoders_count() {
    interrupt::free(|cs| {
        if let Some(ref mut encoder) = LEFT_ENCODER.borrow(cs).borrow_mut().as_mut() {
            encoder.reset();
        }
        if let Some(ref mut encoder) = RIGHT_ENCODER.borrow(cs).borrow_mut().as_mut() {
            encoder.reset();
        }
    });
}

/// Read encoder ticks count
fn read_encoders_count() -> (i32, i32) {
    let mut left_count = 0;
    let mut right_count = 0;
    interrupt::free(|cs| {
        if let Some(ref mut encoder) = LEFT_ENCODER.borrow(cs).borrow_mut().as_mut() {
            left_count = encoder.read();
        }
        if let Some(ref mut encoder) = RIGHT_ENCODER.borrow(cs).borrow_mut().as_mut() {
            right_count = encoder.read();
        }
    });
    (left_count, right_count)
}

/// This struct serves as the entry point and central controller for all robot functionality.
/// It orchestrates the various components and subsystems of the robot.
pub struct App {
    // Components
    left_motor: LeftMotorType,
    right_motor: RightMotorType,
    left_pid_controller: Pid,
    right_pid_controller: Pid,
    serial_stream: SerialStream,
    shell: Shell,

    last_pid_computation: u32,
    last_set_motors_speed_cmd: u32,
}

impl App {
    /// Creates a new App instance with default configuration.
    ///
    /// # Arguments
    /// * `dp` - mutable ATmega328p Peripherals singleton
    ///
    /// # Returns
    /// A new App instance ready to be initialized.
    pub fn new(mut dp: Peripherals) -> Self {
        let pins = arduino_hal::pins!(dp);

        millis_init(&mut dp.TC2);

        // Initialize PWM Timers
        let timer0 = Timer0Pwm::new(dp.TC0, Prescaler::Prescale64);
        let timer1 = Timer1Pwm::new(dp.TC1, Prescaler::Prescale64);

        // Configure MOTORS
        let left_enable_motor = pins.d13.into_output();
        let left_forward_pwm = pins.d10.into_output().into_pwm(&timer1);
        let left_backward_pwm = pins.d6.into_output().into_pwm(&timer0);
        let left_motor = Motor::new(left_enable_motor, left_forward_pwm, left_backward_pwm);

        let right_enable_motor = pins.d12.into_output();
        let right_forward_pwm = pins.d9.into_output().into_pwm(&timer1);
        let right_backward_pwm = pins.d5.into_output().into_pwm(&timer0);
        let right_motor = Motor::new(right_enable_motor, right_forward_pwm, right_backward_pwm);

        // Configure PID
        let left_pid_controller = Pid::new(-PWM_MAX, PWM_MAX, PID_KP, PID_KI, PID_KD, PID_KO);
        let right_pid_controller = Pid::new(-PWM_MAX, PWM_MAX, PID_KP, PID_KI, PID_KD, PID_KO);

        // Configure ENCODERS
        let mut exint = dp.EXINT;
        let left_channel_a_interrupt_in = pins.d2.into_interrupt();
        let left_channel_b_interrupt_in = pins.d3.into_interrupt();
        let mut left_encoder =
            Encoder::new(left_channel_a_interrupt_in, left_channel_b_interrupt_in);
        left_encoder.attach(left_encoder_isr, &mut exint);

        let right_channel_a_interrupt_in = pins.a2.into_interrupt();
        let right_channel_b_interrupt_in = pins.a3.into_interrupt();
        let mut right_encoder =
            Encoder::new(right_channel_a_interrupt_in, right_channel_b_interrupt_in);
        right_encoder.attach(right_encoder_isr, &mut exint);

        interrupt::free(|cs| {
            *LEFT_ENCODER.borrow(cs).borrow_mut() = Some(left_encoder);
            *RIGHT_ENCODER.borrow(cs).borrow_mut() = Some(right_encoder);
        });

        // Configure SHELL
        let serial = arduino_hal::default_serial!(dp, pins, BAUDRATE);
        let serial_stream = SerialStream::new(serial);
        let shell = Shell::new();

        Self {
            left_motor,
            right_motor,
            left_pid_controller,
            right_pid_controller,
            serial_stream,
            shell,
            last_pid_computation: 0,
            last_set_motors_speed_cmd: 0,
        }
    }

    /// Initializes the robot and its components.
    ///
    /// This method should be called once before entering the main control loop.
    /// It configures hardware components and prepares the robot for operation.
    pub fn setup(&mut self) {
        // Enable global interrupts
        unsafe { avr_device::interrupt::enable() };

        self.left_motor.enable();
        self.right_motor.enable();

        let (left_count, right_count) = read_encoders_count();
        self.left_pid_controller.reset(left_count);
        self.right_pid_controller.reset(right_count);

        self.register_commands();
    }

    /// Registers all Andino firmware commands
    fn register_commands(&mut self) {
        self.shell
            .register_command(
                Commands::ResetEncoders.try_from(),
                Commands::ResetEncoders.get_num_params(),
            )
            .or_log_and_panic(&mut self.serial_stream);
        self.shell
            .register_command(
                Commands::ReadEncoders.try_from(),
                Commands::ReadEncoders.get_num_params(),
            )
            .or_log_and_panic(&mut self.serial_stream);
        self.shell
            .register_command(
                Commands::SetMotorsSpeed.try_from(),
                Commands::SetMotorsSpeed.get_num_params(),
            )
            .or_log_and_panic(&mut self.serial_stream);
        self.shell
            .register_command(
                Commands::SetMotorsPwm.try_from(),
                Commands::SetMotorsPwm.get_num_params(),
            )
            .or_log_and_panic(&mut self.serial_stream);
        self.shell
            .register_command(
                Commands::SetPidsTuningGains.try_from(),
                Commands::SetPidsTuningGains.get_num_params(),
            )
            .or_log_and_panic(&mut self.serial_stream);
    }

    /// Updates the robot state in the control loop.
    ///
    /// This method should be called repeatedly in the main control loop.
    /// It reads sensors, updates control algorithms, and drives actuators.
    pub fn update(&mut self) {
        // Process input and handle any errors
        if let Some(Command { name, args }) = self.shell.process_input(&mut self.serial_stream) {
            match Commands::try_from_str(name.as_str()).unwrap() {
                Commands::ResetEncoders => self
                    .cmd_reset_encoders()
                    .or_log_error(&mut self.serial_stream),
                Commands::ReadEncoders => self
                    .cmd_read_encoders()
                    .or_log_error(&mut self.serial_stream),
                Commands::SetMotorsSpeed => self
                    .cmd_set_motors_speed(args)
                    .or_log_error(&mut self.serial_stream),
                Commands::SetMotorsPwm => self
                    .cmd_set_motors_speed_pwm(args)
                    .or_log_error(&mut self.serial_stream),
                Commands::SetPidsTuningGains => self
                    .cmd_set_pid_tuning_gains(args)
                    .or_log_error(&mut self.serial_stream),
                _ => {}
            }
        }

        let now = millis();
        if now - self.last_pid_computation > PID_PERIOD_MS {
            self.last_pid_computation = now;
            self.adjust_motors_speed();
        }

        if now - self.last_set_motors_speed_cmd > AUTO_STOP_WINDOW {
            self.last_set_motors_speed_cmd = now;
            self.stop_motors();
        }
    }

    /// Adjust motors speed based on encoder count and PID computation
    fn adjust_motors_speed(&mut self) {
        let (left_encoder_count, right_encoder_count) = read_encoders_count();
        let left_speed_result = self.left_pid_controller.compute(left_encoder_count);
        let right_speed_result = self.right_pid_controller.compute(right_encoder_count);

        if self.left_pid_controller.is_enabled() {
            if let Some(left_speed) = left_speed_result {
                self.left_motor.set_speed(left_speed);
            }
        }
        if self.right_pid_controller.is_enabled() {
            if let Some(right_speed) = right_speed_result {
                self.right_motor.set_speed(right_speed);
            }
        }
    }

    fn stop_motors(&mut self) {
        self.left_motor.set_speed(0);
        self.right_motor.set_speed(0);
        self.left_pid_controller.disable();
        self.right_pid_controller.disable();
    }

    /// Reset the encoders.
    ///
    /// # Returns
    /// Execution result
    fn cmd_reset_encoders(&mut self) -> Result<(), CommandError> {
        reset_encoders_count();
        let (left_encoder_count, right_encoder_count) = read_encoders_count();
        self.left_pid_controller.reset(left_encoder_count);
        self.right_pid_controller.reset(right_encoder_count);
        ufmt::uwriteln!(self.serial_stream, "OK").ok();
        Ok(())
    }

    /// Read the encoders.
    ///
    /// # Returns
    /// Execution result
    fn cmd_read_encoders(&mut self) -> Result<(), CommandError> {
        let (left_encoder_count, right_encoder_count) = read_encoders_count();
        ufmt::uwriteln!(
            self.serial_stream,
            "{} {}",
            left_encoder_count,
            right_encoder_count
        )
        .ok();
        Ok(())
    }

    /// Set the motors speed
    ///
    /// # Arguments
    /// * `params` - Expected args are left_motor_speed and right_motor_speed
    ///
    /// # Returns
    /// Execution result
    fn cmd_set_motors_speed(&mut self, args: CommandArgs) -> Result<(), CommandError> {
        let left_motor_speed: i32 = args[0]
            .parse()
            .map_err(|_| CommandError::InvalidParameter)?;
        let right_motor_speed: i32 = args[1]
            .parse()
            .map_err(|_| CommandError::InvalidParameter)?;

        self.last_set_motors_speed_cmd = millis();
        if left_motor_speed == 0 && right_motor_speed == 0 {
            self.left_motor.set_speed(0);
            self.right_motor.set_speed(0);

            let (left_encoder_count, right_encoder_count) = read_encoders_count();

            self.left_pid_controller.reset(left_encoder_count);
            self.right_pid_controller.reset(right_encoder_count);
            self.left_pid_controller.disable();
            self.right_pid_controller.disable();
        } else {
            self.left_pid_controller.enable();
            self.right_pid_controller.enable();
        }

        // The target speeds are in ticks per second, so we need to convert them to ticks per
        // PID_RATE.
        self.left_pid_controller
            .set_setpoint(left_motor_speed / PID_RATE as i32);
        self.right_pid_controller
            .set_setpoint(right_motor_speed / PID_RATE as i32);

        ufmt::uwriteln!(self.serial_stream, "OK").ok();
        Ok(())
    }

    /// Set the motors speed in PWM
    ///
    /// # Arguments
    /// * `params` - Expected args are left_motor_speed_pwm and right_motor_speed_pwm
    ///
    /// # Returns
    /// Execution result
    fn cmd_set_motors_speed_pwm(&mut self, args: CommandArgs) -> Result<(), CommandError> {
        let left_motor_speed_pwm: i32 = args[0]
            .parse()
            .map_err(|_| CommandError::InvalidParameter)?;
        let right_motor_speed_pwm: i32 = args[1]
            .parse()
            .map_err(|_| CommandError::InvalidParameter)?;

        self.last_set_motors_speed_cmd = millis();

        let (left_encoder_count, right_encoder_count) = read_encoders_count();

        self.left_pid_controller.reset(left_encoder_count);
        self.right_pid_controller.reset(right_encoder_count);
        self.left_pid_controller.disable();
        self.right_pid_controller.disable();

        self.left_motor.set_speed(left_motor_speed_pwm);
        self.right_motor.set_speed(right_motor_speed_pwm);
        ufmt::uwriteln!(self.serial_stream, "OK").ok();
        Ok(())
    }

    /// Set PID tuning gains
    ///
    /// # Arguments
    /// * `params` - Expected args are kp, kd, ki, ko
    ///
    /// # Returns
    /// Execution result
    fn cmd_set_pid_tuning_gains(&mut self, args: CommandArgs) -> Result<(), CommandError> {
        let kp: i32 = args[0]
            .parse()
            .map_err(|_| CommandError::InvalidParameter)?;
        let ki: i32 = args[2]
            .parse()
            .map_err(|_| CommandError::InvalidParameter)?;
        let kd: i32 = args[1]
            .parse()
            .map_err(|_| CommandError::InvalidParameter)?;
        let ko: i32 = args[3]
            .parse()
            .map_err(|_| CommandError::InvalidParameter)?;

        self.left_pid_controller.set_tunings(kp, ki, kd, ko);
        self.right_pid_controller.set_tunings(kp, ki, kd, ko);
        ufmt::uwriteln!(
            self.serial_stream,
            "PID Updated: {} {} {} {}",
            kp,
            kd,
            ki,
            ko
        )
        .ok();
        ufmt::uwriteln!(self.serial_stream, "OK").ok();
        Ok(())
    }
}
