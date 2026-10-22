use crate::components::encoder::Encoder;
use crate::components::motor::Motor;
use arduino_hal::{
    hal::port::{PB1, PB2, PB4, PB5, PC2, PC3, PD2, PD3, PD5, PD6},
    simple_pwm::{Timer0Pwm, Timer1Pwm},
};

pub mod app;
pub mod constants;

type EncoderLeftMotorType = Encoder<PD2, PD3>;
type EncoderRightMotorType = Encoder<PC2, PC3>;
type LeftMotorType = Motor<PB5, PB2, PD6, Timer1Pwm, Timer0Pwm>;
type RightMotorType = Motor<PB4, PB1, PD5, Timer1Pwm, Timer0Pwm>;

/// Available Andino robot commands supported via serial stream
enum Commands {
    // Reads an analog GPIO.
    ReadAnalogGpio,
    // Reads a digital GPIO.
    ReadDigitalGpio,
    // Reads the encoders tick count values.
    ReadEncoders,
    // Sets the encoders ticks count to zero.
    ResetEncoders,
    // Sets the motors speed [ticks/s].
    SetMotorsSpeed,
    // Sets the motors PWM value [duty range: 0-255].
    SetMotorsPwm,
    // Sets the PIDs tuning gains [format: "kp:kd:ki:ko"].
    SetPidsTuningGains,
    // Gets whether there is an IMU sensor connected.
    GetIsImuConnected,
    // Reads the encoders tick count values and IMU sensor data.
    ReadEncodersAndImu,
}

impl Commands {
    /// Get string representation
    ///
    /// # Returns
    /// A slice string representing the enum
    pub fn try_from(&self) -> &str {
        match self {
            Commands::ReadAnalogGpio => "a",
            Commands::ReadDigitalGpio => "d",
            Commands::ReadEncoders => "e",
            Commands::ResetEncoders => "r",
            Commands::SetMotorsSpeed => "m",
            Commands::SetMotorsPwm => "o",
            Commands::SetPidsTuningGains => "u",
            Commands::GetIsImuConnected => "h",
            Commands::ReadEncodersAndImu => "i",
        }
    }

    /// Get number of params
    ///
    /// # Returns
    /// Number of params given the command
    pub fn get_num_params(&self) -> usize {
        match self {
            Commands::ReadAnalogGpio => 1,
            Commands::ReadDigitalGpio => 1,
            Commands::ReadEncoders => 0,
            Commands::ResetEncoders => 0,
            Commands::SetMotorsSpeed => 2,
            Commands::SetMotorsPwm => 2,
            Commands::SetPidsTuningGains => 4,
            Commands::GetIsImuConnected => 1,
            Commands::ReadEncodersAndImu => 1,
        }
    }

    /// Get the enum representation from the str
    ///
    /// # Arguments
    /// * `s` -  The expected slice string representation.
    ///
    /// # Returns
    /// An optional new Command enum
    pub fn try_from_str(s: &str) -> Option<Self> {
        match s {
            "a" => Some(Commands::ReadAnalogGpio),
            "d" => Some(Commands::ReadDigitalGpio),
            "e" => Some(Commands::ReadEncoders),
            "r" => Some(Commands::ResetEncoders),
            "m" => Some(Commands::SetMotorsSpeed),
            "o" => Some(Commands::SetMotorsPwm),
            "u" => Some(Commands::SetPidsTuningGains),
            "h" => Some(Commands::GetIsImuConnected),
            "i" => Some(Commands::ReadEncodersAndImu),
            _ => None,
        }
    }
}
