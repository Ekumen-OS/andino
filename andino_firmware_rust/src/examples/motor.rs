/*!
* Motor
*
* Commands an Andino motor to move forward and backward.
*/

#![no_std]
#![no_main]

use andino_firmware_rust::components::motor::Motor;
use arduino_hal::{
    delay_ms,
    simple_pwm::{IntoPwmPin, Prescaler, Timer0Pwm, Timer1Pwm},
};
use panic_halt as _;

#[arduino_hal::entry]
fn main() -> ! {
    // Get the Peripherals singleton
    let dp = arduino_hal::Peripherals::take().unwrap();
    let pins = arduino_hal::pins!(dp);

    // Initialize PWM Timers
    let timer0 = Timer0Pwm::new(dp.TC0, Prescaler::Prescale64);
    let timer1 = Timer1Pwm::new(dp.TC1, Prescaler::Prescale64);

    // Initialize PWM and Digital pins
    let enable_motor_pin = pins.d13.into_output();
    let forward_pwm = pins.d10.into_output().into_pwm(&timer1);
    let backward_pwm = pins.d6.into_output().into_pwm(&timer0);

    // Create a new motor instance
    let mut motor = Motor::new(enable_motor_pin, forward_pwm, backward_pwm);

    // Enable the motor
    motor.enable();

    // Set motor speed (positive for forward, negative for backward)
    loop {
        motor.set_speed(150); // Forward at ~59% speed
        delay_ms(2000);
        motor.set_speed(-150); // Backwards at ~59% speed
        delay_ms(2000);
    }
}
