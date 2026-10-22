/*!
* Fade
*
* Example of using Pwm to fade a LED in and out on pin d5.
*
* The circuit:
* - LED attached from digital pin 5 to ground through 220 ohm resistor.
*/

#![no_std]
#![no_main]

use arduino_hal::{
    delay_ms,
    simple_pwm::{IntoPwmPin, Prescaler, Timer0Pwm},
};
use embedded_hal::pwm::SetDutyCycle;
use panic_halt as _;

#[arduino_hal::entry]
fn main() -> ! {
    // Get the Peripherals singleton
    let dp = arduino_hal::Peripherals::take().unwrap();
    let pins = arduino_hal::pins!(dp);

    // Initialize PWM Timers
    let timer0 = Timer0Pwm::new(dp.TC0, Prescaler::Prescale64);

    // Create a PWM pin
    let mut pwm_led = pins.d5.into_output().into_pwm(&timer0);
    pwm_led.enable();

    // Fade LED on and off
    loop {
        for pct in (0..=100).chain((0..100).rev()) {
            pwm_led.set_duty_cycle_percent(pct).unwrap();
            delay_ms(10);
        }
    }
}
