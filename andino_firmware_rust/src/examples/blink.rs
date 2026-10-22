/*!
* Blink
*
* Turns an LED on for one second, then off for one second, repeatedly.
*/

#![no_std]
#![no_main]

use arduino_hal::delay_ms;
use panic_halt as _;

#[arduino_hal::entry]
fn main() -> ! {
    let delay: u32 = 1000;

    // Get the Peripherals singleton
    let dp = arduino_hal::Peripherals::take().unwrap();
    let pins = arduino_hal::pins!(dp);

    // Create a digital output pin
    let mut led = pins.d13.into_output();

    // Blink LED
    loop {
        led.set_high();
        delay_ms(delay);
        led.set_low();
        delay_ms(delay);
    }
}
