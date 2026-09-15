/*!
* Button
*
* Turns on and off an LED connected to digital pin 13,
* when pressing a pushbutton attached to pin 2.
*
* The circuit:
* - LED attached from pin 13 to ground through 220 ohm resistor.
* - Push button attached to pin 2 from +5V.
* - 10K resistor attached to pin 2 from ground.
*/

#![no_std]
#![no_main]

use arduino_hal::delay_ms;
use panic_halt as _;

#[arduino_hal::entry]
fn main() -> ! {
    // Get the Peripherals singleton
    let dp = arduino_hal::Peripherals::take().unwrap();
    let pins = arduino_hal::pins!(dp);

    // Create a digital input pin
    let button = pins.d2.into_pull_up_input();
    // Create a digital output pin
    let mut led = pins.d13.into_output();

    // Turn on LED if button is pressed.
    loop {
        if button.is_high() {
            led.set_high();
        } else {
            led.set_low();
        }
        delay_ms(100);
    }
}
