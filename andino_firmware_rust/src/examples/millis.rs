//! Millisecond Timer Example
//!
//! This example demonstrates the use of the `millis()` timer functionality for
//! non-blocking time tracking. It blinks the onboard LED every 5 seconds and
//! prints the elapsed time in milliseconds to the serial interface.
//!
//! # Hardware Setup
//! - Arduino Nano (ATmega328p)
//! - Built-in LED on pin D13
//! - Serial connection at 57600 baud
//!
//! # What This Example Shows
//! - Initializing the TC0 timer for millisecond counting
//! - Using `millis()` for non-blocking timing
//! - Periodic task execution without blocking delays
//! - Serial output of timing information
//!
//! # Expected Behavior
//! - LED toggles every 5 seconds
//! - Current millisecond count is printed to serial each time the LED toggles
//! - Main loop continues running without blocking, checking time on each iteration

#![no_std]
#![no_main]

use andino_firmware_rust::util::millis::{millis, millis_init};
use andino_firmware_rust::util::serial::SerialStream;
use arduino_hal::delay_ms;
use panic_halt as _;

#[arduino_hal::entry]
fn main() -> ! {
    const BLINK_PERIOD_MS: u32 = 5000; // LED toggle period (5 seconds)
    const DELAY_MS: u32 = 10; // Small delay to prevent busy-waiting (10ms)

    // Configure serial, timer and d13 digital pin
    let mut dp = arduino_hal::Peripherals::take().unwrap();
    let pins = arduino_hal::pins!(dp);

    let serial = arduino_hal::default_serial!(dp, pins, 57600);
    let mut serial_stream = SerialStream::new(serial);

    millis_init(&mut dp.TC2);

    let mut led = pins.d13.into_output();

    unsafe { avr_device::interrupt::enable() };

    let mut current_time = millis();
    loop {
        // Get current time in milliseconds
        let now = millis();

        // Check if it's time to toggle the LED (non-blocking check)
        if now - current_time > BLINK_PERIOD_MS {
            led.toggle();
            current_time = now;
            // Print elapsed time to serial monitor
            ufmt::uwriteln!(&mut serial_stream, "{}", now).unwrap();
        }

        // Small delay to reduce CPU usage while maintaining responsiveness
        delay_ms(DELAY_MS);
    }
}
