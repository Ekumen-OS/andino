/*!
* Interrupt with Button
*
* Toggle LED every time the button interruption is triggered (both edges).
* The main program will print Hello World every 100ms.
*
* The circuit:
* - LED attached from pin 13 to ground through 220 ohm resistor.
* - Push button attached to pin 2 from +5V.
* - 10K resistor attached to pin 2 from ground.
*/

#![no_std]
#![no_main]

use andino_firmware_rust::util::interrupt::IntoInterrupt;
use andino_firmware_rust::util::serial::SerialStream;
use arduino_hal::delay_ms;
use arduino_hal::hal::port::PB5;
use arduino_hal::port::mode::Output;
use arduino_hal::port::Pin;
use avr_device::interrupt;
use core::cell::RefCell;
use panic_halt as _;

/// Static storage for the LED pin so it can be accessed from ISR
static LED: interrupt::Mutex<RefCell<Option<Pin<Output, PB5>>>> =
    interrupt::Mutex::new(RefCell::new(None));

/// ISR callback function to toggle the LED
fn toggle_led(_pci_group: u8) {
    interrupt::free(|cs| {
        if let Some(ref mut led) = LED.borrow(cs).borrow_mut().as_mut() {
            led.toggle();
        }
    });
}

#[arduino_hal::entry]
fn main() -> ! {
    // Get the Peripherals singleton
    let dp = arduino_hal::Peripherals::take().unwrap();
    let pins = arduino_hal::pins!(dp);
    let mut exint = dp.EXINT;

    // Create button interrupt interface
    let button = pins.d2.into_interrupt();

    // Store LED pin in static storage for ISR access
    let led_pin = pins.d13.into_output();
    interrupt::free(|cs| {
        *LED.borrow(cs).borrow_mut() = Some(led_pin);
    });

    // Create serial interface
    let serial = arduino_hal::default_serial!(dp, pins, 57600);
    let mut serial_stream = SerialStream::new(serial);

    // Attach callback and enable interrupt
    button.attach(toggle_led, &mut exint);

    // Enable global interrupts
    unsafe { avr_device::interrupt::enable() };

    // Main loop - print message every 100ms
    loop {
        ufmt::uwriteln!(&mut serial_stream, "Hello world \r").unwrap();
        delay_ms(100);
    }
}
