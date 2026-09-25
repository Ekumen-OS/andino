/*!
* Serial
*/

#![no_std]
#![no_main]

use andino_firmware_rust::util::serial::SerialStream;
use arduino_hal::delay_ms;
use panic_halt as _;

#[arduino_hal::entry]
fn main() -> ! {
    let dp = arduino_hal::Peripherals::take().unwrap();
    let pins = arduino_hal::pins!(dp);
    let serial = arduino_hal::default_serial!(dp, pins, 57600);
    let mut serial_stream = SerialStream::new(serial);

    unsafe { avr_device::interrupt::enable() };

    ufmt::uwrite!(&mut serial_stream, "Type something: ").unwrap();
    loop {
        // Read a byte from the serial connection
        if serial_stream.available() {
            // Safe to unwrap since we checked there were available data
            let b = serial_stream.read().unwrap();
            ufmt::uwriteln!(&mut serial_stream, "Got {}!\r", b.as_str()).unwrap();
        }
        // Answer
        delay_ms(100);
    }
}
