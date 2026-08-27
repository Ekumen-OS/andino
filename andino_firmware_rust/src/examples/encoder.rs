/*!
* Encoder
*/

#![no_std]
#![no_main]

use core::cell::RefCell;

use andino_firmware_rust::components::encoder::Encoder;
use andino_firmware_rust::util::interrupt::IntoInterrupt;
use andino_firmware_rust::util::serial::SerialStream;
use arduino_hal::{
    delay_ms,
    hal::port::{PD2, PD3},
};
use avr_device::interrupt::{self, Mutex};
use panic_halt as _;

type EncoderType = Encoder<PD2, PD3>;
/// Static storage for the LED pin so it can be accessed from ISR
static ENCODER: Mutex<RefCell<Option<EncoderType>>> = Mutex::new(RefCell::new(None));

/// Attach ISR callback to encoder
fn encoder_isr(pci_group: u8) {
    interrupt::free(|cs| {
        if let Some(ref mut encoder) = ENCODER.borrow(cs).borrow_mut().as_mut() {
            encoder.handle_interrupt(pci_group);
        }
    });
}

/// Read encoder ticks count
fn read_count() -> i32 {
    let mut count = 0;
    interrupt::free(|cs| {
        if let Some(ref mut encoder) = ENCODER.borrow(cs).borrow_mut().as_mut() {
            count = encoder.read();
        }
    });
    count
}

#[arduino_hal::entry]
fn main() -> ! {
    // Get the Peripherals singleton
    let dp = arduino_hal::Peripherals::take().unwrap();
    let pins = arduino_hal::pins!(dp);
    let mut exint = dp.EXINT;

    let serial = arduino_hal::default_serial!(dp, pins, 57600);
    let mut serial_stream = SerialStream::new(serial);

    let channel_a_interrupt_in = pins.d2.into_interrupt();
    let channel_b_interrupt_in = pins.d3.into_interrupt();

    let mut encoder = Encoder::new(channel_a_interrupt_in, channel_b_interrupt_in);
    encoder.attach(encoder_isr, &mut exint);
    interrupt::free(|cs| {
        *ENCODER.borrow(cs).borrow_mut() = Some(encoder);
    });

    // Enable global interrupts
    unsafe { avr_device::interrupt::enable() };

    loop {
        delay_ms(100);
        let count = read_count();
        ufmt::uwriteln!(&mut serial_stream, "Count: {}!\r", count).unwrap();
    }
}
