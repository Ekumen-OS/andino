//! Utility modules for managing Arduino Nano (Atmel328p) hardware peripherals.

pub mod interrupt;
pub mod millis;
pub mod serial;

/// Serial Usart0 interface type
pub type SerialType = arduino_hal::hal::usart::Usart0<arduino_hal::DefaultClock>;

/// Serial string size
pub const SERIAL_STRING_SIZE: usize = 64;

/// Serial fixed sized String
pub type SerialString = heapless::String<SERIAL_STRING_SIZE>;
