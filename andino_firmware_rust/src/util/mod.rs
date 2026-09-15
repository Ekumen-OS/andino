//! Utility modules for managing Arduino Nano (Atmel328p) hardware peripherals.

use arduino_hal::{
    hal::port::{PD0, PD1},
    pac::USART0,
    port::{
        mode::{Input, Output},
        Pin,
    },
    usart::{UsartReader, UsartWriter},
};

pub mod interrupt;
pub mod millis;
pub mod serial;

/// Serial Usart0 interface type
pub type SerialType = arduino_hal::hal::usart::Usart0<arduino_hal::DefaultClock>;
pub type SerialReaderType = UsartReader<USART0, Pin<Input, PD0>, Pin<Output, PD1>>;
pub type SerialWriterType = UsartWriter<USART0, Pin<Input, PD0>, Pin<Output, PD1>>;
// pub type SerialWriterType = UsartWriter<Atmega, USART0, Pin<Input, PD0>, Pin<Output, PD1>, MHz16>;

/// Serial string size
pub const SERIAL_STRING_SIZE: usize = 64;

/// Serial fixed sized String
pub type SerialString = heapless::String<SERIAL_STRING_SIZE>;
