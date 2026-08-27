//! Non-blocking serial interface using interrupt-driven I/O.

use core::{cell::RefCell, convert::Infallible};

use arduino_hal::prelude::*;
use avr_device::interrupt::{self, Mutex};
use heapless::spsc::{Consumer, Producer, Queue};

use super::{SerialString, SerialType, SERIAL_STRING_SIZE};

/// Global storage for the USART Serial peripheral, accessible from the ISR.
static SERIAL: Mutex<RefCell<Option<SerialType>>> = Mutex::new(RefCell::new(None));

/// Producer end of the queue, used by the ISR to enqueue received bytes.
static mut SERIAL_QUEUE_PRODUCER: Option<Producer<'static, u8>> = None;

/// A non-blocking wrapper for Arduino's USART (Serial) interface using interrupt-driven I/O.
///
/// `SerialStream` receives data in the background via interrupts, storing incoming bytes
/// in a lock-free queue. This allows the main program to check for available data and read
/// it without blocking.
///
/// When the 64-byte receive buffer is full, new incoming data is dropped.
///
/// ### Examples
/// ```rust
/// let dp = arduino_hal::Peripherals::take().unwrap();
/// let pins = arduino_hal::pins!(dp);
/// let serial = arduino_hal::default_serial!(dp, pins, 9600);
/// let mut serial_stream = SerialStream::new(serial);
///
/// // Enable global interrupts
/// unsafe { avr_device::interrupt::enable() };
///
/// loop {
///     if let Some(command) = serial_stream.read() {
///         ufmt::uwriteln!(&mut serial_stream, "Received: {}", command.as_str()).unwrap();
///     }
///     // Do other work without blocking on serial I/O
/// }
/// ```
pub struct SerialStream {
    /// Consumer end of the queue for reading received bytes.
    consumer: Consumer<'static, u8>,
    cache: SerialString,
}

impl SerialStream {
    /// Creates a new `SerialStream` and configures it for interrupt-driven reception.
    ///
    /// This method must be called exactly once.
    ///
    /// # Arguments
    /// * `serial` - A configured Arduino USART peripheral, typically obtained from
    ///   `arduino_hal::default_serial!()` macro.
    ///
    /// # Returns
    /// A new `SerialStream` instance. Remember to enable global interrupts with
    /// `unsafe { avr_device::interrupt::enable() }` after calling this.
    #[allow(static_mut_refs)]
    pub fn new(mut serial: SerialType) -> Self {
        // Enable RX complete interrupt on the USART peripheral
        serial.listen(arduino_hal::hal::usart::Event::RxComplete);

        // Store the serial peripheral for ISR access
        interrupt::free(|cs| {
            SERIAL.borrow(cs).replace(Some(serial));
        });

        // Create a function-scoped static queue
        static mut QUEUE: Queue<u8, SERIAL_STRING_SIZE> = Queue::new();
        let (producer, consumer) = unsafe { QUEUE.split() };

        // Store producer end for ISR
        unsafe {
            SERIAL_QUEUE_PRODUCER = Some(producer);
        }

        Self {
            consumer,
            cache: SerialString::new(),
        }
    }

    /// Checks if data is available to read from the queue.
    ///
    /// # Returns
    /// `true` if at least one byte is available to read, `false` otherwise.
    pub fn available(&self) -> bool {
        !self.consumer.is_empty()
    }

    /// Reads a line of text from the queue.
    ///
    /// This method consumes bytes from the queue until a line terminator is found
    /// or the queue is empty.
    ///
    /// # Returns
    /// `Some(SerialString)` containing the received text if a complete line was read,
    /// or `None` if no data is available.
    pub fn read(&mut self) -> Option<SerialString> {
        let found_termination = loop {
            match self.consumer.dequeue() {
                Some(b) => {
                    match b {
                        // Terminations: LF / CRLF
                        b'\n' | b'\r' => break true,
                        // Ignore control characters except printable ones
                        32..=126 => self.cache.push(b as char).unwrap(),
                        _ => continue,
                    }
                }
                None => break false,
            }
        };
        found_termination
            .then(|| core::mem::take(&mut self.cache))
            .filter(|s| !s.is_empty())
    }

    /// Writes a string to the serial interface.
    ///
    /// This is a blocking write operation that temporarily disables interrupts
    /// to safely access the shared USART peripheral.
    ///
    /// # Arguments
    /// * `s` - The string slice to write to the serial interface.
    ///
    /// # Returns
    /// `Result<(), Infallible>` Result of writing operation.
    pub fn write(&mut self, s: &str) -> Result<(), Infallible> {
        let mut result = Ok(());
        interrupt::free(|cs| {
            if let Some(serial) = SERIAL.borrow(cs).borrow_mut().as_mut() {
                result = serial.write_str(s)
            }
        });
        result
    }
}

/// Implementation of `ufmt::uWrite` trait for formatted writing support.
///
/// This allows the `SerialStream` to be used with `ufmt` macros like `uwrite!` and `uwriteln!`.
///
/// # Examples
/// ```rust
/// use ufmt::uWrite;
///
/// let temperature = 25;
/// ufmt::uwrite!(&mut serial_stream, "Temperature: {}°C\r\n", temperature).unwrap();
/// ```
impl ufmt::uWrite for SerialStream {
    type Error = Infallible;

    fn write_str(&mut self, s: &str) -> Result<(), Self::Error> {
        self.write(s)
    }
}

/// USART RX Complete interrupt service routine.
///
/// Triggered automatically when a byte is received on the USART. Reads the byte
/// and enqueues it for the main program to consume. If the queue is full (64 bytes),
/// the incoming byte is dropped.
#[avr_device::interrupt(atmega328p)]
#[allow(static_mut_refs)]
fn USART_RX() {
    interrupt::free(|cs| {
        if let Some(serial) = SERIAL.borrow(cs).borrow_mut().as_mut() {
            if let Ok(byte) = serial.read() {
                unsafe {
                    if let Some(producer) = SERIAL_QUEUE_PRODUCER.as_mut() {
                        // Silently drop incoming byte if queue is full
                        let _ = producer.enqueue(byte);
                    }
                }
            }
        }
    });
}
