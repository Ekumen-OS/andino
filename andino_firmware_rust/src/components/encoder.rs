//! Quadrature Encoder utilities for reading rotary encoders for the Andino robot.

use arduino_hal::{pac::EXINT, port::PinOps};

use crate::util::interrupt::{InterruptCallback, InterruptIn};

/// Ticks delta lookup table. Its content is defined as follows:
///   +--------+-----+-----+--------+-----------+
///   | Number | Old | New | Binary | Direction |
///   |        | A B | A B |        |           |
///   +--------+-----+-----+--------+-----------+
///   |    0   | 0 0 | 0 0 |  0000  |     0     |
///   |    1   | 0 0 | 0 1 |  0001  |    +1     |
///   |    2   | 0 0 | 1 0 |  0010  |    -1     |
///   |    3   | 0 0 | 1 1 |  0011  |     0     |
///   |    4   | 0 1 | 0 0 |  0100  |    -1     |
///   |    5   | 0 1 | 0 1 |  0101  |     0     |
///   |    6   | 0 1 | 1 0 |  0110  |     0     |
///   |    7   | 0 1 | 1 1 |  0111  |    +1     |
///   |    8   | 1 0 | 0 0 |  1000  |    +1     |
///   |    9   | 1 0 | 0 1 |  1001  |     0     |
///   |   10   | 1 0 | 1 0 |  1010  |     0     |
///   |   11   | 1 0 | 1 1 |  1011  |    -1     |
///   |   12   | 1 1 | 0 0 |  1100  |     0     |
///   |   13   | 1 1 | 0 1 |  1101  |    -1     |
///   |   14   | 1 1 | 1 0 |  1110  |    +1     |
///   |   15   | 1 1 | 1 1 |  1111  |     0     |
///   +--------+-----+-----+--------+-----------+
static K_TICKS_DELTA: [i32; 16] = [0, 1, -1, 0, -1, 0, 0, 1, 1, 0, 0, -1, 0, -1, 1, 0];

/// A quadrature encoder decoder for rotary encoders with two channels (A and B).
///
/// ### Notes
///
/// - Encoder pins must support pin change interrupts (see [`InterruptIn`] documentation)
/// - Both channels must be on the same PCINT group for a single ISR callback.
///
/// ### Examples
/// ```
/// use andino_firmware_rust::util::interrupt::IntoInterrupt;
///
/// // Global storage for encoder (required for ISR access)
/// static ENCODER: Mutex<RefCell<Option<Encoder<_, _>>>> =
///     Mutex::new(RefCell::new(None));
///
/// // ISR callback function
/// fn encoder_isr(pci_group: u8) {
///     interrupt::free(|cs| {
///         if let Some(ref mut enc) = ENCODER.borrow(cs).borrow_mut().as_mut() {
///             enc.handle_interrupt(pci_group);
///         }
///     });
/// }
///
/// fn main() {
///     let dp = arduino_hal::Peripherals::take().unwrap();
///     let pins = arduino_hal::pins!(dp);
///     let mut exint = dp.EXINT;
///
///     // Create encoder with channels A and B
///     let channel_a = pins.d2.into_interrupt();
///     let channel_b = pins.d3.into_interrupt();
///     let mut encoder = Encoder::new(channel_a, channel_b);
///
///     // Attach interrupt handler
///     encoder.attach(encoder_isr, &mut exint);
///
///     // Store encoder in static for ISR access
///     interrupt::free(|cs| {
///         *ENCODER.borrow(cs).borrow_mut() = Some(encoder);
///     });
///
///     // Enable global interrupts
///     unsafe { avr_device::interrupt::enable() };
///
///     loop {
///         // Read encoder position
///         let position = interrupt::free(|cs| {
///             ENCODER.borrow(cs).borrow()
///                 .as_ref()
///                 .map(|e| e.read())
///                 .unwrap_or(0)
///         });
///
///         // Use position...
///     }
/// }
/// ```
pub struct Encoder<PIN1, PIN2> {
    /// Encoder channel A
    channel_a_interrupt_in: InterruptIn<PIN1>,
    /// Encoder channel B
    channel_b_interrupt_in: InterruptIn<PIN2>,

    /// Encoder state. It contains both the current and previous channels state readings:
    ///   +------+-----+-----+-----+-----+-----+-----+-----+-----+
    ///   | Bits |  7  |  6  |  5  |  4  |  3  |  2  |  1  |  0  |
    ///   +------+-----+-----+-----+-----+-----+-----+-----+-----+
    ///   |      |  x  |  x  |  x  |  x  | PREVIOUS  |  CURRENT  |
    ///   +------+-----+-----+-----+-----+-----+-----+-----+-----+
    state: u8,
    /// Ticks count.
    count: i32,
    /// PCI Group the interrupt pins belong to
    pci_group_interrup_pins: u8,
}

impl<PIN1, PIN2> Encoder<PIN1, PIN2>
where
    PIN1: PinOps,
    PIN2: PinOps,
{
    /// Creates a new `Encoder` instance.
    ///
    /// # Arguments
    /// * `channel_a_interrupt_in` - Interrupt-capable input pin for encoder channel A
    /// * `channel_b_interrupt_in` - Interrupt-capable input pin for encoder channel B
    ///
    /// # Returns
    /// A new `Encoder` instance with position initialized to zero.
    pub fn new(
        channel_a_interrupt_in: InterruptIn<PIN1>,
        channel_b_interrupt_in: InterruptIn<PIN2>,
    ) -> Self {
        Self {
            channel_a_interrupt_in,
            channel_b_interrupt_in,
            state: 0x00,
            count: 0,
            pci_group_interrup_pins: 0,
        }
    }

    /// Attaches an interrupt callback and enables interrupts for both encoder channels.
    ///
    /// # Arguments
    /// * `callback` - The function to call when either encoder pin changes state.
    /// * `exint` - Mutable reference to the external interrupt peripheral.
    pub fn attach(&mut self, callback: InterruptCallback, exint: &mut EXINT) {
        self.channel_a_interrupt_in.attach(callback, exint);
        self.channel_b_interrupt_in.attach(callback, exint);
        self.pci_group_interrup_pins = self.channel_a_interrupt_in.pci_group();
    }

    /// Returns current tickets count.
    ///
    /// # Returns
    /// Ticks count
    pub fn read(&self) -> i32 {
        self.count
    }

    /// Resets the encoder position to zero.
    pub fn reset(&mut self) {
        self.count = 0
    }

    /// Handles encoder state updates during an interrupt.
    ///
    /// # Arguments
    /// * `pci_group` - PCI group invoked by the interruption.
    pub fn handle_interrupt(&mut self, pci_group: u8) {
        // Determine if the interruption is for this encoder
        if pci_group != self.pci_group_interrup_pins {
            return;
        }

        // Shifts the previous state into bits 2-3
        self.state <<= 2;

        // Read current state into bits 0-1
        // Bit 1 = Channel B, Bit 0 = Channel A
        self.state |= ((self.channel_b_interrupt_in.is_high() as u8) << 1)
            | (self.channel_a_interrupt_in.is_high() as u8);

        // Use lower 4 bits as index into lookup table to determine position change
        self.count += K_TICKS_DELTA[(self.state & 0x0F) as usize];
    }
}
