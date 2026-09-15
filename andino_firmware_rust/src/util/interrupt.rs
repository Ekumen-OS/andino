//! Utilities for managing Arduino interrupt-capable digital input pins.
//!
//! This module provides Pin Change Interrupt (PCINT) functionality for all digital pins on the
//! ATmega328p microcontroller. Unlike external interrupts (INT0/INT1) which are only available
//! on pins D2 and D3, pin change interrupts can be configured on any digital pin.

use arduino_hal::hal::port::{
    PB0, PB1, PB2, PB3, PB4, PB5, PC0, PC1, PC2, PC3, PC4, PC5, PD0, PD1, PD2, PD3, PD4, PD5, PD6,
    PD7,
};
use arduino_hal::pac::EXINT;
use arduino_hal::port::mode::{Floating, Input, PullUp};
use arduino_hal::port::{Pin, PinOps};
use avr_device::interrupt;

/// Type alias for interrupt callback functions.
///
/// # Arguments
/// * `pci_group` - Use to indicate the PCI function being trigger [0, 1 or 2].
pub type InterruptCallback = fn(pci_group: u8);

/// Static storage for interrupt callbacks, one for each PCINT group.
static mut INTERRUPT_CALLBACKS: [Option<InterruptCallback>; 3] = [None; 3];
/// Trait to extract Pin Change INTerrupt information from pin types.
///
/// This trait provides the necessary information to configure pin change interrupts
/// for ATmega328p pins, including which PCIE group and bit position within the PCMSK register.
pub trait PcintInfo {
    /// Get the PCI Enable bit this pin belongs to (0, 1, or 2)
    fn pcie() -> u8;
    /// Get the bit position within the PCMSK register (0-7)
    fn pcmsk_bit() -> u8;
}

// Macro rule to implement PcintInfo trait to Atmel328p pins
macro_rules! impl_pcint_info_to_pin {
    ($pin_type:ty, $pcie:expr, $pcmsk_bit:expr) => {
        impl PcintInfo for $pin_type {
            fn pcie() -> u8 {
                $pcie
            }
            fn pcmsk_bit() -> u8 {
                $pcmsk_bit
            }
        }
    };
}

// Implement PcintInfo trait for all Atmel328p pins
// Port B pins (D8-D13) - PCIE0/PCMSK0
impl_pcint_info_to_pin!(PB0, 0, 0); // D8
impl_pcint_info_to_pin!(PB1, 0, 1); // D9
impl_pcint_info_to_pin!(PB2, 0, 2); // D10
impl_pcint_info_to_pin!(PB3, 0, 3); // D11
impl_pcint_info_to_pin!(PB4, 0, 4); // D12
impl_pcint_info_to_pin!(PB5, 0, 5); // D13

// Port C pins (A0-A5) - PCIE1/PCMSK1
impl_pcint_info_to_pin!(PC0, 1, 0); // A0
impl_pcint_info_to_pin!(PC1, 1, 1); // A1
impl_pcint_info_to_pin!(PC2, 1, 2); // A2
impl_pcint_info_to_pin!(PC3, 1, 3); // A3
impl_pcint_info_to_pin!(PC4, 1, 4); // A4
impl_pcint_info_to_pin!(PC5, 1, 5); // A5

// Port D pins (D0-D7) - PCIE2/PCMSK2
impl_pcint_info_to_pin!(PD0, 2, 0); // D0
impl_pcint_info_to_pin!(PD1, 2, 1); // D1
impl_pcint_info_to_pin!(PD2, 2, 2); // D2
impl_pcint_info_to_pin!(PD3, 2, 3); // D3
impl_pcint_info_to_pin!(PD4, 2, 4); // D4
impl_pcint_info_to_pin!(PD5, 2, 5); // D5
impl_pcint_info_to_pin!(PD6, 2, 6); // D6
impl_pcint_info_to_pin!(PD7, 2, 7); // D7

// Interrupt handlers for each PCINT group

// Handle pins D8-D13 (Port B)
#[avr_device::interrupt(atmega328p)]
#[allow(non_snake_case)]
fn PCINT0() {
    unsafe {
        if let Some(callback) = INTERRUPT_CALLBACKS[0] {
            callback(0);
        }
    }
}

// Handle pins A0-A5 (Port C)
#[avr_device::interrupt(atmega328p)]
#[allow(non_snake_case)]
fn PCINT1() {
    unsafe {
        if let Some(callback) = INTERRUPT_CALLBACKS[1] {
            callback(1);
        }
    }
}

// Handle pins D0-D7 (Port D)
#[avr_device::interrupt(atmega328p)]
#[allow(non_snake_case)]
fn PCINT2() {
    unsafe {
        if let Some(callback) = INTERRUPT_CALLBACKS[2] {
            callback(2);
        }
    }
}

/// A wrapper for digital input pins with pin change interrupt capability.
///
/// `InterruptIn` encapsulates an Arduino pin configured for digital input mode with
/// pin change interrupt functionality. Pin change interrupts trigger on any logical
/// change (rising or falling edge) on the pin.
///
/// ### Pin Change Interrupt Groups
/// The ATmega328p organizes pins into three PCINT groups:
/// ```text
/// | Arduino Pin | AVR Pin | PCINT Group  | PCMSK Bit | Interrupt Vector |
/// | ----------- | ------- | ------------ | --------- | ---------------- |
/// | D8-D13      | PB0-PB5 | PCIE0/PCMSK0 | 0-5       | PCINT0           |
/// | A0-A5       | PC0-PC5 | PCIE1/PCMSK1 | 0-5       | PCINT1           |
/// | D0-D7       | PD0-PD7 | PCIE2/PCMSK2 | 0-7       | PCINT2           |
/// ```
///
/// ### Examples
/// ```
/// use util::interrupt::IntoInterrupt;
///
/// // Define callback function
/// fn my_interrupt_handler() {
///     // Handle interrupt here
/// }
///
/// fn main() {
///     let p = arduino_hal::Peripherals::take().unwrap();
///     let pins = arduino_hal::pins!(p);
///     let mut exint = p.EXINT;
///
///     // Convert pin to interrupt-capable input (bring IntoInterrupt into scope first)
///     let interrupt_pin = pins.d2.into_interrupt();
///
///     // Attach callback and enable interrupt
///     interrupt_pin.attach(my_interrupt_handler, &mut exint);
///
///     // Enable global interrupts
///     unsafe { avr_device::interrupt::enable() };
///
///     // Read current state
///     let state = interrupt_pin.read();
/// }
/// ```
pub struct InterruptIn<PIN> {
    /// The underlying Arduino pin in input mode.
    pin: Pin<Input<PullUp>, PIN>,
    /// The PCI group this pin belongs to (0, 1, or 2).
    pci_group: u8,
    /// The bit position within the PCMSK register.
    pcmsk_bit: u8,
}

/// Extension trait for converting a digital input pin into an interrupt-capable [`InterruptIn`].
pub trait IntoInterrupt<PIN> {
    /// Converts this digital input pin into an interrupt-capable pin.
    fn into_interrupt(self) -> InterruptIn<PIN>;
}

impl<PIN> IntoInterrupt<PIN> for Pin<Input<PullUp>, PIN>
where
    PIN: PcintInfo + PinOps,
{
    fn into_interrupt(self) -> InterruptIn<PIN> {
        InterruptIn {
            pin: self,
            pci_group: PIN::pcie(),
            pcmsk_bit: PIN::pcmsk_bit(),
        }
    }
}

impl<PIN> IntoInterrupt<PIN> for Pin<Input<Floating>, PIN>
where
    PIN: PcintInfo + PinOps,
{
    /// Converts this floating input pin into an interrupt-capable pin, enabling the internal pull-up.
    fn into_interrupt(self) -> InterruptIn<PIN> {
        InterruptIn {
            pin: self.into_pull_up_input(),
            pci_group: PIN::pcie(),
            pcmsk_bit: PIN::pcmsk_bit(),
        }
    }
}

impl<PIN> InterruptIn<PIN>
where
    PIN: PinOps,
{
    /// Enables pin change interrupt for this pin.
    ///
    /// # Arguments
    /// * `exint` - Mutable reference to the external interrupt peripheral.
    ///
    /// # Note
    /// This method is private. Use the `attach()` method instead to set up
    /// interrupt callbacks and automatically enable interrupts.
    #[allow(unused_unsafe)]
    fn enable(&self, exint: &mut EXINT) {
        // Enable the PCI group in PCICR
        exint
            .pcicr
            .modify(|r, w| unsafe { w.bits(r.bits() | (1 << self.pci_group)) });

        // Enable the specific pin in the appropriate PCMSK register
        match self.pci_group {
            0 => exint
                .pcmsk0
                .modify(|r, w| unsafe { w.bits(r.bits() | (1 << self.pcmsk_bit)) }),
            1 => exint
                .pcmsk1
                .modify(|r, w| unsafe { w.bits(r.bits() | (1 << self.pcmsk_bit)) }),
            2 => exint
                .pcmsk2
                .modify(|r, w| unsafe { w.bits(r.bits() | (1 << self.pcmsk_bit)) }),
            _ => (), // Should never happen with valid pins
        };
    }

    /// Disables pin change interrupt for this pin.
    ///
    /// # Arguments
    /// * `exint` - Mutable reference to the external interrupt peripheral.
    ///
    /// # Note
    /// The method removes the pin from the pin change interrupt system.
    /// The PCI group will remain enabled if other pins in the same group
    /// are still configured for interrupts.
    #[allow(unused_unsafe)]
    pub fn disable(&self, exint: &mut EXINT) {
        // Disable the specific pin in the appropriate PCMSK register
        match self.pci_group {
            0 => exint
                .pcmsk0
                .modify(|r, w| unsafe { w.bits(r.bits() & !(1 << self.pcmsk_bit)) }),
            1 => exint
                .pcmsk1
                .modify(|r, w| unsafe { w.bits(r.bits() & !(1 << self.pcmsk_bit)) }),
            2 => exint
                .pcmsk2
                .modify(|r, w| unsafe { w.bits(r.bits() & !(1 << self.pcmsk_bit)) }),
            _ => (), // Should never happen with valid pins
        };

        // Optionally disable the PCI group if no pins are enabled
        let pcmsk_value = match self.pci_group {
            0 => exint.pcmsk0.read().bits(),
            1 => exint.pcmsk1.read().bits(),
            2 => exint.pcmsk2.read().bits(),
            _ => 0,
        };

        // If no pins are enabled in this group, disable the PCI
        if pcmsk_value == 0 {
            exint
                .pcicr
                .modify(|r, w| unsafe { w.bits(r.bits() & !(1 << self.pci_group)) });
        }
    }

    /// Reads the current state of the interrupt input pin.
    ///
    /// # Returns
    /// * `true` if the pin is high, `false` if the pin is low.
    pub fn is_high(&self) -> bool {
        self.pin.is_high()
    }

    /// Gets the PCI group number for this pin.
    ///
    /// # Returns
    /// The PCI group number (0, 1, or 2) that this pin belongs to.
    pub fn pci_group(&self) -> u8 {
        self.pci_group
    }

    /// Gets the bit position within the PCMSK register for this pin.
    ///
    /// # Returns
    /// The bit position (0-7) within the PCMSK register for this pin.
    pub fn pcmsk_bit(&self) -> u8 {
        self.pcmsk_bit
    }

    /// Attaches a callback function to the corresponding PCINT interrupt and enables it.
    ///
    /// # Arguments
    /// * `callback` - The function to call when the interrupt fires
    /// * `exint` - Mutable reference to the external interrupt peripheral
    ///
    /// # Important Note
    /// All pins within the same PCINT group share a single callback function.
    /// If you call `attach()` on multiple pins from the same group, only the
    /// most recent callback will be used for all pins in that group.
    ///
    /// **PCINT Groups:**
    /// - Group 0: Pins D8-D13 (Port B) - share one callback
    /// - Group 1: Pins A0-A5 (Port C) - share one callback  
    /// - Group 2: Pins D0-D7 (Port D) - share one callback
    pub fn attach(&self, callback: InterruptCallback, exint: &mut EXINT) {
        interrupt::free(|_| unsafe {
            INTERRUPT_CALLBACKS[self.pci_group as usize] = Some(callback);
        });
        self.enable(exint);
    }
}
