//! Millisecond timer using Timer/Counter0 (TC0) interrupt.
//!
//! This module provides an Arduino-like `millis()` function that returns the number of
//! milliseconds elapsed since the board was powered on or reset. It uses the ATmega328p's
//! Timer/Counter0 peripheral configured in CTC (Clear Timer on Compare Match) mode with
//! interrupts to maintain an accurate millisecond counter.
//!
//! ### Examples
//! ```rust
//! use andino_firmware_rust::util::millis::{millis, millis_init};
//!
//! let mut dp = arduino_hal::Peripherals::take().unwrap();
//! millis_init(&mut dp.TC0);
//!
//! // Enable global interrupts
//! unsafe { avr_device::interrupt::enable() };
//!
//! let start = millis();
//! // Do some work...
//! let elapsed = millis() - start;
//! ```

use core::cell::Cell;

use arduino_hal::pac::TC2;
use avr_device::interrupt;
use avr_device::interrupt::Mutex;

/// MCU clock speed in kHz (16 MHz = 16000 kHz).
const MCU_SPEED: u32 = 16000;
/// Timer prescaler value.
const MILLIS_PRESCALER: u32 = 1024;
/// Number of timer counts before triggering a compare match interrupt.
const TIMER_COUNTS: u32 = 125;

/// Millisecond increment per timer overflow.
///
/// With a 16 MHz CPU clock, 1024 prescaler, and 125 counts:
/// - Time per overflow = (125 * 1024) / 16000 = 8ms
const MILLIS_INCREMENT: u32 = TIMER_COUNTS * MILLIS_PRESCALER / MCU_SPEED;

/// Global millisecond counter.
static MILLIS_COUNTER: Mutex<Cell<u32>> = Mutex::new(Cell::new(0));

/// Timer0 Compare Match A interrupt service routine.
///
/// This ISR runs in the background once `millis_init()` is called and global
/// interrupts are enabled`.
#[interrupt(atmega328p)]
fn TIMER2_COMPA() {
    interrupt::free(|cs| {
        let counter_cell = MILLIS_COUNTER.borrow(cs);
        let counter = counter_cell.get();
        counter_cell.set(counter + MILLIS_INCREMENT);
    })
}

/// Returns the number of milliseconds elapsed since the board was powered on or reset.
///
/// This function provides a non-blocking way to track time, similar to Arduino's `millis()`.
/// The counter will overflow (wrap back to 0) after approximately 49.7 days.
///
/// # Returns
/// The elapsed time in milliseconds as a `u32`.
pub fn millis() -> u32 {
    interrupt::free(|cs| MILLIS_COUNTER.borrow(cs).get())
}

/// Initializes Timer/Counter0 for millisecond counting.
///
/// Configures TC0 in CTC (Clear Timer on Compare Match) mode with the appropriate
/// prescaler and compare value to generate interrupts every 8ms. The millisecond
/// counter is reset to 0.
///
/// This function must be called exactly once before using `millis()`, and global
/// interrupts must be enabled afterward with `unsafe { avr_device::interrupt::enable() }`.
///
/// # Arguments
/// * `tc` - Mutable reference to the Timer/Counter0 peripheral.
///
/// # Configuration
/// - **Mode:** CTC (Clear Timer on Compare Match)
/// - **Prescaler:** 1024
/// - **Compare Value:** 125
/// - **Interrupt Frequency:** ~125 Hz (every 8ms)
pub fn millis_init(tc: &mut TC2) {
    // Configure timer to work on mode Clear Timer on Count match (CTC)
    tc.tccr2a.write(|w| w.wgm2().ctc());
    // Configure timer prescaler
    tc.tccr2b.write(|w| match MILLIS_PRESCALER {
        8 => w.cs2().prescale_8(),
        64 => w.cs2().prescale_64(),
        256 => w.cs2().prescale_256(),
        1024 => w.cs2().prescale_1024(),
        _ => panic!(),
    });
    // Configure timer max count value
    tc.ocr2a.write(|w| w.bits(TIMER_COUNTS as u8));
    // Enable timer interrupt
    tc.timsk2.write(|w| w.ocie2a().set_bit());

    interrupt::free(|cs| {
        MILLIS_COUNTER.borrow(cs).set(0);
    });
}
