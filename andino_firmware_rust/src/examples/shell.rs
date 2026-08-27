/*!
* Shell
*
* Created a simple command-line interface that allows user interaction
* with the Arduino over the serial port. It registers a 'power' command that squares
* a number, and has error handling for all command failures.
*
* Commands:
* - p NUMBER: Squares the provided number and displays the result
*
* Example usage:
* $ p 5
* 5 squared is 25
*
* Commands with invalid parameters or format will display appropriate error messages and panic halt.
*/

#![no_std]
#![no_main]

use andino_firmware_rust::components::shell::Shell;
use andino_firmware_rust::components::{Command, CommandArgs, CommandError, CommandErrorResult};
use andino_firmware_rust::util::serial::SerialStream;
use arduino_hal::delay_ms;
use panic_halt as _;
use ufmt::uwriteln;

// Shell commands
fn power_number(args: CommandArgs, serial_stream: &mut SerialStream) -> Result<(), CommandError> {
    let number: i32 = match args[0].parse() {
        Ok(n) => n,
        Err(_) => return Err(CommandError::InvalidParameter),
    };
    let power = number * number;

    uwriteln!(serial_stream, "{} squared is {}", number, power).ok();
    Ok(())
}

#[arduino_hal::entry]
fn main() -> ! {
    let dp = arduino_hal::Peripherals::take().unwrap();
    let pins = arduino_hal::pins!(dp);
    let serial = arduino_hal::default_serial!(dp, pins, 9600);
    let mut serial_stream = SerialStream::new(serial);

    // Welcome message
    uwriteln!(
        &mut serial_stream,
        "Use the p NUMBER command to calculate the square power."
    )
    .ok();

    let mut shell = Shell::new();

    // Register commands - check for errors
    shell
        .register_command("p", 1)
        .or_log_and_panic(&mut serial_stream);

    // Enable global interruptions
    unsafe { avr_device::interrupt::enable() };

    loop {
        // Process input and handle any errors
        if let Some(Command { name, args }) = shell.process_input(&mut serial_stream) {
            if name.as_str() == "p" {
                power_number(args, &mut serial_stream).or_log_error(&mut serial_stream);
            }
        }

        delay_ms(100); // Reduced delay for better responsiveness
    }
}
