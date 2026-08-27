//! Contains abstractions for all the Andino components.

use crate::util::serial::SerialStream;

pub mod encoder;
pub mod motor;
pub mod pid;
pub mod shell;

/// Command argument string size
pub const COMMAND_ARG_STRING_SIZE: usize = 8;
/// Command string size
pub const COMMAND_STRING_SIZE: usize = 4;

/// Maximum number of commands that can be registered with the shell.
const MAX_NUMBER_OF_COMMANDS: usize = 16;
/// Maximum number of arguments a command can have.
const MAX_NUMBER_OF_COMMAND_PARAMS: usize = 16;

/// Alias for String holding a command argument
pub type CommandArgString = heapless::String<COMMAND_ARG_STRING_SIZE>;
/// Alias for String holding a command name
pub type CommandNameString = heapless::String<COMMAND_STRING_SIZE>;

/// Alias for the vector storing command arguments
pub type CommandArgs = heapless::Vec<CommandArgString, MAX_NUMBER_OF_COMMAND_PARAMS>;

/// Represents command to execute
pub struct Command {
    pub name: CommandNameString,
    pub args: CommandArgs,
}

impl Command {
    /// Creates a new `Command` request.
    ///
    /// # Arguments
    /// * `cmd` - Name of the command to execute.
    /// * `args` - Arguments to the command to execute.
    ///
    /// # Returns
    /// A new `Command` instance
    pub fn new(cmd: CommandNameString, args: CommandArgs) -> Self {
        Self { name: cmd, args }
    }
}

/// Errors that can occur during command processing.
#[derive(Debug)]
pub enum CommandError {
    CommandNotRegistered,
    ExecutionFailed,
    InvalidParameterCount,
    InvalidParameter,
    InvalidCmdName,
    MaxNumberOfCommandsExceeded,
}

impl CommandError {
    /// Returns a descriptive error message for the specific error variant.
    ///
    /// # Returns
    /// A static string describing the error in human-readable format.
    pub fn message(&self) -> &'static str {
        match self {
            CommandError::CommandNotRegistered => "Command was not registered in the shell.",
            CommandError::InvalidParameterCount => "Invalid number of parameters.",
            CommandError::InvalidParameter => "Invalid parameter format.",
            CommandError::InvalidCmdName => {
                "Invalid command name. Check name matches expected size."
            }
            CommandError::ExecutionFailed => "Command execution failed.",
            CommandError::MaxNumberOfCommandsExceeded => "Maximum number of commands exceeded.",
        }
    }
}

// Expand Command Error Result functionality
pub trait CommandErrorResult {
    /// Consumes the result and error log in the serial stream.
    ///
    /// # Arguments
    /// * `serial_stream` - Reference to serial stream
    fn or_log_error(self, serial: &mut SerialStream);

    /// Consumes the result, error log in the serial stream and panic.
    ///
    /// # Arguments
    /// * `serial_stream` - Reference to serial stream
    fn or_log_and_panic(self, serial: &mut SerialStream);
}

impl CommandErrorResult for Result<(), CommandError> {
    fn or_log_error(self, serial: &mut SerialStream) {
        if let Err(err) = self {
            ufmt::uwriteln!(serial, "ERROR: {}", err.message()).ok();
        };
    }

    fn or_log_and_panic(self, serial: &mut SerialStream) {
        if let Err(err) = self {
            ufmt::uwriteln!(serial, "ERROR: {}", err.message()).ok();
            panic!()
        };
    }
}
