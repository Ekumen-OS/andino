//! Shell interface for the Andino robot.

use heapless::index_map::FnvIndexMap;
use heapless::String;

use crate::util::serial::SerialStream;
use ufmt;

use super::{
    Command, CommandArgs, CommandError, CommandNameString, MAX_NUMBER_OF_COMMANDS,
    MAX_NUMBER_OF_COMMAND_PARAMS,
};

/// Type alias for the map storing registered commands.
type RegisteredCmdsMap = FnvIndexMap<CommandNameString, usize, MAX_NUMBER_OF_COMMANDS>;

/// A serial input parser that checks for valid commands to execute.
///
/// # Example
///
/// ```rust
/// // Initialize serial communication
/// let serial = arduino_hal::default_serial!(dp, pins, 9600);
/// let mut serial_stream = SerialStream::new(serial);
///
/// // Define command callbacks
/// fn move_cmd(params: CommandArgs) -> Result<(), CommandError> {
///     // Parse direction and speed from params
///     // Execute motor control logic
///     Ok(())
/// }
///
/// // Create a shell instance with a default command handler
/// let mut shell = Shell::new();
///
/// // Register commands
/// shell.register_command("m", 2).or_log_and_panic(&mut serial_stream);
///
/// // Main loop
/// loop {
///     // Process incoming commands
///     if let Some(Command { name, args }) = shell.process_input(&mut serial_stream) {
///         if name.as_str() == "m" {
///             move_cmd(args).or_log_error(&mut serial_stream);
///         }
///     }
/// }
/// ```
pub struct Shell {
    known_cmds: RegisteredCmdsMap,
}

impl Shell {
    /// Creates a new `Shell` with empty registered commands.
    ///
    /// # Returns
    /// A new `Shell` instance
    pub fn new() -> Self {
        Self {
            known_cmds: RegisteredCmdsMap::new(),
        }
    }

    /// Register a command for validation during execution
    ///
    /// # Arguments
    /// * `name` -  Name of the command
    /// * `num_params` - Expected number of arguments
    ///
    pub fn register_command(&mut self, name: &str, num_params: usize) -> Result<(), CommandError> {
        if num_params > MAX_NUMBER_OF_COMMAND_PARAMS {
            return Err(CommandError::InvalidParameterCount);
        }
        if self.known_cmds.len() == MAX_NUMBER_OF_COMMANDS {
            return Err(CommandError::MaxNumberOfCommandsExceeded);
        }

        let name: CommandNameString = match String::try_from(name) {
            Ok(name) => name,
            Err(_) => {
                return Err(CommandError::InvalidCmdName);
            }
        };
        self.known_cmds.insert(name, num_params).unwrap();
        Ok(())
    }

    /// Reads for available input commands.
    ///
    /// The method first checks for available data on the serial stream.
    /// If found, it validates the input matches a valid command.
    ///
    /// # Arguments
    /// * `serial_stream` - Reference to serial stream
    ///
    /// # Returns
    /// * `Command` if valid command to execute.
    pub fn process_input(&mut self, serial_stream: &mut SerialStream) -> Option<Command> {
        if !serial_stream.available() {
            return None;
        }
        let input = serial_stream.read()?;

        // Split input searching for a valid values
        const COMMAND_PLUS_NUM_PARAMS: usize = MAX_NUMBER_OF_COMMAND_PARAMS + 1;
        let cmd_params: heapless::Vec<&str, COMMAND_PLUS_NUM_PARAMS> =
            input.trim().split(' ').collect();

        let cmd_name: CommandNameString = match String::try_from(cmd_params[0]) {
            Ok(name) => name,
            Err(_) => {
                ufmt::uwriteln!(
                    serial_stream,
                    "ERROR: {}",
                    CommandError::InvalidCmdName.message()
                )
                .ok();
                return None;
            }
        };

        // Extract parameters
        let param_strs = &cmd_params[1..];
        let mut params = CommandArgs::new();
        for param_str in param_strs {
            match String::try_from(*param_str) {
                Ok(param) => {
                    params.push(param).ok();
                }
                Err(_) => {
                    ufmt::uwriteln!(
                        serial_stream,
                        "ERROR: {}",
                        CommandError::InvalidParameter.message()
                    )
                    .ok();
                    return None;
                }
            }
        }

        if let Err(err) = self.validate_command(&cmd_name, &params) {
            ufmt::uwriteln!(serial_stream, "ERROR: {}", err.message()).ok();
            return None;
        }

        Some(Command::new(cmd_name, params))
    }

    /// Check if the name and params match a registered command.
    ///
    /// # Arguments
    /// * `name` - Provided name from serial input
    /// * `params` - Provided params from serial input
    ///
    /// # Returns
    /// Ok if valid command. False otherwise.
    fn validate_command(
        &mut self,
        name: &CommandNameString,
        params: &CommandArgs,
    ) -> Result<(), CommandError> {
        match self.known_cmds.get(name) {
            Some(&expected_num_params) => {
                if params.len() != expected_num_params {
                    return Err(CommandError::InvalidParameterCount);
                }
                Ok(())
            }
            None => Err(CommandError::CommandNotRegistered),
        }
    }
}

impl Default for Shell {
    fn default() -> Self {
        Self::new()
    }
}
