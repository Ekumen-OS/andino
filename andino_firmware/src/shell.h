// BSD 3-Clause License
//
// Copyright (c) 2023, Ekumen Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice, this
//    list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#pragma once

#include <stddef.h>

#include "serial_stream.h"

namespace andino {

/// @brief This class interprets and processes the commands entered by the user.
class Shell {
 public:
  /// @brief Command callback type.
  typedef void (*CommandCallback)(int argc, char** argv);

  /// @brief Sets the serial stream to use.
  ///
  /// @param serial_stream Serial stream.
  void set_serial_stream(const SerialStream* serial_stream);

  /// @brief Sets the default callback for unknown commands.
  ///
  /// @param callback Callback function.
  void set_default_callback(CommandCallback callback);

  /// @brief Adds a command registry entry to the shell.
  ///
  /// @param name Command name.
  /// @param callback Callback function.
  void register_command(const char* name, CommandCallback callback);

  /// @brief Processes the available input at the command prompt (if any). Meant to be called
  /// continously.
  void process_input();

 private:
  /// Maximum command name length.
  static constexpr int kCommandNameLengthMax{8};

  /// Command registry entry definition.
  struct Command {
    /// Command name.
    char name[kCommandNameLengthMax];
    /// Callback function.
    CommandCallback callback;
  };

  /// Maximum number of commands that can be registered.
  static constexpr int kCommandsMax{16};

  /// Maximum number of command arguments that can be processed.
  static constexpr int kCommandArgMax{16};

  /// Maximum command prompt message length.
  static constexpr int kCommandPromptLengthMax{64};

  /// Parses the command prompt message.
  void parse_message();

  /// Executes the corresponding command callback function.
  void execute_callback(int argc, char** argv);

  /// Serial stream.
  const SerialStream* serial_stream_{nullptr};

  /// Default callback for unknown commands.
  CommandCallback default_callback_{nullptr};

  /// Command registry.
  Command commands_[kCommandsMax];

  /// Number of registered commands.
  size_t commands_count_{0};

  /// Command prompt message.
  char message_buffer_[kCommandPromptLengthMax];

  /// Command prompt message index.
  int message_index_{0};
};

}  // namespace andino
