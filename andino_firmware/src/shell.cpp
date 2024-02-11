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
#include "shell.h"

#include <string.h>

namespace andino {

void Shell::set_serial_stream(const SerialStream* serial_stream) { serial_stream_ = serial_stream; }

void Shell::set_default_callback(CommandCallback callback) { default_callback_ = callback; }

void Shell::register_command(const char* name, CommandCallback callback) {
  if (commands_count_ >= kCommandsMax) {
    return;
  }

  Command command;
  strcpy(command.name, name);
  command.callback = callback;
  commands_[commands_count_++] = command;
}

void Shell::process_input() {
  while (serial_stream_->available() > 0) {
    const char input = serial_stream_->read();

    switch (input) {
      case '\r':
        // Terminate command prompt message and parse it.
        message_buffer_[message_index_++] = '\0';
        parse_message();
        // Reset message buffer.
        message_index_ = 0;
        break;

      case '\n':
        // Ignore newline characters.
        break;

      default:
        message_buffer_[message_index_++] = input;
        // Prevent buffer overflow.
        if (message_index_ >= kCommandPromptLengthMax) {
          message_index_ = 0;
        }
        break;
    }
  }
}

void Shell::parse_message() {
  char* argv[kCommandArgMax];
  int argc = 0;

  argv[argc] = strtok(message_buffer_, " ");
  while (argv[argc] != NULL && argc < (kCommandArgMax - 1)) {
    argv[++argc] = strtok(NULL, " ");
  }

  execute_callback(argc, argv);
}

void Shell::execute_callback(int argc, char** argv) {
  for (size_t i = 0; i < commands_count_; i++) {
    if (!strcmp(argv[0], commands_[i].name)) {
      commands_[i].callback(argc, argv);
      return;
    }
  }

  // Unknown command received, executing default callback.
  if (default_callback_ != nullptr) {
    default_callback_(argc, argv);
  }
}

}  // namespace andino
