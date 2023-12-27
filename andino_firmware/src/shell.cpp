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

void Shell::init(Stream& stream) { stream_ = &stream; }

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

// TODO(jballoffet): Modify parsing method to allow command name to be a string.
void Shell::process() {
  while (stream_->available() > 0) {
    // Read the next character
    char chr = stream_->read();

    // Terminate a command with a CR
    if (chr == 13) {
      if (args_count_ == 1) {
        arg1_[arg_index_] = 0;
      } else if (args_count_ == 2) {
        arg2_[arg_index_] = 0;
      }
      execute_callback();
      reset();
    }
    // Use spaces to delimit parts of the command
    else if (chr == ' ') {
      // Step through the arguments
      if (args_count_ == 0) {
        args_count_ = 1;
      } else if (args_count_ == 1) {
        arg1_[arg_index_] = 0;
        args_count_ = 2;
        arg_index_ = 0;
      }
      continue;
    } else {
      if (args_count_ == 0) {
        // The first arg is the single-letter command
        command_ = chr;
      } else if (args_count_ == 1) {
        // Subsequent arguments can be more than one character
        arg1_[arg_index_] = chr;
        arg_index_++;
      } else if (args_count_ == 2) {
        arg2_[arg_index_] = chr;
        arg_index_++;
      }
    }
  }
}

void Shell::reset() {
  command_ = 0;
  memset(arg1_, 0, sizeof(arg1_));
  memset(arg2_, 0, sizeof(arg2_));
  args_count_ = 0;
  arg_index_ = 0;
}

// TODO(jballoffet): Modify parsing method to allow command name to be a string.
void Shell::execute_callback() {
  for (size_t i = 0; i < commands_count_; i++) {
    if (command_ == commands_[i].name[0]) {
      commands_[i].callback(arg1_, arg2_);
      return;
    }
  }

  // Unknown command received, executing default callback.
  if (default_callback_ != nullptr) {
    default_callback_(arg1_, arg2_);
  }
}

}  // namespace andino
