#!/usr/bin/env bash
# BSD 3-Clause License
#
# Copyright (c) 2026, Ekumen Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# Generates the code coverage report of the unit tests of the firmware, which have to be run first
# (the coverage data accumulates between runs, so clean the desktop build to start from scratch):
#
#   pio run -e desktop -t clean
#   pio test -e desktop
#   scripts/coverage.sh [output_directory]
#
# It prints the summary and writes, in the output directory (coverage by default), the HTML report
# (index.html), the Cobertura XML report (coverage.xml) and the markdown report (summary.md). Only
# the firmware sources are measured: the tests and the libraries they use are left out.

set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")/.."

readonly OUTPUT_DIRECTORY="${1:-coverage}"
mkdir -p "${OUTPUT_DIRECTORY}"

gcovr \
  --root . \
  --object-directory .pio/build/desktop \
  --filter 'src/' \
  --exclude-throw-branches \
  --txt \
  --html-details "${OUTPUT_DIRECTORY}/index.html" \
  --cobertura-pretty --cobertura "${OUTPUT_DIRECTORY}/coverage.xml" \
  --markdown "${OUTPUT_DIRECTORY}/summary.md"
