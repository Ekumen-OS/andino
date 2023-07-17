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

// PC4 (pin A4), RIGHT ENCODER PIN A
#define RIGHT_ENCODER_A_GPIO_PIN PC4

// PC5 (pin A5), RIGHT ENCODER PIN B
#define RIGHT_ENCODER_B_GPIO_PIN PC5

// PD2 (pin 2), LEFT ENCODER PIN A
#define LEFT_ENCODER_A_GPIO_PIN PD2

// PD3 (pin 3), LEFT ENCODER PIN B
#define LEFT_ENCODER_B_GPIO_PIN PD3

// PD5 (pin 5), RIGHT MOTOR DRIVER BACKWARD PIN
#define RIGHT_MOTOR_BACKWARD_GPIO_PIN 5

// PD6 (pin 6), LEFT MOTOR DRIVER BACKWARD PIN
#define LEFT_MOTOR_BACKWARD_GPIO_PIN 6

// PB1 (pin 9), RIGHT MOTOR DRIVER FORWARD PIN
#define RIGHT_MOTOR_FORWARD_GPIO_PIN 9

// PB2 (pin 10), LEFT MOTOR DRIVER FORWARD PIN
#define LEFT_MOTOR_FORWARD_GPIO_PIN 10

// PB4 (pin 12), RIGHT MOTOR DRIVER ENABLE PIN
#define RIGHT_MOTOR_ENABLE_GPIO_PIN 12

// PB5 (pin 13), LEFT MOTOR DRIVER ENABLE PIN
#define LEFT_MOTOR_ENABLE_GPIO_PIN 13

// Note: In order to save two pins, the motor driver enable pins could be
// directly jumped to 5V in case your L298N motor driver board has a jumper to
// do so.
