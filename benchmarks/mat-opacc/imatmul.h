// Copyright 2020 ETH Zurich and University of Bologna.
//
// SPDX-License-Identifier: Apache-2.0
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//    http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Author: Miles Rusch
// adapted from
// Author: Matheus Cavalcante, ETH Zurich
//         Samuel Riedel, ETH Zurich

#ifndef IMATMUL_H
#define IMATMUL_H

#include <stdint.h>

void imatmul_4x4_m1(int64_t *c, const int64_t *at, const int64_t *b,
             const unsigned long int m, const unsigned long int k,
             const unsigned long int n);
void imatmul_8x4_m1(int64_t *c, const int64_t *at, const int64_t *b,
             const unsigned long int m, const unsigned long int k,
             const unsigned long int n);

void imatmul_4x4_m4(int64_t *c, const int64_t *at, const int64_t *b,
                 const unsigned long int m, const unsigned long int k,
                 const unsigned long int n);

extern int64_t event_trigger;

#endif
