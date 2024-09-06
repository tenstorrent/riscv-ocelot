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

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "util.h"
#include "imatmul.h"

// Define Matrix dimensions:
// C = AB with A=[MxK], B=[KxN], C=[MxN]
extern uint64_t M;
extern uint64_t N;
extern uint64_t K;

extern int64_t at[] __attribute__((aligned(256)));
extern int64_t b[] __attribute__((aligned(256)));
extern int64_t c[] __attribute__((aligned(256)));
// Gold results
extern int64_t g[] __attribute__((aligned(256)));

// Verify the matrix
int verify_matrix(int64_t *result, int64_t *gold, size_t R, size_t C) {
  for (uint64_t i = 0; i < R; ++i) {
    for (uint64_t j = 0; j < C; ++j) {
      uint64_t idx = i * C + j;
      if (result[idx] != gold[idx]) {
        return (i + j) == 0 ? -1 : idx;
      }
    }
  }
  return 0;
}

int main() {
  // printf("IMATMUL\n");
  unsigned long cycles1, cycles2, instr2, instr1;

  // Matrices are initialized --> Start calculating
  printf("Calculating imatmul...\n");

  instr1 = read_csr(minstret);
  cycles1 = read_csr(mcycle);
  imatmul_4x4_m1(c, at, b, M, K, N);
  asm volatile("fence");
  instr2 = read_csr(minstret);
  cycles2 = read_csr(mcycle);

  // Metrics
  int64_t runtime = cycles2 - cycles1;
  float performance = 2.0 * M * K * N / runtime;

  printf("The execution took %d cycles.\n", runtime);
  printf("The performance is %ld OPs/1000 cycles.\n", (uint64_t)(1000.0 * performance));

  // Verify the result only for M == M (to keep it simple)
  if (M == M) {
    // Verify the result
    printf("Verifying result...\n");
    int error = verify_matrix(c, g, M, N);
    if (error != 0) {
      printf("Error code %d\n", error);
      if (error != -1)
        printf("c[%d]=%x =/= %x\n", error, c[error], g[error]);
      else
      printf("c[%d]=%x =/= %x\n", 0, c[0], g[0]);
      return error;
    } else {
      printf("Passed.\n");
    }
  }


  return 0;
}
