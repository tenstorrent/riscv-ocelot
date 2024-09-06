// Copyright 2020 ETH Zurich and University of Bologna.
//
// SPDX-License-Identifier: Apache-2.0
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain at copy of the License at
//
//    http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Author: Miles Rusch
// adapted from:
// Author: Matheus Cavalcante, ETH Zurich
//         Samuel Riedel, ETH Zurich
#include <stdio.h>
#include "imatmul.h"
// #include "util.h"

// ---------------
// 4x4
// ---------------

void imatmul_4x4_m1(int64_t *c, const int64_t *at, const int64_t *b,
                 const unsigned long int M, const unsigned long int K,
                 const unsigned long int N) {
  
  unsigned long int block_size_p;

  // Set the vector configuration
  asm volatile("vsetvli %0, %1, e64, m1, ta, ma" : "=r"(block_size_p) : "r"(N));
  
  // begin perfomance counters
  // unsigned long cycles1, cycles2, instr2, instr1;
  // instr1 = read_csr(minstret);
  // cycles1 = read_csr(mcycle);
  
  // // Load 4 rows of A and B and C
  int64_t *c_temp = c;
  asm volatile("vle64.v v8, (%0);" ::"r"(c_temp));
  asm volatile("vle64.v v0, (%0);" ::"r"(b));
  asm volatile("vle64.v v4, (%0);" ::"r"(at));
  c_temp += N;
  b += N;
  at += K;
  asm volatile("vle64.v v9, (%0);" ::"r"(c_temp));
  asm volatile("vle64.v v1, (%0);" ::"r"(b));
  asm volatile("vle64.v v5, (%0);" ::"r"(at));
  c_temp += N;
  b += N;
  at += K;
  asm volatile("vle64.v v10, (%0);" ::"r"(c_temp));
  asm volatile("vle64.v v2, (%0);" ::"r"(b));
  asm volatile("vle64.v v6, (%0);" ::"r"(at));
  c_temp += N;
  b += N;
  at += K;
  asm volatile("vle64.v v11, (%0);" ::"r"(c_temp));
  asm volatile("vle64.v v3, (%0);" ::"r"(b));
  asm volatile("vle64.v v7, (%0);" ::"r"(at));
  // c_temp += N;
  // b += N;
  // at += K;

  ///////////////////////////////////////
  //vmv mrf <- vrf x 4 (LMUL=1)
  ///////////////////////////////////////

  // vmv.m.v m0, v8
  asm volatile(".4byte 0x0004000b");
  // vmv.m.v m0, v9
  asm volatile(".4byte 0x0004800b");
  // vmv.m.v m0, v10
  asm volatile(".4byte 0x0005000b");
  // vmv.m.v m0, v11
  asm volatile(".4byte 0x0005800b");


  ///////////////////////////////////////
  //opacc mrf <- vrf x K=4 (LMUL=1)
  ///////////////////////////////////////

  // opacc.v.m m0 v0 v4
  asm volatile(".4byte 0x0040200b");
  // opacc.v.m m0 v1 v5
  asm volatile(".4byte 0x0050a00b");
  // opacc.v.m m0 v2 v6
  asm volatile(".4byte 0x0061200b");
  // opacc.v.m m0 v3 v7
  asm volatile(".4byte 0x0071a00b");

  
  ///////////////////////////////////////
  //vmv vrf <- mrf x 4 (LMUL=1)
  ///////////////////////////////////////
  // mmv.v.m v8 m0
  asm volatile(".4byte 0x0000140b");
  // mmv.v.m v9 m1
  asm volatile(".4byte 0x0000948b");
  // mmv.v.m v10 m2
  asm volatile(".4byte 0x0001150b");
  // mmv.v.m v11 m3
  asm volatile(".4byte 0x0001958b");

  // store results
  c_temp = c;
  asm volatile("vse64.v v8, (%0);" ::"r"(c_temp));
  c_temp += N;
  asm volatile("vse64.v v9, (%0);" ::"r"(c_temp));
  c_temp += N;
  asm volatile("vse64.v v10, (%0);" ::"r"(c_temp));
  c_temp += N;
  asm volatile("vse64.v v11, (%0);" ::"r"(c_temp));

  // // end perfomance counters
  // instr2 = read_csr(minstret);
  // cycles2 = read_csr(mcycle);

  // // Metrics
  // int64_t runtime = cycles2 - cycles1;
  // float performance = 2.0 * M * M * M / runtime;

  // printf("The execution took %d cycles.\n", runtime);
  // printf("The performance is %ld OPs/1000 cycles.\n", (uint64_t)(1000.0 * performance));

}


// ---------------
// 4x8x4 LMUL=1
// --------------- 
void imatmul_8x4_m1(int64_t *c, const int64_t *at, const int64_t *b,
                    const unsigned long int M, const unsigned long int K, 
                    const unsigned long int N) {
  
   
  unsigned long int block_size_p;

  // Set the vector configuration
  asm volatile("vsetvli %0, %1, e64, m1, ta, ma" : "=r"(block_size_p) : "r"(N));
  
  // begin perfomance counters
  // unsigned long cycles1, cycles2, instr2, instr1;
  // instr1 = read_csr(minstret);
  // cycles1 = read_csr(mcycle);
  
  int64_t *c_ = c;
  asm volatile("vle64.v v16, (%0);" ::"r"(c_));
  c_ += N;
  asm volatile("vle64.v v17, (%0);" ::"r"(c_));
  c_ += N;
  asm volatile("vle64.v v18, (%0);" ::"r"(c_));
  c_ += N;
  asm volatile("vle64.v v19, (%0);" ::"r"(c_));
  
  ///////////////////////////////////////
  //vmv mrf <- vrf x K (LMUL=1)
  ///////////////////////////////////////
  // vmv.v.m m0 v16
  asm volatile(".4byte 0x0008000b");
  // vmv.v.m m1 v17
  asm volatile(".4byte 0x0008808b");
  // vmv.v.m m2 v18
  asm volatile(".4byte 0x0009010b");
  // vmv.v.m m3 v19
  asm volatile(".4byte 0x0009818b");
  
  ///////////////////////////////////////
  // do opacc x K (LMUL=1)
  ///////////////////////////////////////
  // Load 4 rows of A and B
  asm volatile("vle64.v v0, (%0);" ::"r"(at));
  asm volatile("vle64.v v8, (%0);" ::"r"(b));
  b += N;
  at += M;
  asm volatile("vle64.v v1, (%0);" ::"r"(at));
  asm volatile("vle64.v v9, (%0);" ::"r"(b));
  b += N;
  at += M;
  asm volatile("vle64.v v2, (%0);" ::"r"(at));
  asm volatile("vle64.v v10, (%0);" ::"r"(b));
  b += N;
  at += M;
  asm volatile("vle64.v v3, (%0);" ::"r"(at));
  asm volatile("vle64.v v11, (%0);" ::"r"(b));
  b += N;
  at += M;
  asm volatile("vle64.v v4, (%0);" ::"r"(at));
  asm volatile("vle64.v v12, (%0);" ::"r"(b));
  b += N;
  at += M;
  asm volatile("vle64.v v5, (%0);" ::"r"(at));
  asm volatile("vle64.v v13, (%0);" ::"r"(b));
  b += N;
  at += M;
  asm volatile("vle64.v v6, (%0);" ::"r"(at));
  asm volatile("vle64.v v14, (%0);" ::"r"(b));b += N;
  b += N;
  at += M;
  asm volatile("vle64.v v7, (%0);" ::"r"(at));
  asm volatile("vle64.v v15, (%0);" ::"r"(b));

  // opacc.v.m m0 v0 v8
  asm volatile(".4byte 0x0080200b");
  // opacc.v.m m0 v1 v9
  asm volatile(".4byte 0x0090a00b");
  // opacc.v.m m0 v2 v10
  asm volatile(".4byte 0x00a1200b");
  // opacc.v.m m0 v3 v11
  asm volatile(".4byte 0x00b1a00b");
  // opacc.v.m m0 v4 v12
  asm volatile(".4byte 0x00c2200b");
  // opacc.v.m m0 v5 v13
  asm volatile(".4byte 0x00d2a00b");
  // opacc.v.m m0 v6 v14
  asm volatile(".4byte 0x00e3200b");
  // opacc.v.m m0 v7 v15
  asm volatile(".4byte 0x00f3a00b");
    
  // vmv vrf <- mrf
  // mmv.v.m v16 m0
  asm volatile(".4byte 0x0000180b");
  // mmv.v.m v17 m1
  asm volatile(".4byte 0x0000988b");
  // mmv.v.m v18 m2
  asm volatile(".4byte 0x0001190b");
  // mmv.v.m v19 m3
  asm volatile(".4byte 0x0001998b");

  // store results
  c_ = c;
  asm volatile("vse64.v v16, (%0);" ::"r"(c_));
  c_ += N;
  asm volatile("vse64.v v17, (%0);" ::"r"(c_));
  c_ += N;
  asm volatile("vse64.v v18, (%0);" ::"r"(c_));
  c_ += N;
  asm volatile("vse64.v v19, (%0);" ::"r"(c_));

  // end perfomance counters
  // asm volatile("fence");
  // instr2 = read_csr(minstret);
  // cycles2 = read_csr(mcycle);

  // // Metrics
  // int64_t runtime = cycles2 - cycles1;
  // float performance = 2.0 * M * M * M / runtime;

  // printf("The execution took %d cycles.\n", runtime);
  // printf("The performance is %ld OPs/1000 cycles.\n", (uint64_t)(1000.0 * performance));
}

////////////
// LMUL = 4
////////////
void imatmul_4x4_m4(int64_t *c, const int64_t *at, const int64_t *b,
                 const unsigned long int M, const unsigned long int K,
                 const unsigned long int N) {
    
  int64_t *c_temp = c;
  unsigned long int block_size_p;

  // Set the vector configuration
  printf("N: %d\n", N);
  asm volatile("vsetvli %0, %1, e64, m4, ta, ma" : "=r"(block_size_p) : "r"(4*N));
  printf("m1 block_size_p: %d\n", block_size_p);
  
  // // Load 4 rows of A and B and C
  asm volatile("vle64.v v8, (%0);" ::"r"(c_temp));
  asm volatile("vle64.v v0, (%0);" ::"r"(b));
  asm volatile("vle64.v v4, (%0);" ::"r"(at));
  c_temp += N;
  b += N;
  at += K;

  // vmv.m.v m0, v8
  asm volatile(".4byte 0x0004000b");
  
  // do opacc 4x (for LMUL=1)
  // opacc.v.m m0 v0 v4
  asm volatile(".4byte 0x0040200b");
  
  // move from mrf to vrf
  // mmv.v.m v8 m0
  asm volatile(".4byte 0x0000140b");
  
  // store results
  c_temp = c;
  asm volatile("vse64.v v0, (%0);" ::"r"(c_temp));
  c_temp += N;
  asm volatile("vse64.v v1, (%0);" ::"r"(c_temp));
  c_temp += N;
  asm volatile("vse64.v v2, (%0);" ::"r"(c_temp));
  c_temp += N;
  asm volatile("vse64.v v3, (%0);" ::"r"(c_temp));
  c_temp += N;
  printf("* c = %d\n", *c);
}
