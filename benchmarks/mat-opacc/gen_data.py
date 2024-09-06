#!/usr/bin/env python3
# Copyright 2022 ETH Zurich and University of Bologna.
#
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#    http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Author: Miles Rusch
# adapted from
# Author: Matteo Perotti

# C = AB with  =[MxK], B=[KxN], C=[MxN]
# arg1, arg2, arg3: M, K, N

import random as rand
import numpy as np
import sys

def emit(name, array, alignment='8'):
  print(".global %s" % name)
  print(".balign " + alignment)
  print("%s:" % name)
  bs = array.tobytes()
  for i in range(0, len(bs), 4):
    s = ""
    for n in range(4):
      s += "%02x" % bs[i+3-n]
    print("    .word 0x%s" % s)

############
## SCRIPT ##
############

if len(sys.argv) == 4:
  M = int(sys.argv[1])
  K = int(sys.argv[2])
  N = int(sys.argv[3])
else:
  print("Error. Give me three argument: M, K, N.")
  print("C = AB with A=[MxN], B=[NxP], C=[MxP]")
  sys.exit()

dtype = np.int64

# Matrices and results
A = np.arange(1, M*K+1).reshape((M, K)).astype(dtype)
B = np.arange(0, M*K).reshape((K, N)).astype(dtype)
# B = np.arange(1, K*N+1).reshape((K, N)).astype(dtype)
C = np.zeros([M, N], dtype=dtype)
# Golden result matrix with transposed A for mmu kernel
G = np.matmul(A, B).astype(dtype)

# Create the file
print(".section .data,\"aw\",@progbits")
emit("M", np.array(M, dtype=np.uint64))
emit("K", np.array(K, dtype=np.uint64))
emit("N", np.array(N, dtype=np.uint64))
emit("at", A.T, '256')
emit("b", B, '256')
emit("c", C, '256')
emit("g", G, '256') # str(G.dtype.itemsize * 8))

# print("A:", A.shape)
# for i in range(A.shape[1]-1, -1, -1):
#   print("%02x" % A[3][i], "%02x" % A[2][i], "%02x" % A[1][i], "%02x" % A[0][i])
# print("B:")
# for i in range(B.shape[0]-1, -1, -1):
#   print("%02x" % B[i][3], "%02x" % B[i][2], "%02x" % B[i][1], "%02x" % B[i][0])
# print("G:")
# for i in range(G.shape[0]-1, -1, -1):
#   print("%02x" % G[i][3], "%02x" % G[i][2], "%02x" % G[i][1], "%02x" % G[i][0])