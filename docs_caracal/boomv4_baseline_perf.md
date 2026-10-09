### CYCLES CONSUMED

| TEST (BUILD) | BOOMv4 Small - SCALAR ONLY | BOOMv4 Medium - SCALAR ONLY | BOOMv4 Large - SCALAR ONLY | BOOMv4 Mega - SCALAR ONLY | Attempt 1 Medium - RVV1.0 | Attempt 1 Mega - RVV1.0 |
|---|---|---|---|---|---|---|
| **_axpy** | 9796 | 9336 | 8866 | 8436 | 10296 | 9796 |
| **arith_mean** | 27486 | 24376 | 23416 | 22996 | | |
| **axpy** | 50306 | 43276 | 31056 | 26336 | 57826 | 38576 |
| **conv1d** | 112256 | 75146 | 55396 | 53106 | 79376 | 74686 |
| **conv2d** | 385836 | 241526 | 162566 | 152726 | | |
| **inner_product** | 48416 | 36646 | 26706 | 26306 | | |
| **relu** | 32376 | 27186 | 25316 | 24396 | 26256 | 25806 |
| **sgemm** | 66206 | 46966 | 38956 | 36616 | | |
| **transpose** | 283796 | 223626 | 177746 | 167536 | | |

Note that Blank Rows where functional failures where the test failed to complete for Attempt 1 Caracal.