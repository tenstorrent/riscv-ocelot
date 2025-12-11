# Vector Extension

## Overview
The added Risc-V Vector (RVV) Unit is based on ratified [Vector Extension 1.0 specification](https://github.com/riscv/riscv-v-spec/releases/tag/v1.0)

![](docs/figures/proj_bobtail/Bobtail2_architecture.svg)

The original RVV (RISC-V Vector) unit operates as an in-order pipeline and lacks support for register renaming. To facilitate integration with the out-of-order Boom core, we have adopted the [Open Vector Interface](https://github.com/semidynamics/OpenVectorInterface) with some extensions, complemented by several microarchitectural (uArch) units that ensure functionality and performance optimization:

- **In-order Vector Issue Queue**: Functioning similarly to other issue queues, it monitors write-back signals from INT/FP/MEM units, activating vector instructions with dependencies on INT/FP registers.

- **Request Queue**: This queue manages vector instructions that have resolved their INT/FP register dependencies. It employs a Point of No Return (PNR) mechanism and supports speculative execution where instructions complete but results are discarded if speculation fails.

- **Detached Vector Load/Store Unit**: In alignment with the OVI specifications, the core must generate vector load/store requests. Leveraging this requirement, we have created a fully detached unit to enhance performance, enabling unit-stride and strided loads to be executed out-of-order.

- **Augmented Load/Store Unit**: Modifications to the existing Load/Store Unit (LSU) have been made to handle dependencies between scalar and vector Load/Store requests effectively.

## OVI Extensions
- **Segment Load/Store Operations**: The OVI specification does not explicitly define behavior for segment load/store operations. This implementation extends OVI by supporting segment operations with a conservative packing strategy - segments are packed within fields but not across element boundaries.

## Limitations & Known Issues
1. The integer unit only supports rounding mode of vxrm==0 (round-to-nearest-up)
2. No vector exception support - vector instructions do not generate precise exceptions

## Enabling Vector Unit
Vector Unit can be enabled by adding the following line in config-mixins.

For example,
```
class MediumBobcatConfig extends Config(
  new boom.common.WithVector(2) ++                               // MediumBoom with 2 issues
  new boom.common.WithBoomDebugHarness ++                        // Enable debug harness
  new WithCustomBootROM ++                                       // Use custom BootROM to enable COSIM
  new boom.common.WithNMediumBooms(1) ++                         // 1 MediumBoom core
  new chipyard.config.AbstractConfig)
```

## Testing
A few vector test binaries are included in the [Chipyard](https://github.com/tenstorrent/chipyard/tree/bobcat/tests/rvv) repro.

Here's example commands to compile and run a vector test with FSDB generated:
```
export CHIPYARD=<path_to_chipyard>
make -C sims/vcs run-binary-debug-hex CONFIG=SmallBobcatConfig BINARY=$CHIPYARD/tests/rvv/isg/riscv_vector_ms5_smoke_test.elf SIM_FLAGS="+cosim"
```
## Slides
[Bobcat Final Presentation](docs/Bobcat_Final_Presentation.pdf)

[Bobtail Final Presentation](docs/Bobtail-Final-Presentation.pdf)

## Micro-Architecture
### Vector Pipeline
![Vector Pipeline](docs/figures/rvv/Bobcat_Design_VPU.png)
### Vector Floating-Point Unit
![Vector Floating-Point Unit](docs/figures/rvv/VFP_Unit_Overview.png)
### Vector Floating-Point Encoder
![Vector Floating-Point Encoder](docs/figures/rvv/VFP_Encoder_256b.png)
### Vector Floating-Point Encoder Lane
![Vector Floating-Point Encoder Lane](docs/figures/rvv/VFP_Encoder_Lane.png)
### Vector Floating-Point Lane
![Vector Floating-Point Lane](docs/figures/rvv/VFP_Lane.png)
### Vector Floating-Point Execution Unit
![Vector Floating-Point Execution Unit](docs/figures/rvv/VFP_EX_Unit.png)
### Vector Floating-Point Scalar Unit
![Vector Floating-Point Scalar Unit](docs/figures/rvv/VFP_Scalar_Unit.png)
