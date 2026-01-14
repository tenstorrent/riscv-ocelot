# Ocelot Simulation Tutorial with Chipyard using a pre-built image

This tutorial will guide you through setting up and running simulations of Ocelot in the Chipyard environment using a pre-built Docker/Podman image.

## Prerequisites

- Podman or Docker installed on your system
- Access to GitHub Container Registry (ghcr.io)
- X11 forwarding capability (for GUI tools like Verdi)

## Step 1: Download the Image

### Install Podman

Install Podman on your machine (Docker can be used as an alternative if Podman is not available):

```bash
# The following command might be different based on your OS
apt -y install podman
```

### Login to GitHub Container Registry

Login to `ghcr.io` using your GitHub credentials (use tokens, not passwords):

```bash
podman login ghcr.io
```

### Pull the Image

Pull the Chipyard simulation image:

```bash
podman pull ghcr.io/kychentt/tt-chipyard-whisper:latest
```

## Step 2: Clone and Setup Chipyard

Before mounting the image, you need to clone and set up the Chipyard repository:

### Clone the Tenstorrent Fork of Chipyard

```bash
git clone git@github.com:tenstorrent/chipyard.git
```

### Initialize Submodules

```bash
cd chipyard
./scripts/init-submodules-no-riscv-tools.sh -f
```

**Note:** After initializing submodules, you should see the `boom` repository under the `generators` directory. Both Chipyard and Boom should be on the HEAD of the bobtail/main branch.




## Step 3: Mount the Image with VCS/Verdi

**Note:** This step assumes you have VCS and Verdi installed and configured on your host machine. The container will use the tools from your host system.

**Important:** Run this command from inside the chipyard folder.

```bash
podman run --security-opt seccomp=unconfined \
  --security-opt label=disable \
  --env VCS_HOME="$VCS_HOME" \
  --env LM_LICENSE_FILE="$LM_LICENSE_FILE" \
  --env SNPSLMD_LICENSE_FILE="$SNPSLMD_LICENSE_FILE" \
  --env VERDI_HOME=${VERDI_HOME} \
  --env PATH=${PATH}:${VCS_HOME}/bin:${VERDI_HOME}/bin \
  -it --rm \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
  -v $HOME/.ssh:/root/.ssh \
  -v $(pwd):/root/my-chipyard \
  -v $VCS_HOME:$VCS_HOME \
  -v $VERDI_HOME:$VERDI_HOME \
  -w /root/my-chipyard \
  ghcr.io/kychentt/tt-chipyard-whisper:latest bash
```

### Required Environment Variables

Before running the container, ensure these variables are set:

- `$VCS_HOME`: Directory of VCS tool
- `$VERDI_HOME`: Directory of Verdi tool
- `$LM_LICENSE_FILE`: Login Machine License
- `$SNPSLMD_LICENSE_FILE`: Synopsys License

## Step 4: Verify the Setup (Optional)

### Build a Simple RocketCore with Verilator

This step verifies that all the original Chipyard toolchains are in place:

```bash
cd sims/verilator
make CONFIG=RocketConfig
```

### Build a Simple Bobcat Core with VCS

This step verifies that all the add-on toolchains are in place:

```bash
cd sims/vcs
make CONFIG=SmallBobcatConfig debug
```


## Step 5: Run Simulation and View Waveforms

### Run Simulation

Run the simulation from the chipyard root directory. You can try different tests under the `tests/rvv` directory:

```bash
./run_simple.sh tests/rvv/bringup_tests/ms4p6_vle32_8.elf
```

### Open Waveform in Verdi

Open the waveform viewer. **Remember to change the `*.fsdb` file path when you change tests:**

```bash
verdi -dbdir sims/vcs/simv-chipyard-SmallBobcatConfig-debug.daidir/ \
      -ssf sims/vcs/output/chipyard.TestHarness.SmallBobcatConfig/ms4p6_vle32_8.fsdb
```

### DUT Path to Inspect

In Verdi, navigate to the following DUT (Device Under Test) path to examine the Ocelot core:

```
TestDriver.testHarness.chiptop.system.tile_prci_domain.tile_reset_domain_boom_tile.core
```


## Step 6: Create Your Own Test (Optional)

You can compile custom RISC-V tests using the toolchain. Here's an example:

```bash
riscv64-unknown-elf-gcc \
  -march=rv64imafdcv \
  -mabi=lp64d \
  -Wl,--no-relax \
  -nostdlib \
  -T tests/rvv/bringup_tests/bobcat_linker.ld \
  tests/rvv/bringup_tests/ms1_vmv_vi.S \
  -o tests/rvv/bringup_tests/ms1_vmv_vi.elf \
  -static
```


## Step 7: Exit the Image

When you're done, exit the container:

```bash
exit
```

