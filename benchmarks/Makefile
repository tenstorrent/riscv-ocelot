#=======================================================================
# UCB VLSI FLOW: Makefile for riscv-bmarks
#-----------------------------------------------------------------------
# Yunsup Lee (yunsup@cs.berkeley.edu)
#

XLEN ?= 64

default: all

src_dir = .

instname = riscv-bmarks
instbasedir = $(UCB_VLSI_HOME)/install

#--------------------------------------------------------------------
# Sources
#--------------------------------------------------------------------

bmarks = \
	vec-conditional \
	vec-conjugate-gradient \
	vec-conv-3 \
	vec-cos \
	vec-div-approx \
	vec-dotprod \
	vec-dropout \
	vec-exp \
	vec-fconv2d \
	vec-fconv3d \
	vec-fdotprod \
	vec-fft \
	vec-iconv2d \
	vec-igemm \
	mat-igemm \
	mat-opacc \
	vec-jacobi2d \
	vec-log \
	vec-mixed_width_mask \
	vec-pathfinder \
	vec-roi-align \
	vec-sep-conv-3 \
	vec-sgemm \
	vec-sgemm-v2 \
	vec-sgemm-v3 \
	vec-sgemv \
	vec-slide-conv \
	vec-softmax \
	vec-strlen \
	vec-spmv \
	vec-square-root-approx \
	vec-transpose-load \
	vec-transpose-store \

cpp_bmarks = \
	vec-tasks \
	vec-daxpy

#--------------------------------------------------------------------
# Build rules
#--------------------------------------------------------------------

RISCV_PREFIX ?= riscv$(XLEN)-unknown-elf-
RISCV_GCC ?= $(RISCV_PREFIX)gcc
RISCV_GXX ?= $(RISCV_PREFIX)g++
RISCV_COMMON_OPTS ?= -DPREALLOCATE=1 -mcmodel=medany -static -O2 -g -ffast-math -fno-common -fno-builtin-printf -fno-tree-loop-distribute-patterns -march=rv$(XLEN)gcv_zfh_zicsr -mabi=lp64d
RISCV_GCC_OPTS ?= $(RISCV_COMMON_OPTS) -std=gnu99
RISCV_GXX_OPTS ?= $(RISCV_COMMON_OPTS) -std=c++17 -specs=htif_nano.specs
RISCV_LINK ?= $(RISCV_GCC) -T $(src_dir)/common/test.ld $(incs)
RISCV_LINK_OPTS ?= -static -nostdlib -nostartfiles -lm -lgcc -T $(src_dir)/common/test.ld
RISCV_OBJDUMP ?= $(RISCV_PREFIX)objdump -C -D -S --disassemble-all --disassemble-zeroes --section=.text --section=.text.startup --section=.text.init --section=.data
RISCV_SIM ?= spike --isa=rv$(XLEN)gcv_zfh_zicsr -p4 -m0x70020000:0x20000,0x80000000:0x10000000

incs  += -I$(src_dir)/env -I$(src_dir)/common $(addprefix -I$(src_dir)/, $(bmarks))
objs  :=

COMMON_SRCS = $(wildcard $(src_dir)/common/*.c) $(wildcard $(src_dir)/common/*.S)

define compile_template
$(1).riscv: $(wildcard $(src_dir)/$(1)/*) $(wildcard $(src_dir)/common/*)
	$$(RISCV_GCC) $$(incs) $$(RISCV_GCC_OPTS) -o $$@ $(wildcard $(src_dir)/$(1)/*.c) $(wildcard $(src_dir)/$(1)/*.S) $(COMMON_SRCS) $$(RISCV_LINK_OPTS)
endef

define compile_cpp_template
$(1).riscv: $(wildcard $(src_dir)/$(1)/*) $(src_dir)/utasks/utasks.h
	$$(RISCV_GXX) $$(incs) $$(RISCV_GXX_OPTS) -I$(src_dir)/utasks -o $$@ $(wildcard $(src_dir)/$(1)/*.cc) $(wildcard $(src_dir)/$(1)/*.S)
endef


$(foreach bmark,$(bmarks),$(eval $(call compile_template,$(bmark))))
$(foreach bmark,$(cpp_bmarks),$(eval $(call compile_cpp_template,$(bmark))))

#------------------------------------------------------------
# Build and run benchmarks on riscv simulator

bmarks_riscv_bin  = $(addsuffix .riscv,  $(bmarks) $(cpp_bmarks))
bmarks_riscv_dump = $(addsuffix .riscv.dump, $(bmarks) $(cpp_bmarks))
bmarks_riscv_out  = $(addsuffix .riscv.out,  $(bmarks) $(cpp_bmarks))

$(bmarks_riscv_dump): %.riscv.dump: %.riscv
	$(RISCV_OBJDUMP) $< > $@

$(bmarks_riscv_out): %.riscv.out: %.riscv
	$(RISCV_SIM) $< > $@

riscv: $(bmarks_riscv_dump)
run: $(bmarks_riscv_out)

junk += $(bmarks_riscv_bin) $(bmarks_riscv_dump) $(bmarks_riscv_hex) $(bmarks_riscv_out)

#------------------------------------------------------------
# Default

all: riscv

#------------------------------------------------------------
# Install

date_suffix = $(shell date +%Y-%m-%d_%H-%M)
install_dir = $(instbasedir)/$(instname)-$(date_suffix)
latest_install = $(shell ls -1 -d $(instbasedir)/$(instname)* | tail -n 1)

install:
	mkdir $(install_dir)
	cp -r $(bmarks_riscv_bin) $(bmarks_riscv_dump) $(install_dir)

install-link:
	rm -rf $(instbasedir)/$(instname)
	ln -s $(latest_install) $(instbasedir)/$(instname)

#------------------------------------------------------------
# Clean up

clean:
	rm -rf $(objs) $(junk)
