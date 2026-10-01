# ====================================================================
#  Ternary FDTD on the Tang Nano 20K
#  yosys -> nextpnr-himbaechel -> gowin_pack -> openFPGALoader
#
#  Board strings carried over from the uart_echo bring-up:
#    DEVICE = GW2AR-LV18QN88C8/I7
#    FAMILY = GW2A-18C           (NOT GW2AR-18C; this one silently
#                                 breaks the build if wrong)
#    BOARD  = tangnano20k
#
#  Usage:
#    make sim          encoder unit test + full-core differential test
#    make              synthesise, place, route, pack
#    make flash        program to SRAM (lost on power cycle)
#    make flash-persist
#    make capture      run the host capture script (needs Octave)
#    make convergence  the S-sweep, in the C model, on the host
#
#  The SINGLE SOURCE OF TRUTH for every physical parameter is the C
#  golden model.  It writes build/params.vh and build/sine_lut.hex,
#  which BOTH the simulation and the synthesis read.  Change a
#  parameter here and the reference, the test vectors, the source LUT
#  and the bitstream all move together.
# ====================================================================

DEVICE = GW2AR-LV18QN88C8/I7
FAMILY = GW2A-18C
BOARD  = tangnano20k
BUILD  = build

# ---- bisection targets ----------------------------------------------
#   make                 the solver          (top_fdtd_hw)
#   make TARGET=echo     UART echo only      (is the board/port alive?)
#   make TARGET=blast    frame sender only   (is the SENDER alive?)
TARGET ?= fdtd

ifeq ($(TARGET),echo)
  TOP     = top_echo
  TOPRTL  = rtl/top_echo.v rtl/uart_rx.v rtl/uart_tx.v
  NEEDGEN =
else ifeq ($(TARGET),blast)
  TOP     = top_blast
  TOPRTL  = rtl/top_blast.v rtl/uart_tx.v
  NEEDGEN =
else
  TOP     = top_fdtd_hw
  TOPRTL  = rtl/top_fdtd_hw.v rtl/top_fdtd.v rtl/fdtd_ternary.v \
            rtl/tern_encode.v rtl/uart_rx.v rtl/uart_tx.v
  NEEDGEN = hw-params
endif

CC     ?= gcc
OCTAVE ?= octave
MC     ?=
PORT   ?= /dev/serial/by-id/usb-SIPEED_USB_Debugger_2025030317-if01-port0

# The default goal is stated EXPLICITLY.  Make otherwise takes the first
# target in the file, which silently changes meaning whenever a target is
# added above another one.
.DEFAULT_GOAL := all

# ---- convergence sweep on hardware ----------------------------------
# Rebuilds, reflashes and captures at each Courant number.  The host
# reads build/params.vh, so it always analyses with the S that was
# actually built -- the two cannot drift apart.
.PHONY: sweep
sweep:
	@for mc in 8 7 6 5 4; do \
	   echo "=========== S = 1/$$(( 1 << (13 - $$mc) ))  (MC_LOG=$$mc) ==========="; \
	   $(MAKE) HW_MC_LOG=$$mc all  || exit 1; \
	   $(MAKE) HW_MC_LOG=$$mc flash || exit 1; \
	   grep FDTD_MC_LOG $(BUILD)/params.vh; \
	   md5sum $(BUILD)/$(TOP).fs; \
	   sleep 1; \
	   $(OCTAVE) host/fdtd_capture.m || exit 1; \
	   mv fpga_state.txt fpga_state_mc$$mc.txt; \
	   cp $(BUILD)/params.vh     fpga_params_mc$$mc.vh; \
	   cp $(BUILD)/golden_hw.txt fpga_golden_mc$$mc.txt; \
	 done
	@echo
	@echo "captures: fpga_state_mc*.txt   parameters: fpga_params_mc*.vh"
	@echo "bitstream checksums above MUST all differ; identical ones mean"
	@echo "the build did not change and the sweep is worthless."
	@md5sum fpga_state_mc*.txt

# ---- physical parameters -------------------------------------------
# HARDWARE build.  S = 2^(MC_LOG-M_LOG) = 2^-8 = 1/256, which the C
# model puts at about 2 % snapshot error against the float reference.
HW_NX     ?= 32
HW_MC_LOG ?= 5
HW_M_LOG  ?= 13
# The reference run must be AT LEAST as long as the capture, or the
# analysis truncates to the shorter of the two and the coarse-S points
# vanish entirely (their transient alone exceeds a short reference).
# This is also the sample count the capture script uses, read back from
# params.vh -- one number, one place.
HW_NSTEP  ?= 172032

# SIMULATION build.  Smaller grid and a coarser time step so the wave
# reaches the probe inside a tractable number of simulated cycles.
# The scheme is identical; only the resolution differs.
SIM_NX     ?= 16
SIM_MC_LOG ?= 8
SIM_M_LOG  ?= 13
SIM_NSTEP  ?= 600

WAVELEN ?= 16
SRC_K   ?= 3

GOLDFLAGS = -O2 -lm -DWAVELEN=$(WAVELEN) -DSRC_K=$(SRC_K)

RTL = $(TOPRTL)
CST = constr/tangnano20k_fdtd.cst
SDC = constr/tangnano20k.sdc

.PHONY: all sim sim-enc sim-core sim-frame flash flash-persist capture diag raw raw-diag spectrum convergence-figure convergence clean help

help:
	@echo "targets: sim  all  flash  flash-persist  capture  diag  spectrum  convergence  clean"

all: $(BUILD)/$(TOP).fs

$(BUILD):
	mkdir -p $(BUILD)

# --------------------------------------------------------------------
#  Golden model: reference stream, source LUT, Verilog parameters
# --------------------------------------------------------------------

# The binary name CARRIES THE PARAMETERS.  With a fixed name, make sees
# the binary as newer than fdtd_golden.c and skips the recompile when
# only a -D flag changes -- so params.vh and the sine LUT keep their old
# values and every bitstream in a sweep comes out identical.  That is
# not hypothetical: it silently produced five byte-identical captures.
GOLD_HW = $(BUILD)/gold_hw_$(HW_NX)_$(HW_M_LOG)_$(HW_MC_LOG)_$(HW_NSTEP)

$(GOLD_HW): golden/fdtd_golden.c | $(BUILD)
	$(CC) $(GOLDFLAGS) -DNX=$(HW_NX) -DNY=$(HW_NX) \
	      -DM_LOG=$(HW_M_LOG) -DMC_LOG=$(HW_MC_LOG) -DNSTEP=$(HW_NSTEP) \
	      -o $@ $< -lm

GOLD_SIM = $(BUILD)/gold_sim_$(SIM_NX)_$(SIM_M_LOG)_$(SIM_MC_LOG)_$(SIM_NSTEP)

$(GOLD_SIM): golden/fdtd_golden.c | $(BUILD)
	$(CC) $(GOLDFLAGS) -DNX=$(SIM_NX) -DNY=$(SIM_NX) \
	      -DM_LOG=$(SIM_M_LOG) -DMC_LOG=$(SIM_MC_LOG) -DNSTEP=$(SIM_NSTEP) \
	      -o $@ $< -lm

$(BUILD)/encvec: golden/enc_vectors.c | $(BUILD)
	$(CC) -O2 -DM_LOG=$(SIM_M_LOG) -o $@ $<

# Simulation parameter set
sim-params: $(GOLD_SIM)
	$(GOLD_SIM) $(BUILD)/golden.txt $(BUILD)/sine_lut.hex $(BUILD)/params.vh

# Hardware parameter set (overwrites the same three files)
hw-params: $(GOLD_HW)
	$(GOLD_HW) $(BUILD)/golden_hw.txt $(BUILD)/sine_lut.hex $(BUILD)/params.vh
	@grep -q "FDTD_MC_LOG *$(HW_MC_LOG)$$" $(BUILD)/params.vh || \
	  { echo "ERROR: params.vh does not carry MC_LOG=$(HW_MC_LOG)"; exit 1; }

# --------------------------------------------------------------------
#  Simulation
# --------------------------------------------------------------------

sim: sim-enc sim-core sim-frame

sim-enc: $(BUILD)/encvec | $(BUILD)
	$(BUILD)/encvec $(BUILD)/enc_vectors.txt
	iverilog -g2005 -o $(BUILD)/sim_enc.out sim/tb_tern_encode.v rtl/tern_encode.v
	vvp $(BUILD)/sim_enc.out

sim-core: sim-params
	iverilog -g2005 -I. -o $(BUILD)/sim_core.out \
	         sim/tb_fdtd_ternary.v rtl/fdtd_ternary.v rtl/tern_encode.v
	vvp $(BUILD)/sim_core.out

# Framing: decodes the serial line the way the host does and checks
# one frame per step with a contiguous step byte.  A duplicated or
# dropped frame is invisible in the captured data -- it decodes
# perfectly and only the sample rate is wrong -- so it has to be
# caught here.
sim-frame: sim-params
	iverilog -g2005 -I. -o $(BUILD)/sim_frame.out \
	         sim/tb_top_fdtd.v rtl/top_fdtd.v rtl/fdtd_ternary.v \
	         rtl/tern_encode.v rtl/uart_rx.v rtl/uart_tx.v
	vvp $(BUILD)/sim_frame.out

# --------------------------------------------------------------------
#  Synthesis.  -nodsp is not an optimisation choice here: it is the
#  claim under test.  If the design still builds and meets timing with
#  every DSP block forbidden, the datapath really is multiplier-free.
# --------------------------------------------------------------------

$(BUILD)/$(TOP).json: $(RTL) $(NEEDGEN)
	yosys -p "read_verilog -I. $(RTL); \
	          synth_gowin -top $(TOP) -nodsp -json $(BUILD)/$(TOP).json"

$(BUILD)/$(TOP).pnr.json: $(BUILD)/$(TOP).json $(CST) $(SDC)
	nextpnr-himbaechel \
		--device $(DEVICE) \
		--json $(BUILD)/$(TOP).json \
		--write $(BUILD)/$(TOP).pnr.json \
		--vopt family=$(FAMILY) \
		--vopt cst=$(CST) \
		--sdc $(SDC) \
		--report $(BUILD)/$(TOP).pnr.rpt

$(BUILD)/$(TOP).fs: $(BUILD)/$(TOP).pnr.json
	gowin_pack -d $(FAMILY) -o $(BUILD)/$(TOP).fs $(BUILD)/$(TOP).pnr.json

# SRAM: fast, and the device runs it immediately.  Use this while
# iterating.  Lost on power cycle or re-enumeration -- if the board
# goes quiet after a replug, this is why.
flash: $(BUILD)/$(TOP).fs
	openFPGALoader -b $(BOARD) $(BUILD)/$(TOP).fs

# Flash: survives a power cycle, but openFPGALoader ERASES SRAM to do
# it, so the device is left unconfigured and silent until it reboots
# from flash.  The --reset makes it do that; without it the board looks
# dead and the next capture hangs with no progress at all.
flash-persist: $(BUILD)/$(TOP).fs
	openFPGALoader -b $(BOARD) -f $(BUILD)/$(TOP).fs
	openFPGALoader -b $(BOARD) --reset || \
	  echo "  *** --reset unsupported: POWER-CYCLE the board before capturing ***"

# --------------------------------------------------------------------
#  Host side
# --------------------------------------------------------------------

# Capture from the board.  Writes fpga_state.txt.
capture:
	$(OCTAVE) host/fdtd_capture.m

# Diagnostic run: the board reports the SOURCE sample instead of the
# field, so the host can see whether the source ROM initialised in the
# bitstream.  Takes about 8 s.  Run this once after any change to the
# build, before trusting a field capture.
diag:
	FDTD_DIAG=1 $(OCTAVE) host/fdtd_capture.m

# Two-second raw byte dump.  No framing assumptions: answers "is the
# board transmitting at all" before any question about the protocol.
raw:
	python3 host/fdtd_raw.py $(PORT)

raw-diag:
	python3 host/fdtd_raw.py $(PORT) --diag

# Re-analyse fpga_state.txt with no board attached.  Run this as often
# as you like: it removes the turn-on transient, windows the record and
# identifies the peaks against the hard-wall cavity eigenfrequencies.
# make spectrum          analyse fpga_state.txt
# make spectrum MC=5     analyse fpga_state_mc5.txt from a sweep
# --persist keeps the figure windows open after the script ends;
# without it Octave exits and takes them with it.  The script also
# writes each figure to a PNG regardless.
spectrum:
	FDTD_MC=$(MC) $(OCTAVE) --persist host/fdtd_spectrum.m

# Convergence figure from the sweep captures.  Python + matplotlib
# rather than Octave: this one has to read five captures, five
# parameter files and five reference runs together, and the plotting is
# the part least worth reimplementing.
convergence-figure:
	python3 host/plot_convergence.py

# Convergence study in the golden model: fixed physical time, S halved
# five times.  This is the software half of the hardware claim; run the
# same points on the board by rebuilding with HW_MC_LOG set to each.
convergence: | $(BUILD)
	@rm -f $(BUILD)/convergence.txt
	@for mc in 8 7 6 5 4 3; do \
	   ns=$$(( 64 << ($(HW_M_LOG) - $$mc) )); \
	   $(CC) $(GOLDFLAGS) -DNX=$(HW_NX) -DNY=$(HW_NX) \
	         -DM_LOG=$(HW_M_LOG) -DMC_LOG=$$mc -DNSTEP=$$ns \
	         -o $(BUILD)/gconv golden/fdtd_golden.c -lm; \
	   $(BUILD)/gconv | tee -a $(BUILD)/convergence.txt | \
	     grep -E "^# grid|relative error"; \
	 done

clean:
	rm -rf $(BUILD)
