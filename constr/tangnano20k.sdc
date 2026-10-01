# Timing constraints -- Tang Nano 20K.
#
# The 27 MHz oscillator is a 37.04 ns period.  Without this, nextpnr has
# no constraint to check against and defaults to 12 MHz, so its "PASS"
# line is meaningless.  nextpnr-himbaechel implements a SUBSET of SDC:
# comments are '#', not the '//' the Gowin .cst format uses, and
# -waveform is not accepted.
create_clock -name clk -period 37.04 [get_ports {clk}]
