# Board-top build script -- non-project-mode Vivado batch flow for
# rtl/single_fpga_top.v against xdc/single_fpga_top.xdc (the real AC7200 pin
# constraints), through to a bitstream. Unlike the two spike scripts, this is
# the design that actually goes on the board: currently clocks, POR and the
# status LED only (see single_fpga_top.v's header comment).
#
# Part is the AC7200's own XC7A200T-2FBG484I (-2 speed grade; the spikes use
# -1 as a conservative utilization/timing target).
#
# Usage: make build-top   (from the repo root; runs ~/bin/vivado.sh, which sets
#        up Vivado's environment -- see Makefile), or directly:
#        vivado -mode batch -source build_top.tcl
# (run from anywhere; all paths below are relative to this script's own
# location via [file dirname [info script]], not the invoking shell's cwd)

set script_dir [file dirname [info script]]
set repo_root  [file normalize "$script_dir/../../.."]
set part       xc7a200tfbg484-2
set outdir     "$script_dir/build_top"

file mkdir $outdir

read_verilog [list \
    $repo_root/fpga/single_fpga/rtl/status_led.v \
    $repo_root/fpga/single_fpga/rtl/single_fpga_top.v \
]
read_xdc $repo_root/fpga/single_fpga/xdc/single_fpga_top.xdc

synth_design -top single_fpga_top -part $part
write_checkpoint -force $outdir/post_synth.dcp

opt_design
place_design
route_design
write_checkpoint -force $outdir/post_route.dcp
report_io -file $outdir/io_routed.rpt
report_utilization -file $outdir/utilization_routed.rpt
report_timing_summary -file $outdir/timing_summary_routed.rpt
report_methodology -file $outdir/methodology_routed.rpt
report_drc -file $outdir/drc_routed.rpt

write_bitstream -force $outdir/single_fpga_top.bit

puts "==== BUILD COMPLETE ===="
