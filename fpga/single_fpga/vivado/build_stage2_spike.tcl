# Stage 2 feasibility spike build script -- non-project-mode Vivado batch
# flow, no .xpr, no XDC (see single_fpga_top_stage2_spike.v's header comment
# and the project's plan file, "Single-FPGA hub (ALINX AC7200 module)"
# section, Stage 2). Goal is a real post-place-and-route LUT/register/DSP/
# control-set number for "pipeline + packetizer + vendored 1G RGMII MAC"
# together on xc7a200tfbg484-1, to compare against Stage 1's own 43.52%
# pipeline-only baseline -- nothing else.
#
# Usage: make build-stage2-spike   (from the repo root; runs ~/bin/vivado.sh, which sets
#        up Vivado's environment -- see Makefile), or directly:
#        vivado -mode batch -source build_stage2_spike.tcl
# (run from anywhere; all paths below are relative to this script's own
# location via [file dirname [info script]], not the invoking shell's cwd)

set script_dir [file dirname [info script]]
set repo_root  [file normalize "$script_dir/../../.."]
set part       xc7a200tfbg484-1
set outdir     "$script_dir/build_stage2"
set tp         "$repo_root/fpga/single_fpga/rtl/third_party/verilog-ethernet"

file mkdir $outdir

read_verilog -sv [list \
    $repo_root/fpga/multi_fpga/cluster/rtl/pdm_line_demux.v \
    $repo_root/fpga/multi_fpga/cluster/rtl/cic_decimator.v \
    $repo_root/fpga/multi_fpga/cluster/rtl/fir_compensator.v \
    $repo_root/fpga/single_fpga/rtl/gbe_packetizer.v \
    $repo_root/fpga/single_fpga/rtl/single_fpga_pipeline_top.v \
    $tp/eth_mac_1g_rgmii.v \
    $tp/eth_mac_1g.v \
    $tp/rgmii_phy_if.v \
    $tp/axis_gmii_rx.v \
    $tp/axis_gmii_tx.v \
    $tp/ssio_ddr_in.v \
    $tp/oddr.v \
    $tp/iddr.v \
    $tp/lfsr.v \
    $repo_root/fpga/single_fpga/rtl/single_fpga_top_stage2_spike.v \
]

synth_design -top single_fpga_top_stage2_spike -part $part
write_checkpoint -force $outdir/post_synth.dcp
report_utilization -file $outdir/utilization_synth.rpt

opt_design
report_utilization -file $outdir/utilization_opt.rpt

place_design
report_utilization -file $outdir/utilization_placed.rpt
report_utilization -hierarchical -file $outdir/utilization_placed_hier.rpt
report_control_sets -verbose -file $outdir/control_sets_placed.rpt

route_design
write_checkpoint -force $outdir/post_route.dcp
report_utilization -file $outdir/utilization_routed.rpt
report_timing_summary -file $outdir/timing_summary_routed.rpt
report_methodology -file $outdir/methodology_routed.rpt
report_drc -file $outdir/drc_routed.rpt

puts "==== BUILD COMPLETE ===="
