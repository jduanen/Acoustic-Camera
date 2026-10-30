# Board-top build script -- non-project-mode Vivado batch flow for
# rtl/single_fpga_top.v against xdc/single_fpga_top.xdc (the real AC7200 pin
# constraints), through to a bitstream. Unlike the two spike scripts, this is
# the design that actually goes on the board: currently clocks, POR, the
# status LED, DATA sampling, the CIC/FIR/packetizer pipeline and the RGMII
# MAC (see single_fpga_top.v's header comment).
#
# Part is the AC7200's own XC7A200T-2FBG484I (-2 speed grade; the spikes use
# -1 as a conservative utilization/timing target).
#
# Usage: make build-top   (from the repo root; runs ~/bin/vivado.sh, which sets
#        up Vivado's environment -- see Makefile), or directly:
#        vivado -mode batch -source build_top.tcl
#
# Network addresses: set any of BOARD_MAC, BOARD_IP, HOST_MAC, HOST_IP in the
# environment (make passes them through) to override single_fpga_top.v's
# defaults, e.g.
#        make build-top BOARD_IP=10.0.0.2 HOST_IP=10.0.0.1 HOST_MAC=a4:5e:60:12:34:56
# MACs as six hex octets separated by ':' or '-', IPs as dotted decimal. Unset
# or empty leaves the default; a malformed value stops the build before
# synthesis.
# (run from anywhere; all paths below are relative to this script's own
# location via [file dirname [info script]], not the invoking shell's cwd)

set script_dir [file dirname [info script]]
set repo_root  [file normalize "$script_dir/../../.."]
set part       xc7a200tfbg484-2
set outdir     "$script_dir/build_top"

file mkdir $outdir

set tp "$repo_root/fpga/single_fpga/rtl/third_party/verilog-ethernet"
read_verilog [list \
    $tp/eth_mac_1g_rgmii.v \
    $tp/eth_mac_1g.v \
    $tp/rgmii_phy_if.v \
    $tp/axis_gmii_rx.v \
    $tp/axis_gmii_tx.v \
    $tp/ssio_ddr_in.v \
    $tp/oddr.v \
    $tp/iddr.v \
    $tp/lfsr.v \
    $repo_root/fpga/multi_fpga/cluster/rtl/cic_decimator.v \
    $repo_root/fpga/multi_fpga/cluster/rtl/fir_compensator.v \
    $repo_root/fpga/single_fpga/rtl/gbe_packetizer.v \
    $repo_root/fpga/single_fpga/rtl/single_fpga_pipeline_top.v \
    $repo_root/fpga/single_fpga/rtl/status_led.v \
    $repo_root/fpga/single_fpga/rtl/mdio_init.v \
    $repo_root/fpga/single_fpga/rtl/arp_responder.v \
    $repo_root/fpga/single_fpga/rtl/pdm_sampler.v \
    $repo_root/fpga/single_fpga/rtl/single_fpga_top.v \
]
read_xdc $repo_root/fpga/single_fpga/xdc/single_fpga_top.xdc

# ---- network address overrides (see Usage) ----
proc mac_generic {name s} {
    set o [split [string trim $s] ":-"]
    if {[llength $o] != 6} { error "$name: '$s' is not a MAC address (six hex octets)" }
    set hex ""
    foreach b $o {
        if {![regexp {^[0-9A-Fa-f]{1,2}$} $b]} { error "$name: '$s' is not a MAC address" }
        append hex [format %02x [scan $b %x]]
    }
    return "$name=48'h$hex"
}
proc ip_generic {name s} {
    set o [split [string trim $s] "."]
    if {[llength $o] != 4} { error "$name: '$s' is not a dotted-decimal IPv4 address" }
    set hex ""
    foreach b $o {
        if {![regexp {^[0-9]{1,3}$} $b] || [scan $b %d] > 255} { error "$name: '$s' is not an IPv4 address" }
        append hex [format %02x [scan $b %d]]
    }
    return "$name=32'h$hex"
}
set generics {}
foreach {name kind} {BOARD_MAC mac BOARD_IP ip HOST_MAC mac HOST_IP ip} {
    if {[info exists ::env($name)] && [string trim $::env($name)] ne ""} {
        lappend generics -generic [${kind}_generic $name $::env($name)]
    }
}
puts "Network address overrides: [expr {[llength $generics] ? [join [lmap {f g} $generics {set g}] {, }] : {none (defaults)}}]"

synth_design -top single_fpga_top -part $part {*}$generics
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
