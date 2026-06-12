set project_dir {E:/verilog/board/vivado/kitti_2oc_32max}
set dcp_file [file join $project_dir project_1.runs impl_1 design_1_wrapper_routed.dcp]
set out_dir {E:/verilog/pillarnest/analysis/dsp_check_reports}

file mkdir $out_dir
open_checkpoint $dcp_file

report_utilization -hierarchical -file [file join $out_dir report_utilization_hierarchical.rpt]
catch {
    report_dsp_utilization -file [file join $out_dir report_dsp_utilization.rpt]
} dsp_report_result

set fh [open [file join $out_dir dsp_cell_name_scan.txt] w]
puts $fh "DCP: $dcp_file"
puts $fh "report_dsp_utilization result: $dsp_report_result"
puts $fh ""

set dsp_cells [get_cells -hier -quiet -filter {REF_NAME =~ DSP*}]
puts $fh "DSP-like cells by REF_NAME =~ DSP*: [llength $dsp_cells]"
foreach c [lsort $dsp_cells] {
    puts $fh [format "%s | REF_NAME=%s | PRIMITIVE_TYPE=%s" \
        $c \
        [get_property REF_NAME $c] \
        [get_property PRIMITIVE_TYPE $c]]
}

puts $fh ""
puts $fh "Pattern scans:"
set patterns {
    {*pfn*st2_mult*}
    {*pfn*dequant*}
    {*pfe*fxp_mul_x*}
    {*pfe*fxp_mul_y*}
    {*pfe*fxp_mul_z*}
    {*u_pfn_layer*}
    {*u_pfe*}
}

foreach pat $patterns {
    set cells [get_cells -hier -quiet $pat]
    puts $fh ""
    puts $fh "PATTERN $pat => [llength $cells] cells"
    set shown 0
    foreach c [lsort $cells] {
        if {$shown >= 250} {
            puts $fh "... truncated ..."
            break
        }
        puts $fh [format "%s | REF_NAME=%s | PRIMITIVE_TYPE=%s" \
            $c \
            [get_property REF_NAME $c] \
            [get_property PRIMITIVE_TYPE $c]]
        incr shown
    }
}

close $fh
close_design
