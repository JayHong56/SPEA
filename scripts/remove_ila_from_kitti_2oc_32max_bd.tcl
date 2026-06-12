set project_dir {E:/verilog/board/vivado/kitti_2oc_32max}
set board_ip_dir {E:/verilog/board/IP}
set local_ip_repo [file join $project_dir ip_repo]
set xpr [file join $project_dir project_1.xpr]

open_project $xpr
set_property ip_repo_paths [list $local_ip_repo $board_ip_dir E:/verilog/pillarnest] [current_project]
update_ip_catalog -rebuild

set bd_file [file join $project_dir project_1.srcs sources_1 bd design_1 design_1.bd]
open_bd_design $bd_file

set ila_cells [get_bd_cells -quiet -filter {VLNV =~ "xilinx.com:ip:ila:*" || VLNV =~ "xilinx.com:ip:system_ila:*"}]
puts "ILA cells before delete: $ila_cells"
if {[llength $ila_cells] > 0} {
    delete_bd_objs $ila_cells
    save_bd_design
}

set remaining_ila [get_bd_cells -quiet -filter {VLNV =~ "xilinx.com:ip:ila:*" || VLNV =~ "xilinx.com:ip:system_ila:*"}]
puts "ILA cells after delete: $remaining_ila"
if {[llength $remaining_ila] > 0} {
    error "Failed to remove all ILA cells: $remaining_ila"
}

validate_bd_design
close_project
