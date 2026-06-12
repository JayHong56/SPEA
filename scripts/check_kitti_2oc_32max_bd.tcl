set project_dir {E:/verilog/board/vivado/kitti_2oc_32max}
set board_ip_dir {E:/verilog/board/IP}
set local_ip_repo [file join $project_dir ip_repo]
set xpr [file join $project_dir project_1.xpr]
set voxelize_32max_vlnv {xilinx.com:user:voxelize_wrapper_kitti_32max_v1_1:1.0}

open_project $xpr
set_property ip_repo_paths [list $local_ip_repo $board_ip_dir E:/verilog/pillarnest] [current_project]
update_ip_catalog -rebuild

if {[llength [get_ipdefs -quiet $voxelize_32max_vlnv]] == 0} {
    error "32max voxelize IP not found in catalog: $voxelize_32max_vlnv"
}

set bd_file [file join $project_dir project_1.srcs sources_1 bd design_1 design_1.bd]
open_bd_design $bd_file
set voxel_cell [get_bd_cells -quiet voxelize_wrapper_kit_1]
if {[llength $voxel_cell] == 0} {
    error "BD cell voxelize_wrapper_kit_1 not found"
}
set cell_vlnv [get_property VLNV $voxel_cell]
puts "voxelize_wrapper_kit_1 VLNV = $cell_vlnv"
if {$cell_vlnv ne $voxelize_32max_vlnv} {
    error "voxelize_wrapper_kit_1 is bound to $cell_vlnv, expected $voxelize_32max_vlnv"
}
validate_bd_design
close_project
