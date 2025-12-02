source helpers.tcl
set test_name medium01
read_lef ./nangate45.lef
read_def ./$test_name.def

mcp::kernighan_lin_placement -compact
#global_placement
set def_file [make_result_file $test_name.def]
write_def $def_file
diff_file $def_file $test_name.defok
source report_hpwl.tcl
