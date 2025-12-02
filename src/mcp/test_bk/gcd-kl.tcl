source helpers.tcl
set test_name gcd
read_lef ./Nangate45/Nangate45.lef
read_def ./design/nangate45/gcd/gcd.def

#mcp::KL_placement
#detailed_placement

#set def_file [make_result_file $test_name.def]
#write_def $def_file
#diff_file $def_file $test_name.defok
