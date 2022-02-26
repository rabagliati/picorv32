
read_verilog system.sv
read_verilog ../../picorv32.v
read_verilog ../../picosoc/simpleuart.v
read_xdc synth_nexys4.xdc

synth_design -part xc7a100t -top system
opt_design
place_design
route_design

report_utilization
report_timing

# write_verilog -force synth_system.v
write_bitstream -force synth_nexys4.bit
# write_mem_info -force synth_nexys4.mmi
write_cfgmem -force -format mcs -interface spix4 -size 128 -loadbit {up 0x0 "synth_nexys4.bit"} -file synth_nexys4.mcs


