#-----------------------------------------------------------
# Vivado v2019.2 (64-bit)
# SW Build 2708876 on Wed Nov  6 21:39:14 MST 2019
# IP Build 2700528 on Thu Nov  7 00:09:20 MST 2019
# Start of session at: Fri May 29 00:29:50 2020
# Process ID: 9298
# Current directory: /home/andyr
# Command line: vivado
# Log file: /home/andyr/vivado.log
# Journal file: /home/andyr/vivado.jou
#-----------------------------------------------------------
# open_project /home/andyr/git/vivado/src/jforth/jforth.xpr

open_hw_manager
connect_hw_server -allow_non_jtag

open_hw_target
set_property PROBES.FILE {} [get_hw_devices xc7a100t_0]
set_property FULL_PROBES.FILE {} [get_hw_devices xc7a100t_0]
set_property PROGRAM.FILE {/home/andyr/git/picorv32/scripts/vivado/synth_nexys4.bit} [get_hw_devices xc7a100t_0]
program_hw_devices [get_hw_devices xc7a100t_0]

refresh_hw_device [lindex [get_hw_devices xc7a100t_0] 0]
create_hw_cfgmem -hw_device [get_hw_devices xc7a100t_0] -mem_dev [lindex [get_cfgmem_parts {s25fl128sxxxxxx0-spi-x1_x2_x4}] 0]

