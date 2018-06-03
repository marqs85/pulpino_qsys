# TCL File Generated by Component Editor 17.1
# Sat May 12 20:01:07 EEST 2018
# DO NOT MODIFY


# 
# pulpino "pulpino" v1.0
#  2018.05.12.20:01:07
# Pulpino 
# 

# 
# request TCL package from ACDS 16.1
# 
package require -exact qsys 16.1


# 
# module pulpino
# 
set_module_property DESCRIPTION "Pulpino "
set_module_property NAME pulpino
set_module_property VERSION 1.0
set_module_property INTERNAL false
set_module_property OPAQUE_ADDRESS_MAP true
set_module_property GROUP "Processors and Peripherals/Embedded Processors"
set_module_property AUTHOR ""
set_module_property DISPLAY_NAME pulpino
set_module_property INSTANTIATE_IN_SYSTEM_MODULE true
set_module_property EDITABLE true
set_module_property REPORT_TO_TALKBACK false
set_module_property ALLOW_GREYBOX_GENERATION false
set_module_property REPORT_HIERARCHY false


# 
# file sets
# 
add_fileset QUARTUS_SYNTH QUARTUS_SYNTH "" ""
set_fileset_property QUARTUS_SYNTH TOP_LEVEL new_component
set_fileset_property QUARTUS_SYNTH ENABLE_RELATIVE_INCLUDE_PATHS false
set_fileset_property QUARTUS_SYNTH ENABLE_FILE_OVERWRITE_MODE false
add_fileset_file cluster_clock_gating.sv SYSTEM_VERILOG PATH pulpino/rtl/components/cluster_clock_gating.sv
add_fileset_file cluster_clock_inverter.sv SYSTEM_VERILOG PATH pulpino/rtl/components/cluster_clock_inverter.sv
add_fileset_file cluster_clock_mux2.sv SYSTEM_VERILOG PATH pulpino/rtl/components/cluster_clock_mux2.sv
add_fileset_file adbg_axi_biu.sv SYSTEM_VERILOG PATH adv_dbg_if/rtl/adbg_axi_biu.sv
add_fileset_file adbg_axi_module.sv SYSTEM_VERILOG PATH adv_dbg_if/rtl/adbg_axi_module.sv
add_fileset_file adbg_crc32.v VERILOG PATH adv_dbg_if/rtl/adbg_crc32.v
add_fileset_file adbg_lint_biu.sv SYSTEM_VERILOG PATH adv_dbg_if/rtl/adbg_lint_biu.sv
add_fileset_file adbg_lint_module.sv SYSTEM_VERILOG PATH adv_dbg_if/rtl/adbg_lint_module.sv
add_fileset_file adbg_lintonly_top.sv SYSTEM_VERILOG PATH adv_dbg_if/rtl/adbg_lintonly_top.sv
add_fileset_file adbg_or1k_biu.sv SYSTEM_VERILOG PATH adv_dbg_if/rtl/adbg_or1k_biu.sv
add_fileset_file adbg_or1k_module.sv SYSTEM_VERILOG PATH adv_dbg_if/rtl/adbg_or1k_module.sv
add_fileset_file adbg_or1k_status_reg.sv SYSTEM_VERILOG PATH adv_dbg_if/rtl/adbg_or1k_status_reg.sv
add_fileset_file adbg_tap_top.v VERILOG PATH adv_dbg_if/rtl/adbg_tap_top.v
add_fileset_file adbg_top.sv SYSTEM_VERILOG PATH adv_dbg_if/rtl/adbg_top.sv
add_fileset_file adbg_wb_biu.v VERILOG PATH adv_dbg_if/rtl/adbg_wb_biu.v
add_fileset_file adbg_wb_module.v VERILOG PATH adv_dbg_if/rtl/adbg_wb_module.v
add_fileset_file adv_dbg_if.sv SYSTEM_VERILOG PATH adv_dbg_if/rtl/adv_dbg_if.sv
add_fileset_file bytefifo.v VERILOG PATH adv_dbg_if/rtl/bytefifo.v
add_fileset_file syncflop.v VERILOG PATH adv_dbg_if/rtl/syncflop.v
add_fileset_file syncreg.v VERILOG PATH adv_dbg_if/rtl/syncreg.v
add_fileset_file zeroriscy_defines.sv SYSTEM_VERILOG PATH zero-riscy/include/zeroriscy_defines.sv
add_fileset_file zeroriscy_alu.sv SYSTEM_VERILOG PATH zero-riscy/zeroriscy_alu.sv
add_fileset_file zeroriscy_compressed_decoder.sv SYSTEM_VERILOG PATH zero-riscy/zeroriscy_compressed_decoder.sv
add_fileset_file zeroriscy_controller.sv SYSTEM_VERILOG PATH zero-riscy/zeroriscy_controller.sv
add_fileset_file zeroriscy_core.sv SYSTEM_VERILOG PATH zero-riscy/zeroriscy_core.sv
add_fileset_file zeroriscy_cs_registers.sv SYSTEM_VERILOG PATH zero-riscy/zeroriscy_cs_registers.sv
add_fileset_file zeroriscy_debug_unit.sv SYSTEM_VERILOG PATH zero-riscy/zeroriscy_debug_unit.sv
add_fileset_file zeroriscy_decoder.sv SYSTEM_VERILOG PATH zero-riscy/zeroriscy_decoder.sv
add_fileset_file zeroriscy_ex_block.sv SYSTEM_VERILOG PATH zero-riscy/zeroriscy_ex_block.sv
add_fileset_file zeroriscy_fetch_fifo.sv SYSTEM_VERILOG PATH zero-riscy/zeroriscy_fetch_fifo.sv
add_fileset_file zeroriscy_id_stage.sv SYSTEM_VERILOG PATH zero-riscy/zeroriscy_id_stage.sv
add_fileset_file zeroriscy_if_stage.sv SYSTEM_VERILOG PATH zero-riscy/zeroriscy_if_stage.sv
add_fileset_file zeroriscy_int_controller.sv SYSTEM_VERILOG PATH zero-riscy/zeroriscy_int_controller.sv
add_fileset_file zeroriscy_load_store_unit.sv SYSTEM_VERILOG PATH zero-riscy/zeroriscy_load_store_unit.sv
add_fileset_file zeroriscy_multdiv_fast.sv SYSTEM_VERILOG PATH zero-riscy/zeroriscy_multdiv_fast.sv
add_fileset_file zeroriscy_multdiv_slow.sv SYSTEM_VERILOG PATH zero-riscy/zeroriscy_multdiv_slow.sv
add_fileset_file zeroriscy_prefetch_buffer.sv SYSTEM_VERILOG PATH zero-riscy/zeroriscy_prefetch_buffer.sv
add_fileset_file zeroriscy_register_file_ff.sv SYSTEM_VERILOG PATH zero-riscy/zeroriscy_register_file_ff.sv
add_fileset_file core_top.sv SYSTEM_VERILOG PATH core_top.sv TOP_LEVEL_FILE


# 
# parameters
# 
add_parameter AXI_ADDR_WIDTH integer 32 "AXI address width"
add_parameter AXI_DATA_WIDTH integer 32 "AXI data width"
add_parameter AXI_ID_MASTER_WIDTH integer 10 "AXI ID master width"
add_parameter AXI_ID_SLAVE_WIDTH integer 10 "AXI ID slave width"
add_parameter AXI_USER_WIDTH integer 0 "AXI ID user width"
add_parameter USE_ZERO_RISCY integer 1 "Use Zero-RISCY"
add_parameter RISCY_RV32F integer 0 "RISCY_RV32F"
add_parameter ZERO_RV32M integer 1 "ZERO_RV32M"
add_parameter ZERO_RV32E integer 0 "ZERO_RV32E"


# 
# display items
# 


# 
# connection point reset_sink
# 
add_interface reset_sink reset end
set_interface_property reset_sink associatedClock clk_sink
set_interface_property reset_sink synchronousEdges DEASSERT
set_interface_property reset_sink ENABLED true
set_interface_property reset_sink EXPORT_OF ""
set_interface_property reset_sink PORT_NAME_MAP ""
set_interface_property reset_sink CMSIS_SVD_VARIABLES ""
set_interface_property reset_sink SVD_ADDRESS_GROUP ""

add_interface_port reset_sink reset_sink_reset reset_n Input 1


# 
# connection point config
# 
add_interface config conduit end
set_interface_property config associatedClock clk_sink
set_interface_property config associatedReset ""
set_interface_property config ENABLED true
set_interface_property config EXPORT_OF ""
set_interface_property config PORT_NAME_MAP ""
set_interface_property config CMSIS_SVD_VARIABLES ""
set_interface_property config SVD_ADDRESS_GROUP ""

add_interface_port config testmode_i testmode_i Input 1
add_interface_port config fetch_enable_i fetch_enable_i Input 1
add_interface_port config clock_gating_i clock_gating_i Input 1
add_interface_port config boot_addr_i boot_addr_i Input 32
add_interface_port config core_busy_o core_busy_o Input 1


# 
# connection point clk_sink
# 
add_interface clk_sink clock end
set_interface_property clk_sink clockRate 25000000
set_interface_property clk_sink ENABLED true
set_interface_property clk_sink EXPORT_OF ""
set_interface_property clk_sink PORT_NAME_MAP ""
set_interface_property clk_sink CMSIS_SVD_VARIABLES ""
set_interface_property clk_sink SVD_ADDRESS_GROUP ""

add_interface_port clk_sink clk_clk clk Input 1


# 
# connection point interrupt_receiver
# 
add_interface interrupt_receiver interrupt start
set_interface_property interrupt_receiver associatedAddressablePoint ""
set_interface_property interrupt_receiver associatedClock clk_sink
set_interface_property interrupt_receiver associatedReset reset_sink
set_interface_property interrupt_receiver irqScheme INDIVIDUAL_REQUESTS
set_interface_property interrupt_receiver ENABLED true
set_interface_property interrupt_receiver EXPORT_OF ""
set_interface_property interrupt_receiver PORT_NAME_MAP ""
set_interface_property interrupt_receiver CMSIS_SVD_VARIABLES ""
set_interface_property interrupt_receiver SVD_ADDRESS_GROUP ""

add_interface_port interrupt_receiver irq_i irq Input 32


# 
# connection point avalon_master_debug
# 
add_interface avalon_master_debug avalon start
set_interface_property avalon_master_debug addressUnits SYMBOLS
set_interface_property avalon_master_debug associatedClock clk_sink
set_interface_property avalon_master_debug associatedReset reset_sink
set_interface_property avalon_master_debug bitsPerSymbol 8
set_interface_property avalon_master_debug burstOnBurstBoundariesOnly false
set_interface_property avalon_master_debug burstcountUnits WORDS
set_interface_property avalon_master_debug doStreamReads false
set_interface_property avalon_master_debug doStreamWrites false
set_interface_property avalon_master_debug holdTime 0
set_interface_property avalon_master_debug linewrapBursts false
set_interface_property avalon_master_debug maximumPendingReadTransactions 0
set_interface_property avalon_master_debug maximumPendingWriteTransactions 0
set_interface_property avalon_master_debug readLatency 0
set_interface_property avalon_master_debug readWaitTime 1
set_interface_property avalon_master_debug setupTime 0
set_interface_property avalon_master_debug timingUnits Cycles
set_interface_property avalon_master_debug writeWaitTime 0
set_interface_property avalon_master_debug ENABLED true
set_interface_property avalon_master_debug EXPORT_OF ""
set_interface_property avalon_master_debug PORT_NAME_MAP ""
set_interface_property avalon_master_debug CMSIS_SVD_VARIABLES ""
set_interface_property avalon_master_debug SVD_ADDRESS_GROUP ""

add_interface_port avalon_master_debug wb_adr_o address Output 32
add_interface_port avalon_master_debug wb_dat_o writedata Output 32
add_interface_port avalon_master_debug wb_dat_i readdata Input 32
add_interface_port avalon_master_debug wb_stb_o chipselect Output 1
add_interface_port avalon_master_debug wb_sel_o byteenable Output 4
add_interface_port avalon_master_debug wb_we_o write Output 1
add_interface_port avalon_master_debug wb_ack_i waitrequest Input 1
set_interface_assignment avalon_master_debug wb_err_i 0


# 
# connection point jtag
# 
add_interface jtag conduit end
set_interface_property jtag associatedClock clk_sink
set_interface_property jtag associatedReset ""
set_interface_property jtag ENABLED true
set_interface_property jtag EXPORT_OF ""
set_interface_property jtag PORT_NAME_MAP ""
set_interface_property jtag CMSIS_SVD_VARIABLES ""
set_interface_property jtag SVD_ADDRESS_GROUP ""

add_interface_port jtag tck_i tck_i Input 1
add_interface_port jtag trstn_i trstn_i Input 1
add_interface_port jtag tms_i tms_i Input 1
add_interface_port jtag tdi_i tdi_i Input 1
add_interface_port jtag tdo_o tdo_o Output 1

