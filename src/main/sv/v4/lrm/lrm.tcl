clear -all
analyze -sv09 lrm.sv lrm_model.sv lrm_assertions.sv
set top lrm
elaborate -top $top -bbox_mul 256
clock clk
reset !reset_n
