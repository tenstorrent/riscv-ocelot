// See LICENSE.TT for license details.

module tt_compare #(parameter 
                    DISABLE_ASSERTIONS=0,
                    VALUE_WIDTH=32,                     
                    ENTRIES=4
)
(
input                                i_clk                      , // This is needed for the assertion
input                                i_reset_n                  , // This is needed for the assertion
input                                i_compare_enable           , // This can be used to qualify the output (either for functionality or as a data-gating signal for power)
input         [VALUE_WIDTH-1:0]      i_compare_value            , 
input         [VALUE_WIDTH-1:0]      i_compare_value_mask       , 
input                                i_compare_valid_mask       , 
input         [VALUE_WIDTH-1:0]      i_entry_values[ENTRIES-1:0],
input         [ENTRIES-1:0]          i_entry_valids             ,
output  logic [ENTRIES-1:0]          o_compare_match 
);

integer  e;

`ifdef SIM
   generate
     // If these assertions are "don't care" for this instance of the module, set the DISABLE_ASSERTIONS=1 parameter when instantiating the module
     if(DISABLE_ASSERTIONS == 0) begin
         // Check if the match is one-hot
         `ASSERT_COND_CLK(i_compare_enable, $onehot0(o_compare_match[ENTRIES-1:0]), "tt_compare matched against more than one entry");
     end
   endgenerate
`endif

always_comb begin 
    o_compare_match[ENTRIES-1:0]   = {ENTRIES{1'b0}};
    for(e=0; e<ENTRIES; e++) begin
       o_compare_match[e] =      i_compare_enable
                            &  ( i_entry_valids[e] | i_compare_valid_mask ) 
                            & ~(
                                  |( 
                                         (     i_compare_value[VALUE_WIDTH-1:0] ^ i_entry_values[e][VALUE_WIDTH-1:0]) 
                                      &  ~i_compare_value_mask[VALUE_WIDTH-1:0]
                                   )
                               );
    end
end



endmodule
