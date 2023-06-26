// See LICENSE.TT for license details.

`include "briscv_defines.h"

module tt_cam_buffer #(parameter 
                    ALLOW_CAM_MULITIHIT=0,
                    TAG_WIDTH=32, 
                    DATA_WIDTH=32,                     
                    ENTRIES=8, 
                    ENTRIES_LOG2=$clog2(ENTRIES), 
                    TAG_WRITE_PORTS=1, 
                    DATA_WRITE_PORTS=1,                     
                    READ_PORTS=1, 
                    READ_WIDTH=(TAG_WIDTH+DATA_WIDTH),
                    DISABLE_WRITE_MUX_ASSERTIONS=0,
                    CAM_PORTS=1)                    
(
   input  logic                        i_clk                                      ,
   input  logic                        i_reset_n                                  ,

   input  logic                        i_read_en                [READ_PORTS-1:0]  ,       // enable signal for direct read of an entire buffer entry (tag+data)
   input  logic  [ENTRIES_LOG2-1:0]    i_read_addr              [READ_PORTS-1:0]  ,       // address of the buffer entry to be read
   output logic    [READ_WIDTH-1:0]    o_read_value             [READ_PORTS-1:0]  ,       // output value of the buffer entry which is being read

   input  logic                        i_compare_en              [CAM_PORTS-1:0]  ,       // will compare the given tag value against the existing entries (but not necessarily mux out the associated data when a match is found)
   input  logic                        i_compare_read_en         [CAM_PORTS-1:0]  ,       // when both the i_compare_en and i_compare_read_en are active, then the data from the matching entry will be muxed onto the o_cam_data bus
   input  logic     [TAG_WIDTH-1:0]    i_compare_tag_value       [CAM_PORTS-1:0]  ,       // tag value to compare
   input  logic     [TAG_WIDTH-1:0]    i_compare_tag_value_mask  [CAM_PORTS-1:0]  ,       // tag value to mask the compare; setting the bit position will exclude the bit from the comparison
   input  logic                        i_compare_tag_valid_mask  [CAM_PORTS-1:0]  ,       // masks the tag_valid bit from the compare
   output logic    [DATA_WIDTH-1:0]    o_cam_data_value          [CAM_PORTS-1:0]  ,       // data value associated with the matching tag value (only valid when the compare_read_en signal is active)
   output logic       [ENTRIES-1:0]    o_compare_tag_hit         [CAM_PORTS-1:0]  ,       // bit vector which indicates which valid entry(s) were matched by the given tag_value

   input  logic                        i_write_tag_en      [TAG_WRITE_PORTS-1:0]  ,       // enable signal for direct write of the tag portion of a buffer entry
   input  logic  [ENTRIES_LOG2-1:0]    i_write_tag_addr    [TAG_WRITE_PORTS-1:0]  ,       // address of the buffer entry whose tag portion is to be written
   input  logic     [TAG_WIDTH-1:0]    i_write_tag_value   [TAG_WRITE_PORTS-1:0]  ,       // value to be written into the tag portion of the buffer entry

   input  logic                        i_write_data_en    [DATA_WRITE_PORTS-1:0]  ,       // enable signal for direct write of the data portion of a buffer entry
   input  logic  [ENTRIES_LOG2-1:0]    i_write_data_addr  [DATA_WRITE_PORTS-1:0]  ,       // address of the buffer entry whose data portion is to be written
   input  logic    [DATA_WIDTH-1:0]    i_write_data_value [DATA_WRITE_PORTS-1:0]  ,       // value to be written into the data portion of the buffer entry

   input  logic       [ENTRIES-1:0]    i_set_tag_valid                            ,       // signal to set the tag valid bit of a buffer entry (would be expected to accompany a write of the tag to enable a later compare match)
   input  logic       [ENTRIES-1:0]    i_clear_tag_valid                          ,       // signal to clear the tag valid bit of a buffer entry (to prevent a later compare match); Clear is dominant over set!
   output logic       [ENTRIES-1:0]    o_broadside_tag_valid                      ,       // broadside output of the tag valid bit for all buffer entries
   output logic     [TAG_WIDTH-1:0]    o_broadside_tag_value       [ENTRIES-1:0]  ,       // broadside output of the tag values for all buffer entries

   input  logic       [ENTRIES-1:0]    i_set_data_valid                           ,       // signal to set the data valid bit of a buffer entry (would be expected to accompany a write of the data to enable a later check of the data valid status)
   input  logic       [ENTRIES-1:0]    i_clear_data_valid                         ,       // signal to clear the data valid bit of a buffer entry (might accompnay the set_tag_valid when the data portion is not yet written); Clear is dominant over set!
   output logic       [ENTRIES-1:0]    o_broadside_data_valid                     ,       // broadside output of the data valid bit for all buffer entries
   output logic    [DATA_WIDTH-1:0]    o_broadside_data_value      [ENTRIES-1:0]          // broadside output of the data values for all buffer entries
);


genvar e ;

integer wrport, camport, i;

logic                             any_tag_valid_update                                  ;
logic                             any_data_valid_update                                 ;
logic  [ENTRIES-1:0]              entry_write_tag_en                                    ;
logic  [ENTRIES-1:0]              entry_write_data_en                                   ;
                 
logic  [ENTRIES-1:0]              port_write_tag_en          [TAG_WRITE_PORTS-1:0]      ;
logic  [ENTRIES-1:0]              port_write_data_en        [DATA_WRITE_PORTS-1:0]      ;
logic  [TAG_WRITE_PORTS-1:0]      entry_write_tag_mux_sel            [ENTRIES-1:0]      ;
logic  [DATA_WRITE_PORTS-1:0]     entry_write_data_mux_sel           [ENTRIES-1:0]      ;
                 
logic  [ENTRIES-1:0]              entry_tag_valid_in                                    ;
logic  [ENTRIES-1:0]              entry_tag_valid_q                                     ;
logic  [TAG_WIDTH-1:0]            entry_tag_in                  [ENTRIES-1:0]           ;
logic  [TAG_WIDTH-1:0]            entry_tag_q                   [ENTRIES-1:0]           ;

logic  [ENTRIES-1:0]              entry_data_valid_in                                   ;
logic  [ENTRIES-1:0]              entry_data_valid_q                                    ;
logic  [DATA_WIDTH-1:0]           entry_data_in                 [ENTRIES-1:0]           ;
logic  [DATA_WIDTH-1:0]           entry_data_q                  [ENTRIES-1:0]           ;
                 
logic  [ENTRIES-1:0]              port_read_mux_sel             [READ_PORTS-1:0]        ;
                 
logic  [READ_WIDTH-1:0]           entry_combined_q              [ENTRIES-1:0]           ;
                 
logic  [ENTRIES-1:0]              tag_compare_match             [CAM_PORTS-1:0]         ;
logic                             tag_compare_port_any_match    [CAM_PORTS-1:0]         ;



// Use a common enable for all valid bit updates, since it's a smaller number of flops and may not otherwise be given a clock gater
assign any_tag_valid_update =     (|  i_set_tag_valid)
                                | (|i_clear_tag_valid);

assign any_data_valid_update =     (|  i_set_data_valid)
                                 | (|i_clear_data_valid);

assign entry_tag_valid_in[ENTRIES-1:0]  = (entry_tag_valid_q[ENTRIES-1:0]  | i_set_tag_valid[ENTRIES-1:0])  & ~i_clear_tag_valid[ENTRIES-1:0];

assign entry_data_valid_in[ENTRIES-1:0] = (entry_data_valid_q[ENTRIES-1:0] | i_set_data_valid[ENTRIES-1:0]) & ~i_clear_data_valid[ENTRIES-1:0];

// combine all ports into a "per entry" write enable signal for use as the flop enable
always_comb begin 
   entry_write_tag_en[ENTRIES-1:0]   = {ENTRIES{1'b0}};
  entry_write_data_en[ENTRIES-1:0]   = {ENTRIES{1'b0}};        
  for(wrport=0; wrport<TAG_WRITE_PORTS; wrport++) begin
       entry_write_tag_en[ENTRIES-1:0] =  entry_write_tag_en[ENTRIES-1:0] |  port_write_tag_en[wrport][ENTRIES-1:0];
  end
  for(wrport=0; wrport<DATA_WRITE_PORTS; wrport++) begin
      entry_write_data_en[ENTRIES-1:0] = entry_write_data_en[ENTRIES-1:0] | port_write_data_en[wrport][ENTRIES-1:0];
  end
end


tt_pipe_stage #(.WIDTH (ENTRIES)) 
u_tag_valid_flops (
  .i_clk     ( i_clk                                 ), 
  .i_reset_n ( i_reset_n                             ), 
  .i_en      ( any_tag_valid_update                  ),
  .i_d       ( entry_tag_valid_in[ENTRIES-1:0]       ), 
  .o_q       ( entry_tag_valid_q[ENTRIES-1:0]        )
);

assign o_broadside_tag_valid[ENTRIES-1:0] = entry_tag_valid_q[ENTRIES-1:0];

tt_pipe_stage #(.WIDTH (ENTRIES)) 
u_data_valid_flops (
  .i_clk     ( i_clk                                 ), 
  .i_reset_n ( i_reset_n                             ), 
  .i_en      ( any_data_valid_update                  ),
  .i_d       ( entry_data_valid_in[ENTRIES-1:0]       ), 
  .o_q       ( entry_data_valid_q[ENTRIES-1:0]        )
);

assign o_broadside_data_valid[ENTRIES-1:0] = entry_data_valid_q[ENTRIES-1:0];

generate
    for (e=0; e<ENTRIES; e=e+1) begin 

      assign entry_combined_q[e][READ_WIDTH-1:0] = {entry_tag_q[e][TAG_WIDTH-1:0], entry_data_q[e][DATA_WIDTH-1:0]};

      tt_pipe_stage #(.WIDTH (TAG_WIDTH)) 
      u_tag_flops (
        .i_clk     ( i_clk                                 ), 
        .i_reset_n ( i_reset_n                             ), 
        .i_en      ( entry_write_tag_en[e]                 ),
        .i_d       ( entry_tag_in[e][TAG_WIDTH-1:0]        ), 
        .o_q       ( entry_tag_q[e][TAG_WIDTH-1:0]         )
      );

      tt_pipe_stage #(.WIDTH (DATA_WIDTH)) 
      u_data_flops (
        .i_clk     ( i_clk                                 ), 
        .i_reset_n ( i_reset_n                             ), 
        .i_en      ( entry_write_data_en[e]                ),
        .i_d       ( entry_data_in[e][DATA_WIDTH-1:0]      ), 
        .o_q       ( entry_data_q[e][DATA_WIDTH-1:0]       )
      );

    end
endgenerate

assign o_broadside_tag_value  = entry_tag_q;
assign o_broadside_data_value = entry_data_q;

// --------------------------------------------------------------------------------------------------------------------------------------------------
// CAM READ LOGIC
// --------------------------------------------------------------------------------------------------------------------------------------------------
always_comb begin 
  for(camport=0; camport<CAM_PORTS; camport++) begin
    tag_compare_port_any_match[camport]   = |tag_compare_match[camport][ENTRIES-1:0];
  end
end


tt_compare #(
  .DISABLE_ASSERTIONS  (ALLOW_CAM_MULITIHIT),
  .VALUE_WIDTH         (TAG_WIDTH),
  .ENTRIES             (ENTRIES)  
)
u_tag_compare [CAM_PORTS-1:0]
(
    .i_clk                  (i_clk),
    .i_reset_n              (i_reset_n),
    .i_compare_enable       (i_compare_en),
    .i_compare_value        (i_compare_tag_value),    
    .i_compare_value_mask   (i_compare_tag_value_mask),        
    .i_compare_valid_mask   (i_compare_tag_valid_mask),            
    .i_entry_values         (entry_tag_q),
    .i_entry_valids         (entry_tag_valid_q),
    .o_compare_match        (tag_compare_match)
);


//assign o_compare_hit[CAM_PORTS-1:0][ENTRIES-1:0] = tag_compare_match[CAM_PORTS-1:0][ENTRIES-1:0];
assign o_compare_tag_hit = tag_compare_match;

tt_decoded_mux #(
  .VALUE_WIDTH     (DATA_WIDTH),
  .MUX_WIDTH       (ENTRIES)  
)
u_cam_read_mux [CAM_PORTS-1:0]
(
    .i_clk             (i_clk),
    .i_reset_n         (i_reset_n),
    .i_enable          (tag_compare_port_any_match),
    .i_inputs          (entry_data_q[ENTRIES-1:0]),
    .i_select          (tag_compare_match),    
    .o_output          (o_cam_data_value)
);


// --------------------------------------------------------------------------------------------------------------------------------------------------
// READ PORT LOGIC
// --------------------------------------------------------------------------------------------------------------------------------------------------

tt_decoder #(
  .DECODED_WIDTH     (ENTRIES),
  .ENCODED_WIDTH     (ENTRIES_LOG2)
)
u_read_entry_decoder [READ_PORTS-1:0]
(
    .i_clk             (i_clk),
    .i_reset_n         (i_reset_n),
    .i_enable          (i_read_en),
    .i_encoded_signal  (i_read_addr),
    .o_decoded_signal  (port_read_mux_sel)          
);

tt_decoded_mux #(
  .VALUE_WIDTH     (READ_WIDTH),
  .MUX_WIDTH       (ENTRIES)  
)
u_read_mux [READ_PORTS-1:0]
(
    .i_clk             (i_clk),
    .i_reset_n         (i_reset_n),
    .i_enable          (i_read_en),
    .i_inputs          (entry_combined_q[ENTRIES-1:0]),
    .i_select          (port_read_mux_sel),    
    .o_output          (o_read_value)
);


// --------------------------------------------------------------------------------------------------------------------------------------------------
// WRITE PORT LOGIC
// --------------------------------------------------------------------------------------------------------------------------------------------------

tt_decoder #(
  .DECODED_WIDTH     (ENTRIES),
  .ENCODED_WIDTH     (ENTRIES_LOG2)
)
u_write_tag_en_decoder [TAG_WRITE_PORTS-1:0]
(
    .i_clk             (i_clk),
    .i_reset_n         (i_reset_n),
    .i_enable          (i_write_tag_en),
    .i_encoded_signal  (i_write_tag_addr),
    .o_decoded_signal  (port_write_tag_en)          
);

tt_decoder #(
  .DECODED_WIDTH     (ENTRIES),
  .ENCODED_WIDTH     (ENTRIES_LOG2)
)
u_write_data_en_decoder [DATA_WRITE_PORTS-1:0]
(
    .i_clk             (i_clk),
    .i_reset_n         (i_reset_n),
    .i_enable          (i_write_data_en),
    .i_encoded_signal  (i_write_data_addr),
    .o_decoded_signal  (port_write_data_en)          
);

// For each entry, we need to create a mux select which muxes in the data from one of the WRITE_PORTS; 
// What we have so far is enable signals of the shape [WRITE_PORTS][ENTRIES], and what we want is to convert that to [ENTRIES][WRITE_PORTS]

tt_reshape #(
  .X_WIDTH         (TAG_WRITE_PORTS),
  .Y_WIDTH         (ENTRIES)
)
u_write_tag_en_reshape
(
  .i_xy_signal     (port_write_tag_en[TAG_WRITE_PORTS-1:0]),
  .o_yx_signal     (entry_write_tag_mux_sel[ENTRIES-1:0])
);

tt_reshape #(
  .X_WIDTH         (DATA_WRITE_PORTS),
  .Y_WIDTH         (ENTRIES)
)
u_write_data_en_reshape
(
  .i_xy_signal     (port_write_data_en[DATA_WRITE_PORTS-1:0]),
  .o_yx_signal     (entry_write_data_mux_sel[ENTRIES-1:0])
);

tt_decoded_mux #(
  .VALUE_WIDTH     (TAG_WIDTH),
  .MUX_WIDTH       (TAG_WRITE_PORTS),
  .DISABLE_ASSERTIONS (DISABLE_WRITE_MUX_ASSERTIONS)
)
u_write_tag_mux [ENTRIES-1:0]
(
    .i_clk             (i_clk),
    .i_reset_n         (i_reset_n),
    .i_enable          (entry_write_tag_en),
    .i_inputs          (i_write_tag_value[TAG_WRITE_PORTS-1:0]),
    .i_select          (entry_write_tag_mux_sel),    
    .o_output          (entry_tag_in)
);

tt_decoded_mux #(
  .VALUE_WIDTH     (DATA_WIDTH),
  .MUX_WIDTH       (DATA_WRITE_PORTS),
  .DISABLE_ASSERTIONS (DISABLE_WRITE_MUX_ASSERTIONS)
)
u_write_data_mux [ENTRIES-1:0]
(
    .i_clk             (i_clk),
    .i_reset_n         (i_reset_n),
    .i_enable          (entry_write_data_en),
    .i_inputs          (i_write_data_value[DATA_WRITE_PORTS-1:0]),
    .i_select          (entry_write_data_mux_sel),    
    .o_output          (entry_data_in)
);





endmodule
