`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 06/27/2025 11:40:36 AM
// Design Name: 
// Module Name: CntConnectWrapper
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module CntConnectWrapper #(

    parameter CNT_MAX_VAL       = 32,

    localparam IN_X_WIDTH      = 16,
    localparam IN_Y_WIDTH      = 16,
    localparam X_CNT_OFFSET    = 256,
    localparam Y_CNT_OFFSET    = 272,
    
    // Registers' bits position.
	localparam SYNC_BIT        = 0,
	localparam RST_CNT_BIT     = 1,
	localparam EN_DIS_CNT_BIT  = 2,
	localparam LATCH_CNT_BIT   = 3,
	localparam GRST_BIT        = 4
    )
    (
    // Input registers
    // Control register.
    input wire [31:0] in_control_reg,
    // Counter selector.
    input wire [31:0] in_cnt_data_selector, 
    // Counter data output.         
    output wire [31:0] out_cnt_data,
    // RSVD.
    input wire [31:0] in_RSVD3,
    input wire [31:0] in_RSVD4,

    //
    input clk,
    input rst,
    input [IN_X_WIDTH - 1:0] x_in,
    input [IN_Y_WIDTH - 1:0] y_in
        
    );
    

genvar i, j;
    
wire [287:0][31:0] wireCntOut;     
    
/* Counter multiplexer control. */
DataSelector #(CNT_MAX_VAL)  DataSelector_inst0(

    .clk(clk),
    .rst(rst),
    .ctrl_reg(in_control_reg),
    .data_sel_reg(in_cnt_data_selector),
    .in_cnt_data(wireCntOut),
    .out_cnt_data(out_cnt_data)
);

/* XY counters connection. */
generate

for(i = 0; i < IN_X_WIDTH; i++) 
begin: genXLine
	for(j = 0; j < IN_Y_WIDTH; j++) 
		begin: genYLine		
			PixelCnt #(CNT_MAX_VAL) record(			
                .clk(clk),
                .rst(rst),
                .ctrl_reg(in_control_reg),
                .x_in(x_in[i]),
                .y_in(y_in[j]),
                .cnt_out(wireCntOut[i*IN_X_WIDTH + j])
			);
		end
end   

endgenerate
    
/* Connect X counters to the selector. */
generate

for(i = 0; i < IN_X_WIDTH; i++) 
	begin: genXline			
		ChannelCnt #(CNT_MAX_VAL) record(		
            .clk(clk),
            .rst(rst),
            .ctrl_reg(in_control_reg),
            .xy_in(x_in[i]),
            .cnt_out(wireCntOut[X_CNT_OFFSET + i])
		);
	end
		
endgenerate

/* Connect Y counters to the selector. */
generate

for(i = 0; i < IN_Y_WIDTH; i++) 
	begin: genYline			
		ChannelCnt #(CNT_MAX_VAL) record(
            .clk(clk),
            .rst(rst),
            .ctrl_reg(in_control_reg),
            .xy_in(y_in[i]),
            .cnt_out(wireCntOut[Y_CNT_OFFSET + i])
		);
	end
		
endgenerate
    
endmodule
