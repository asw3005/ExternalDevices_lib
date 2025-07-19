`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 06/24/2025 12:11:59 PM
// Design Name: 
// Module Name: DataSelector
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


module DataSelector #(

    parameter CNT_MAX_VAL       = 32,
    localparam DATA_SEL_VAL     = 11,
    localparam CNT_AMOUNT       = 288,
    // Registers' bits position.
	localparam SYNC_BIT        = 0,
	localparam RST_CNT_BIT     = 1,
	localparam EN_DIS_CNT_BIT  = 2,
	localparam LATCH_CNT_BIT   = 3,
	localparam GRST_BIT        = 4
)
(

    input clk,
    input rst,
    input [31:0] ctrl_reg,
    input [31:0] data_sel_reg,
    input [CNT_AMOUNT - 1:0][CNT_MAX_VAL - 1:0] in_cnt_data,
    output reg [CNT_MAX_VAL - 1:0] out_cnt_data

);
    
always @(posedge clk) begin 

    if(rst | ctrl_reg[GRST_BIT]) begin     
        out_cnt_data <= 32'hDEADBEAF;
    end
    else begin
        
        if(data_sel_reg[DATA_SEL_VAL - 1:0] < CNT_AMOUNT) begin
            out_cnt_data <= in_cnt_data[data_sel_reg[DATA_SEL_VAL - 1:0]]; 
        end
        else if(data_sel_reg[DATA_SEL_VAL - 1:0] >= CNT_AMOUNT) begin
            out_cnt_data <= 32'hDEADBEAF;
        end
        
    end        
end    

    
endmodule