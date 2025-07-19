`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 06/24/2025 12:11:59 PM
// Design Name: 
// Module Name: ChannelCnt
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


module ChannelCnt #(

    parameter CNT_MAX_VAL      = 32,
    //localparam XY_LOCK_THR      = 4'b001,
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
    input xy_in,
    output reg [CNT_MAX_VAL - 1:0] cnt_out

 );
    
reg [CNT_MAX_VAL - 1:0] counter;
reg [3:0] xy_lock;
//reg [2:0] latch_cnt_value;

// Sync XY pulses.
always @(posedge clk) begin
    xy_lock <= { xy_lock[2:0], xy_in };
end
 
// Counter logic.    
always @(posedge clk) begin

    if(rst | ctrl_reg[GRST_BIT]) begin 
        
        counter <= 0;
        cnt_out <= 0;
        
    end
    else begin
    
        if(ctrl_reg[RST_CNT_BIT]) begin
            counter <= 0;
        end
        else if(ctrl_reg[EN_DIS_CNT_BIT]) begin 
            // Catch the rising edge of XY pulse.       
            if(!xy_lock[3] & xy_lock[2]) begin
                counter <= counter + 1'b1;
            end    
        end
        
        if(ctrl_reg[LATCH_CNT_BIT]) begin
            cnt_out <= counter;
        end
        
    end

end
    
endmodule
