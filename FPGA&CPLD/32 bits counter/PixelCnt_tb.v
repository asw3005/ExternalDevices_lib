`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 06/25/2025 11:15:59 AM
// Design Name: 
// Module Name: PixelCnt_tb
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


module PixelCnt_tb #(

    localparam CNT_MAX_VAL      = 32,
    localparam IN_PULSE_WIDTH   = 10,
    // Registers' bits position.
    localparam SYNC_BIT        = 0,
    localparam RST_CNT_BIT     = 1,
    localparam EN_DIS_CNT_BIT  = 2,
    localparam LATCH_CNT_BIT   = 3,
    localparam GRST_BIT        = 4
)
(
);
    
integer i;
    
reg clk;
reg rst;
reg [31:0] ctrl_reg;
reg x_in;
reg y_in;
wire [CNT_MAX_VAL - 1:0] cnt_out;
    
always #2.5 clk = ~clk;

PixelCnt PixelCnt_uut (
    .clk(clk),
    .rst(rst),
    .ctrl_reg(ctrl_reg),
    .x_in(x_in),
    .y_in(y_in),
    .cnt_out(cnt_out)
);



initial
    begin

        clk = 0;
        rst = 0;
        x_in = 0;
        y_in = 0;
        ctrl_reg = 0;
        #20
        rst = 1;
        
        // Check counter with no cycling.
        #20
        rst = 0;
        ctrl_reg[EN_DIS_CNT_BIT] = 1;
        x_in = 1;
        #10
        y_in = 1;
        #30
        x_in = 0;
        y_in = 0;
        #5
        x_in = 1;
        y_in = 1;
        #20
        x_in = 0;
        y_in = 0;
        
        // Check counter with cycling.
        #50        
        for(i = 0; i < 50; i = i +1) begin        
            x_in = ~x_in;
            y_in = ~y_in;
            #IN_PULSE_WIDTH;        
        end
        
        ctrl_reg[EN_DIS_CNT_BIT] = 0;
        ctrl_reg[LATCH_CNT_BIT] = 1;
        #50
        ctrl_reg[LATCH_CNT_BIT] = 0;
        ctrl_reg[RST_CNT_BIT] = 1;
        #5
        ctrl_reg[RST_CNT_BIT] = 0;
        
        // Check counter EN_BIT = 0.
        #50        
        for(i = 0; i < 10; i = i +1) begin        
            x_in = ~x_in;
            y_in = ~y_in;
            #IN_PULSE_WIDTH;        
        end
        
        ctrl_reg[EN_DIS_CNT_BIT] = 1;        
        #50        
        for(i = 0; i < 100; i = i +1) begin        
            x_in = ~x_in;
            y_in = ~y_in;
            #IN_PULSE_WIDTH;        
        end
        
        ctrl_reg[EN_DIS_CNT_BIT] = 0;
        ctrl_reg[LATCH_CNT_BIT] = 1;
        #50
        ctrl_reg[LATCH_CNT_BIT] = 0;
        
        // Check counter with GRST_BIT.
        ctrl_reg[GRST_BIT] = 1;
        ctrl_reg[EN_DIS_CNT_BIT] = 1;        
        #50        
        for(i = 0; i < 10; i = i +1) begin        
            x_in = ~x_in;
            y_in = ~y_in;
            #IN_PULSE_WIDTH;        
        end
        
        #20
        ctrl_reg[GRST_BIT] = 0;      
        #50        
        for(i = 0; i < 10; i = i +1) begin        
            x_in = ~x_in;
            y_in = ~y_in;
            #IN_PULSE_WIDTH;        
        end
        
        ctrl_reg[EN_DIS_CNT_BIT] = 0;
        ctrl_reg[LATCH_CNT_BIT] = 1;
        #50
        ctrl_reg[LATCH_CNT_BIT] = 0;
        #25
        
        $finish;
    end	    
    
endmodule
