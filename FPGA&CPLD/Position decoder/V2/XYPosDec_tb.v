`timescale 1ns / 1ns
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 04/03/2025 04:04:55 PM
// Design Name: 
// Module Name: XYPosDec_tb
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


module XYPosDec_tb#(

localparam IN_WIDTH = 32,
localparam OUT_WIDTH = 6

);

integer i;
reg clock;
reg [IN_WIDTH - 1:0] InputX;						 
reg [IN_WIDTH - 1:0] InputY;						 
wire [OUT_WIDTH - 1:0] OutputX;						 
wire [OUT_WIDTH - 1:0] OutputY;


always #5 clock = ~clock;

XYPosDec XYPosDec_uut (
    .clock(clock),
    .InputX(InputX),
    .InputY(InputY),
    .OutputX(OutputX),
    .OutputY(OutputY)
);

initial
    begin
        i = 0;
        clock = 0;
        InputX = 0;
        InputY = 0;
        #20
        //InputXY test.
        for(i = 0; i < 32; i = i + 1) begin
            InputX = 32'd1 << i;
            InputY = 32'd1 << i;
            #20;
        end
        #20
        InputX = 33;
        InputY = 33;
        #20
        InputX = 12;
        InputY = 12;
        #20
        InputX = 24;
        InputY = 24;
        #20
        InputX = 45;
        InputY = 45;
        #20
        InputX = 32'h80000001;
        InputY = 32'h80000001;
        #20
        InputX = 32'h80011001;
        InputY = 32'h80011001;
        #20
        InputX = 32'b0101;
        InputY = 32'b0101;
        #20
        InputX = 32'b0100;
        InputY = 32'b0100;
        #20
        //InputXY test.
        for(i = 0; i < 33; i = i + 1) begin
            InputX = i;
            InputY = i;
            #20;
        end
        #20
        $finish;
    end		

						 

endmodule
