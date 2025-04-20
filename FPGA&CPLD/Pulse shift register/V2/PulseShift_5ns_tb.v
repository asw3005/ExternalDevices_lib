`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 04/10/2025 10:01:03 AM
// Design Name: 
// Module Name: PulseShift_5ns_tb
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


module PulseShift_5ns_tb #(

    localparam XY_DEC_IO_WIDTH  = 6
    )
    
    (
    );
    
reg rst;
reg clock;
reg sync_data;
reg [XY_DEC_IO_WIDTH - 1:0] InXData;
reg [XY_DEC_IO_WIDTH - 1:0] InYData;
wire [XY_DEC_IO_WIDTH - 1:0] OutXData; 
wire [XY_DEC_IO_WIDTH - 1:0] OutYData;    
    
always #2.5 clock = ~clock;

PulseShift_5ns PulseShift_5ns_uut (
    .rst(rst),
    .clock(clock),
    .sync_data(sync_data),
    .XShiftCtrl(5),
    .YShiftCtrl(0),
    .InXData(InXData),
    .InYData(InYData),
    .OutXData(OutXData),
    .OutYData(OutYData)
);



initial
    begin

        clock = 0;
        rst = 0;
        InXData = 0;
        InYData = 0;
        #20
        rst = 1;
        #20
        rst = 0;
        #20
        sync_data = 1;
        #40
        sync_data = 0;
        #20
        InXData = 6'b101101;
        InYData = 6'b101101;
        #300

        
        $finish;
    end	

    
endmodule
