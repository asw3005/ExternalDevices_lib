`timescale 1ns / 100ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 06/30/2025 12:15:48 PM
// Design Name: 
// Module Name: DataSelector_tb
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


module DataSelector_tb #(

    //Clock def.
    localparam GCLK             = 2.5,
    localparam GPAUSE           = 20,

    localparam CNT_MAX_VAL      = 32,
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
);

integer i;     

reg clk;
reg rst;
reg [31:0] ctrl_reg;
reg [31:0] data_sel_reg;
reg [CNT_AMOUNT - 1:0][CNT_MAX_VAL - 1:0] in_cnt_data;
wire [CNT_MAX_VAL - 1:0] out_cnt_data;    

always #GCLK clk = ~clk;
    
DataSelector DataSelector_uut (

    .clk(clk),
    .rst(rst),
    .ctrl_reg(ctrl_reg),
    .data_sel_reg(data_sel_reg),
    .in_cnt_data(in_cnt_data),
    .out_cnt_data(out_cnt_data)
);
    
initial
    begin

        clk = 0;
        rst = 0;
        ctrl_reg = 0;
        data_sel_reg = 0;
        in_cnt_data = 0;
        #20
        rst = 1;
        #20
        rst = 0;
        #20 
        
        //Data enumiration.
        for(i = 0; i < CNT_AMOUNT; i++) begin
            in_cnt_data[i] = i;
            data_sel_reg = i;
            #GPAUSE;
        end
        
        data_sel_reg = 288;
        #GPAUSE
        data_sel_reg = 287;
        #GPAUSE
        data_sel_reg = 512;
        #GPAUSE
        data_sel_reg = 3;
        #GPAUSE
        
        // Check GRST_BIT.
        ctrl_reg[GRST_BIT] = 1;
        #GPAUSE
        ctrl_reg[GRST_BIT] = 0;
        #GPAUSE
         
        $finish;
    end    
    
    
    
    
    
endmodule
