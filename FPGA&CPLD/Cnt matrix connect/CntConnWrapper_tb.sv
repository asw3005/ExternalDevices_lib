`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 07/01/2025 10:01:11 AM
// Design Name: 
// Module Name: CntConnWrapper_tb
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


module CntConnWrapper_tb #(

    //Clock def.
    localparam GCLK                 = 2.5,
    localparam GPAUSE               = 20,
    localparam IN_PULSE_WIDTH       = 10,

    localparam IN_X_WIDTH           = 16,
    localparam IN_Y_WIDTH           = 16,
    localparam X_CNT_OFFSET         = 256,
    localparam Y_CNT_OFFSET         = 272,
    
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
    
reg [31:0] in_control_reg;
reg [31:0] in_cnt_data_selector;   
wire [31:0] out_cnt_data;
reg [31:0] in_RSVD3;
reg [31:0] in_RSVD4;

reg clk;
reg rst;
reg [IN_X_WIDTH - 1:0] x_in;
reg [IN_Y_WIDTH - 1:0] y_in;    
    
always #GCLK clk = ~clk;    
    
CntConnectWrapper CntConnectWrapper_inst0(

    .in_control_reg(in_control_reg),
    .in_cnt_data_selector(in_cnt_data_selector),   
    .out_cnt_data(out_cnt_data),
    .in_RSVD3(in_RSVD3),
    .in_RSVD4(in_RSVD4),
    
    .clk(clk),
    .rst(rst),
    .x_in(x_in),
    .y_in(y_in)
);
    
    
    
    
initial
    begin

        clk = 0;
        rst = 0;
        x_in = 0;
        y_in = 0; 
        in_control_reg = 0;
        in_cnt_data_selector = 0;
        in_RSVD3 = 0;
        in_RSVD4 = 0;
        #20
        rst = 1;
        #20
        rst = 0;
        #20 
        
        in_control_reg[EN_DIS_CNT_BIT] = 1;
        //Data enumiration.
        // Check counter with cycling.
        #GPAUSE        
        for(i = 0; i < 20; i = i +1) begin        
            x_in = ~x_in;
            y_in = ~y_in;
            #IN_PULSE_WIDTH;        
        end  
        
        /* Latch timers' data. */
        in_control_reg[LATCH_CNT_BIT] = 1;
        #GPAUSE      
        in_control_reg[LATCH_CNT_BIT] = 0;
        
        #GPAUSE        
        for(i = 0; i < 20; i = i +1) begin        
            x_in[0] = ~x_in[0];
            y_in[0] = ~y_in[0];
            #IN_PULSE_WIDTH;        
        end 
        
        #GPAUSE        
        for(i = 0; i < 40; i = i +1) begin        
            x_in[0] = ~x_in[0];
            y_in[1] = ~y_in[1];
            #IN_PULSE_WIDTH;        
        end 
        
        /* Latch timers' data. */
        in_control_reg[LATCH_CNT_BIT] = 1;
        #GPAUSE      
        in_control_reg[LATCH_CNT_BIT] = 0;
        in_cnt_data_selector = 1;
        #GPAUSE
        in_cnt_data_selector = 288;
        #GPAUSE        
        in_cnt_data_selector = 256;
        #GPAUSE        
        in_cnt_data_selector = 257;      
        #GPAUSE
        in_cnt_data_selector = 258;
        #GPAUSE        
                    
        for(i = 0; i < 20; i = i +1) begin        
            x_in = ~x_in;
            y_in = ~y_in;
            #IN_PULSE_WIDTH;        
        end
        
        #GPAUSE        
        for(i = 0; i < 40; i = i +1) begin        
             y_in[15] = ~y_in[15];
            #IN_PULSE_WIDTH;        
        end 
       
        in_control_reg[LATCH_CNT_BIT] = 1;
        #GPAUSE      
        in_control_reg[LATCH_CNT_BIT] = 0;
        #GPAUSE
        in_cnt_data_selector = 287;
        #GPAUSE
              
        for(i = 0; i < 20; i = i +1) begin        
            x_in = ~x_in;
            y_in = ~y_in;
            #IN_PULSE_WIDTH;        
        end
        
        #200
        /* Latch timers' data. */
        in_control_reg[LATCH_CNT_BIT] = 1;
        #GPAUSE      
        in_control_reg[LATCH_CNT_BIT] = 0;
        in_cnt_data_selector = 289;
        #GPAUSE
    
        for(i = 0; i < 40; i = i +1) begin        
            x_in[0] = ~x_in[0];
            y_in[3] = ~y_in[3];
            #IN_PULSE_WIDTH;        
        end 
        
          /* Latch timers' data. */
        in_control_reg[LATCH_CNT_BIT] = 1;
        #GPAUSE      
        in_control_reg[LATCH_CNT_BIT] = 0;
        #GPAUSE
        
        in_cnt_data_selector = 3;
        #GPAUSE

        
        
       for (i = 0; i < 300; i++) begin        
            in_cnt_data_selector = i;
            #GPAUSE;    
       end
        
        // Check GRST_BIT.
        in_control_reg[GRST_BIT] = 1;
        #GPAUSE
        in_control_reg[GRST_BIT] = 0;
        #GPAUSE
         
        $finish;
    end    
    
endmodule
