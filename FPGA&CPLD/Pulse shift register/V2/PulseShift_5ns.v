//
//Pulse shift 5nS

module PulseShift_5ns #(

    localparam XY_DEC_IO_WIDTH  = 6,
    localparam SHIFT_CTRL_WIDTH = 6,
    localparam SH_DELAY_WIDTH   = 12,
    localparam OUT_WIDTH        = 6,
    /* Minimum is 2. */
    localparam MAX_SHIFT        = 32
    )								 
    (
    input rst,
    input clock,
    input sync_data,
    input wire [SHIFT_CTRL_WIDTH - 1:0] XShiftCtrl,
    input wire [SHIFT_CTRL_WIDTH - 1:0] YShiftCtrl,
    input wire [XY_DEC_IO_WIDTH - 1:0] InXData,
    input wire [XY_DEC_IO_WIDTH - 1:0] InYData,
    output reg [XY_DEC_IO_WIDTH - 1:0] OutXData,
    output reg [XY_DEC_IO_WIDTH - 1:0] OutYData						  
    );

integer i;

reg [0:0] SyncLock;
reg [SH_DELAY_WIDTH - 1:0] DelaySelect; 

reg [MAX_SHIFT - 1:0]  XShiftReg0;
reg [MAX_SHIFT - 1:0]  XShiftReg1;
reg [MAX_SHIFT - 1:0]  XShiftReg2;
reg [MAX_SHIFT - 1:0]  XShiftReg3;
reg [MAX_SHIFT - 1:0]  XShiftReg4;
reg [MAX_SHIFT - 1:0]  XShiftReg5;

reg [MAX_SHIFT - 1:0] YShiftReg0;
reg [MAX_SHIFT - 1:0] YShiftReg1;
reg [MAX_SHIFT - 1:0] YShiftReg2;
reg [MAX_SHIFT - 1:0] YShiftReg3;
reg [MAX_SHIFT - 1:0] YShiftReg4;
reg [MAX_SHIFT - 1:0] YShiftReg5;

/* Data delay selector update. */
always @(posedge clock) begin
	
	if(rst) begin
	   DelaySelect <= 0;
	   SyncLock    <= 0;
	end
	else begin
	
	   if(sync_data & !SyncLock) begin
	       DelaySelect <= { YShiftCtrl, XShiftCtrl };
	       SyncLock <= 1;
	   end
	   else if(!sync_data) begin 
	       SyncLock <= 0;
	   end
	
	end	
end

/* Delay shift registers. */
always @(posedge clock) begin

    XShiftReg0[0] <= InXData[0];
    XShiftReg0[MAX_SHIFT - 1:1] <= XShiftReg0[MAX_SHIFT - 2:0];
    XShiftReg1[0] <= InXData[1];
    XShiftReg1[MAX_SHIFT - 1:1] <= XShiftReg1[MAX_SHIFT - 2:0];
    XShiftReg2[0] <= InXData[2];
    XShiftReg2[MAX_SHIFT - 1:1] <= XShiftReg2[MAX_SHIFT - 2:0];
    XShiftReg3[0] <= InXData[3];
    XShiftReg3[MAX_SHIFT - 1:1] <= XShiftReg3[MAX_SHIFT - 2:0];
    XShiftReg4[0] <= InXData[4];
    XShiftReg4[MAX_SHIFT - 1:1] <= XShiftReg4[MAX_SHIFT - 2:0];
    XShiftReg5[0] <= InXData[5];
    XShiftReg5[MAX_SHIFT - 1:1] <= XShiftReg5[MAX_SHIFT - 2:0];

    YShiftReg0[0] <= InYData[0];
    YShiftReg0[MAX_SHIFT - 1:1] <= YShiftReg0[MAX_SHIFT - 2:0];
    YShiftReg1[0] <= InYData[1];
    YShiftReg1[MAX_SHIFT - 1:1] <= YShiftReg1[MAX_SHIFT - 2:0];
    YShiftReg2[0] <= InYData[2];
    YShiftReg2[MAX_SHIFT - 1:1] <= YShiftReg2[MAX_SHIFT - 2:0];
    YShiftReg3[0] <= InYData[3];
    YShiftReg3[MAX_SHIFT - 1:1] <= YShiftReg3[MAX_SHIFT - 2:0];
    YShiftReg4[0] <= InYData[4];
    YShiftReg4[MAX_SHIFT - 1:1] <= YShiftReg4[MAX_SHIFT - 2:0];
    YShiftReg5[0] <= InYData[5];
    YShiftReg5[MAX_SHIFT - 1:1] <= YShiftReg5[MAX_SHIFT - 2:0];

end

/* Delay selecting. */
always @(posedge clock) begin
	 
    if(DelaySelect[5:0] < 6'd32) begin
	   OutXData <= { XShiftReg5[DelaySelect[5:0]], XShiftReg4[DelaySelect[5:0]], XShiftReg3[DelaySelect[5:0]], XShiftReg2[DelaySelect[5:0]], XShiftReg1[DelaySelect[5:0]], XShiftReg0[DelaySelect[5:0]] };
	end
	else if(DelaySelect[5:0] == 6'd63) begin
	   OutXData <= InXData;
	end
	else begin
	   OutXData <= { XShiftReg5[0], XShiftReg4[0], XShiftReg3[0], XShiftReg2[0], XShiftReg1[0], XShiftReg0[0] };
	end
	

    if(DelaySelect[11:6] < 6'd32) begin
	   OutYData <= { YShiftReg5[DelaySelect[11:6]], YShiftReg4[DelaySelect[11:6]], YShiftReg3[DelaySelect[11:6]], YShiftReg2[DelaySelect[11:6]], YShiftReg1[DelaySelect[11:6]], YShiftReg0[DelaySelect[11:6]] };
	end
	else if(DelaySelect[11:6] == 6'd63) begin
	   OutYData <= InYData;
	end
	else begin
	   OutYData <= { YShiftReg5[0], YShiftReg4[0], YShiftReg3[0], YShiftReg2[0], YShiftReg1[0], YShiftReg0[0] };
	end

end

endmodule