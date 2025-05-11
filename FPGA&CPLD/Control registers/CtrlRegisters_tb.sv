`timescale 1ns / 1ps
/* */
module CtrlRegisters_tb; 

reg clk;
reg rst;
reg SpiClk;
reg SpiCs;
wire SpiMosi;
reg DataSync;
wire SpiCmdLock;
wire [31:0] SpiCmd;
wire [15:0][31:0] DataReg;

reg [31:0] CmdWord;

/* Comman register bits. */
localparam START_SEL_BIT 	= 0;
localparam STOP_SEL_BIT		= 1;
localparam THR_START_CH 	= 2;
localparam THR_START_HL		= 3;
localparam PRG_START_BIT	= 4;
localparam SYNC_BIT 		= 30;
localparam RST_BIT 			= 31;

/* SPI commands. */
localparam SPI_READ			= 1'b1;
localparam SPI_WRITE		= 1'b0;
localparam SPI_READ_BACK	= 4'b0001;
localparam SPI_COMMAND		= 4'b0010;
localparam SPI_SAMPLECNT	= 4'b0011;
localparam SPI_SCNTTEST		= 4'b0100;
localparam SPI_ADCRTEST		= 4'b0101;
localparam SPI_RFIFO0		= 4'b0110;
localparam SPI_RFIFO1		= 4'b0111;
localparam SPI_RPIPE		= 4'b1000;
localparam SPI_RSVD			= {6{1'b0}};
localparam SPI_DEF_ADDR		= {5{1'b0}};
localparam SPI_DUMMY		= {16{1'b0}};

/* Reggister map. */
localparam ControlReg 		= 5'd0;
localparam PeriodLimiter 	= 5'd1;
localparam FStartTim		= 5'd2;
localparam SStartTim		= 5'd3;
localparam FStopTim			= 5'd4;
localparam SStopTim			= 5'd5;
localparam ThrStartHigh		= 5'd6;
localparam ThrStartLow		= 5'd7;
localparam Thr0Stop			= 5'd8;
localparam Thr1Stop			= 5'd9;

localparam SPI_WCMD_CLK		= 64*2;

/* Registers. */
reg [63:0] SpiMosiCmdData = { SPI_DUMMY, SPI_WRITE, SPI_COMMAND, SPI_RSVD, SPI_DEF_ADDR, 16'h86A0, 16'h0001 };

integer i;


always #5 clk = ~clk;


CtrlRegisters CtrlRegisters_uut (

	/* Clocks. */
	.clk(clk),
	.SpiClk(SpiClk),
	/* Ext logic reset. */
	.rst(rst),

	/* Sync pulse. */
	.DataSync(DataSync),
	
	/* SPI command and register control. */
	.SpiCs(SpiCs),
	.SpiMosi(SpiMosi),	

	/* Outputs. */
	.SpiCmdLock(SpiCmdLock),
	.SpiCmd(SpiCmd),
	.DataReg(DataReg)	

);

/* Write register value. */
task WriteReg(input bit [4:0] addr, input bit [31:0] data);

		SpiMosiCmdData = { SPI_DUMMY, SPI_WRITE, SPI_COMMAND, SPI_RSVD, addr, data[15:0], data[31:16] };
		#20
		
		SpiCs = 0;
		#20
		
		for(i = 0; i < SPI_WCMD_CLK; i = i + 1) begin			
			#10 SpiClk = ~SpiClk;		
		end			
		
		#20
		SpiCs = 1;
		#20;

endtask


/* Testbench start. */
initial 
	begin
		$stop;
		
		clk 		= 0;
		SpiClk 		= 0;
		rst 		= 1;
		SpiCs 		= 1;
		//SpiMosi 	= 0;
		DataSync 	= 0;
		CmdWord		= 0;
		#20
		
		rst = 0;
		#20
		
		/* Write reg X. */
		WriteReg(5'd0, CmdWord);
		WriteReg(5'd1, 32'd100000001);
		WriteReg(5'd2, 32'd10001);
		WriteReg(5'd3, 32'd12501);
		WriteReg(5'd4, 32'd20001);
		WriteReg(5'd5, 32'd20001);
		WriteReg(5'd6, 32'd3073);
		WriteReg(5'd7, 32'd1025);
		WriteReg(5'd8, 32'd25);
		WriteReg(5'd9, 32'd25);
		
		WriteReg(5'd10, 32'd11);
		WriteReg(5'd11, 32'd12);
		WriteReg(5'd12, 32'd13);
		WriteReg(5'd13, 32'd14);
		WriteReg(5'd14, 32'd15);
		WriteReg(5'd15, 32'd16);	
		
		
		CmdWord[SYNC_BIT] = 1;
		#20
		WriteReg(5'd0, CmdWord);
		#20
	
		$stop;
	end
	

always @(posedge SpiClk) begin

	{ /* SpiMosi, */ SpiMosiCmdData[63:0] } <= { SpiMosiCmdData[63:0] , 1'b0 };

end

assign SpiMosi = SpiMosiCmdData[63];

endmodule