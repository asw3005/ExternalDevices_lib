`timescale 1ns / 100ps
/* */
module FifoControl_tb; 

/* SPI command word bits. */
localparam SPI_DUMMY_MSB	= 31;
localparam SPI_DUMMY_LSB 	= 16;
localparam SPI_RW		 		= 15;
localparam SPI_DC_MSB	 	= 14;
localparam SPI_DC_LSB	 	= 11;
localparam SPI_RSVD_MSB 	= 10;
localparam SPI_RSVD_LSB 	= 5;
localparam SPI_ADDR_MSB 	= 4;
localparam SPI_ADDR_LSB 	= 0;

/* SPI commands. */
localparam SPI_READ			= 1'b1;
localparam SPI_WRITE			= 1'b0;
localparam SPI_READ_BACK	= 4'b0001;
localparam SPI_COMMAND		= 4'b0010;
localparam SPI_SAMPLECNT	= 4'b0011;
localparam SPI_SCNTTEST		= 4'b0100;
localparam SPI_ADCRTEST		= 4'b0101;
localparam SPI_RFIFO0		= 4'b0110;
localparam SPI_RFIFO1		= 4'b0111;
localparam SPI_RSVD			= {6{1'b0}};
localparam SPI_DEF_ADDR		= {5{1'b0}};
localparam SPI_DUMMY			= {16{1'b0}};

/* Comman register bits. */
localparam START_SEL_BIT 	= 0;
localparam STOP_SEL_BIT		= 1;
localparam THR_START_CH 	= 2;
localparam THR_START_HL		= 3;
localparam PRG_START_BIT	= 4;
localparam SYNC_BIT 			= 30;
localparam RST_BIT 			= 31;

localparam DATA_IN_WIDTH 	= 128;
localparam FIFO_PIPE			= 4;
localparam SPI_WCMD			= 64;
localparam SPI_DATA_RCNT	= 10;

reg wrclk;
reg clk;
reg SpiClk;

reg rst;
reg fifo_srst;


reg FAcqStart;
reg SAcqStart;

reg [31:0] CmdWord;
reg [31:0] FTimStopValue;
reg [31:0] STimStopValue;
reg [31:0] Thr0StopValue;
reg [31:0] Thr1StopValue;
reg [DATA_IN_WIDTH - 1:0] DataIn;
reg [31:0] SpiCmd;
reg SpiCmdLock;
reg SpiCs;
reg SpiMosi;

wire Buff0Full;
wire Buff1Full;
wire SpiMiso;

integer i;

always #5 clk = ~clk;
always #5 wrclk = ~wrclk;




FifoControl FifoControl_uut (

	/* No signls. */ 
	.SpiMosi(0),

	/* Resets. */
	.rst(rst),
	.fifo_srst(fifo_srst),
	/* clocks. */
	.wrclk(wrclk),
	.clk(clk),
	.SpiClk(SpiClk),
	
	/* SPI read/write and command. */ 
	.SpiCs(SpiCs),
	.CmdWord(CmdWord),
	.SpiCmdLock(SpiCmdLock),
	.SpiCmd(SpiCmd),
	
	
	/* Control data. */
	.DataIn(DataIn),
	.FTimStopValue(FTimStopValue),
	.STimStopValue(STimStopValue),
	.Thr0StopValue(Thr0StopValue),
	.Thr1StopValue(Thr1StopValue),	
	
	/* Start signals. */
	.FAcqStart(FAcqStart),
	.SAcqStart(SAcqStart),
	
	/* Outputs. */
	.Buff0Full(Buff0Full),
	.Buff1Full(Buff1Full),
	.SpiMiso(SpiMiso)
);

initial 
	begin
		$stop;
		rst 				= 1;
		fifo_srst 		= 1;
		
		wrclk 			= 0;
		clk 				= 0;
		SpiClk 			= 0;		
		
		
		SpiCs 			= 1;
		SpiCmdLock 		= 0;
		SpiCmd 			= { SPI_DUMMY, SPI_READ, SPI_RFIFO0, SPI_RSVD, SPI_DEF_ADDR };
		
		#20
		
		CmdWord[SYNC_BIT] 		= 1;
		CmdWord[STOP_SEL_BIT] 	= 0;		
		
		#20
		
		FAcqStart 		= 0;
		SAcqStart 		= 0;
		
		DataIn 			= { 32'h89_AB_CD_EF, 32'h89_AB_CD_EF, 32'h89_AB_CD_EF, 32'h89_AB_CD_EF };
		FTimStopValue 	= 100;
		STimStopValue 	= 100;
		Thr0StopValue 	= 10;
		Thr1StopValue 	= 10;
	
		#20
		
		rst 				= 0;
		fifo_srst 		= 0;
		//SpiCs 			= 0;
		
		#20
		
		SpiCmdLock 		= 1;		
		CmdWord[SYNC_BIT] = 0;
	
		#20
	
		FAcqStart 		= 1;
		SAcqStart 		= 1;
	
		#20
	
		FAcqStart 		= 0;
		SAcqStart 		= 0;
		
		#20
		SpiCs 			= 0;
	
		#200
		
		for(i = 0; i < (128*FIFO_PIPE + SPI_WCMD)*2; i = i + 1) begin
			
			#5 SpiClk = ~SpiClk;
		
		end
		
		#20
		SpiCs 			= 1;
		
		#20
		SpiCs 			= 0;
	
		#200
		
		for(i = 0; i < (128*SPI_DATA_RCNT + SPI_WCMD)*2; i = i + 1) begin
			
			#5 SpiClk = ~SpiClk;
		
		end
		
		#20
		SpiCs 			= 1;
		
		#5000
		
		fifo_srst 		= 1;
		#20
		fifo_srst 		= 0;
		#20
		
		//SpiCs 			= 0;
		FAcqStart 		= 1;
		SAcqStart 		= 1;
	
		#20
	
		FAcqStart 		= 0;
		SAcqStart 		= 0;
	
		#20
		SpiCs 			= 0;
	
		#200
		
		for(i = 0; i < (128*FIFO_PIPE + SPI_WCMD)*2; i = i + 1) begin
			
			#5 SpiClk = ~SpiClk;
		
		end
		
		#20
		SpiCs 			= 1;
		
		#20
		SpiCs 			= 0;
	
		#200
		
		for(i = 0; i < (128*SPI_DATA_RCNT + SPI_WCMD)*2; i = i + 1) begin
			
			#5 SpiClk = ~SpiClk;
		
		end
		
		#20
		SpiCs 			= 1;
		
		#5000
		
	
		$stop;
	end

endmodule