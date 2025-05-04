/* Fifo control logic. */
module FifoControl
(



/* Input clock to write data to the FIFO. */
input wrclk,
/* Input clk 100MHz. */
input clk,
input rst,
input fifo_srst,
input SpiClk,
input SpiCs,
input SpiMosi,
input FAcqStart,
input SAcqStart,
input [31:0] CmdWord,
input [31:0] FTimStopValue,
input [31:0] STimStopValue,
input [31:0] Thr0StopValue,
input [31:0] Thr1StopValue,
input [128 - 1:0] DataIn,
input [31:0] SpiCmd,
input SpiCmdLock,

output wire [0:0] Buff0Full,
output wire [0:0] Buff1Full,
output reg [0:0] SpiMiso


);

/* Parameters. */
parameter SPI_DUMMY_MSB		= 31;
parameter SPI_DUMMY_LSB 	= 16;
parameter SPI_RW		 	= 15;
parameter SPI_DC_MSB	 	= 14;
parameter SPI_DC_LSB	 	= 11;
parameter SPI_RSVD_MSB 		= 10;
parameter SPI_RSVD_LSB 		= 5;
parameter SPI_ADDR_MSB 		= 4;
parameter SPI_ADDR_LSB 		= 0;
	
parameter SPI_READ			= 1'b1;
parameter SPI_WRITE			= 1'b0;
parameter SPI_READ_BACK		= 4'b0001;
parameter SPI_COMMAND		= 4'b0010;
parameter SPI_SAMPLECNT		= 4'b0011;
parameter SPI_SCNTTEST		= 4'b0100;
parameter SPI_ADCRTEST		= 4'b0101;
parameter SPI_RFIFO0		= 4'b0110;
parameter SPI_RFIFO1		= 4'b0111;
parameter SPI_RPIPE			= 4'b1000;
parameter SPI_RSVD			= {6{1'b0}};
parameter SPI_DEF_ADDR		= {5{1'b0}};

parameter START_SEL_BIT 	= 0;
parameter STOP_SEL_BIT		= 1;
parameter THR_START_CH 		= 2;
parameter THR_START_HL		= 3;
parameter PRG_START_BIT		= 4;
parameter SYNC_BIT 			= 30;
parameter RST_BIT 			= 31;

parameter FTIM_STOP_VAL		= 32'd2000;
parameter STIM_STOP_VAL		= 32'd2000;
parameter THR0_STOP_VAL		= 32'd256;
parameter THR1_STOP_VAL		= 32'd256;
parameter FIFO_FULL_WIDTH	= 8'd10;

parameter SPI_CLK_CNTWIDTH 	= 32;
//parameter DATA_IN_WIDTH 	= 128;
parameter FIFO_OUT_WIDTH 	= 128;
parameter DATA_OUT_WIDTH 	= 128;

/* Internal connection. */

/* FIFO write full flags. WFull[3:0] - FIFO3..0. */
wire [3:0] WFull;

/* FIFO read empty flags. REmpty[3:0] - FIFO3..0. */
wire [3:0] REmpty;

/* FIFO write request signals. WREQ[3:0] - FIFO3..0. */
reg [3:0] WReq 				= 0;

/* FIFO read request signals. RREQ[3:0] - FIFO3..0. */
reg [3:0] RReq 				= 0;

/* FIFO pipe read. */
integer i;

/* FIFO buses. */
wire Fifo0Rst;
wire Fifo1Rst;
wire [10:0] WRUseFifo0;
wire [10:0] WRUseFifo1;
wire [FIFO_OUT_WIDTH - 1:0] Fifo0Out;
wire [FIFO_OUT_WIDTH - 1:0] Fifo1Out;
reg [DATA_OUT_WIDTH - 1:0] SpiData;

/* Registers. */
//reg PipeClkAct				= 1'b0;

reg [0:0] RClk0 			= 1'b0;
reg [0:0] RClk1 			= 1'b0;

reg [SPI_CLK_CNTWIDTH - 1:0] SpiClkCnt	= {SPI_CLK_CNTWIDTH{1'b0}};

reg [7:0] RSpiClkCnt = 8'd0;

reg [31:0] FTimStopCnt 		= {32{1'b0}};
reg [31:0] STimStopCnt 		= {32{1'b0}};
reg [15:0] FTimWReqCnt 		= {15{1'b0}};
reg [15:0] STimWReqCnt 		= {15{1'b0}};
reg [31:0] FTimStopVal		= FTIM_STOP_VAL;
reg [31:0] STimStopVal		= STIM_STOP_VAL;
reg [31:0] FifoOutSample	= {32{1'b0}};
reg [7:0] Fifo0FullCnt		= {8{1'b0}};
reg [7:0] Fifo1FullCnt		= {8{1'b0}};
/* One data block is four 16 bit samples (128  bit width). */
reg [31:0] Thr0StopVal		= THR0_STOP_VAL;
reg [31:0] Thr1StopVal		= THR1_STOP_VAL;
reg [0:0] Fifo0Full 			= 1'b0;
reg [0:0] Fifo1Full			= 1'b0;
reg [0:0] FifoSampleAct 	= 1'b0;
reg [0:0] DataOutAct			= 1'b0; 
reg [0:0] DummyLock			= 1'b0;

reg [31:0] SpiCmdWord;


/* FIFO instances create. */
FIFO_IO FIFO_IO_inst0(	.aclr(rst | fifo_srst), 
								.data(DataIn),
								.rdclk(RClk0),
								.rdreq(RReq[0]),
								.wrclk(wrclk),
								.wrreq(WReq[0]),
								.q(Fifo0Out),
								.rdempty(REmpty[0]),
								//.rdfull(),
								//.rdusedw(),
								//.wrempty(),
								.wrfull(WFull[0]),
								.wrusedw(WRUseFifo0)
							);


FIFO_IO FIFO_IO_inst1(	.aclr(rst | fifo_srst), 
								.data(DataIn),
								.rdclk(RClk1),
								.rdreq(RReq[1]),
								.wrclk(wrclk),
								.wrreq(WReq[1]),
								.q(Fifo1Out),
								.rdempty(REmpty[1]),
								//.rdfull(),
								//.rdusedw(),
								//.wrempty(),
								.wrfull(WFull[1]),
								.wrusedw(WRUseFifo1)
							);
							
						
/* FIFO write reg update logic. */
always @(posedge wrclk) begin

	if(rst) begin
		FTimStopVal 	<= FTIM_STOP_VAL;
		STimStopVal 	<= STIM_STOP_VAL;
		Thr0StopVal 	<= THR0_STOP_VAL;
		Thr1StopVal 	<= THR1_STOP_VAL;
		FTimStopCnt 	<= {32{1'b0}};
		STimStopCnt 	<= {32{1'b0}};
		FTimWReqCnt 	<= {16{1'b0}};
		STimWReqCnt 	<= {16{1'b0}};
		Fifo0FullCnt	<= {8{1'b0}};
		Fifo1FullCnt	<= {8{1'b0}};
		Fifo0Full 		<= 1'b0;
		Fifo1Full 		<= 1'b0;
		WReq				<= {4{1'b0}};
	end
	else begin	
	
		if(CmdWord[SYNC_BIT]) begin
			FTimStopVal <= FTimStopValue;
			STimStopVal <= STimStopValue;
			Thr0StopVal <= Thr0StopValue;
			Thr1StopVal <= Thr1StopValue;
		end
		else begin	
		
			if(FAcqStart & !Fifo0Full) begin 
				WReq[0] <= 1'b1;
			end
			
			if(SAcqStart & !Fifo1Full) begin 
				WReq[1] <= 1'b1;
			end

			if(CmdWord[STOP_SEL_BIT]) begin
			
				if(FTimStopCnt == FTimStopVal - 1'd1) begin
					FTimStopCnt <= {32{1'b0}};
					WReq[0] <= 1'b0;
					Fifo0Full <= 1'b1;
				end
				else if(WReq[0]) begin
					FTimStopCnt <= FTimStopCnt + 1'd1;	
				end
			
			
				if(STimStopCnt == STimStopVal - 1'd1) begin
					STimStopCnt <= {32{1'b0}};
					WReq[1] <= 1'b0;
					Fifo1Full <= 1'b1;
				end
				else if(WReq[1]) begin
					STimStopCnt <= STimStopCnt + 1'd1;	
				end
			end
			else begin
			
				if(WReq[0]) begin
					if(FTimWReqCnt == Thr0StopVal - 1'd1) begin
						WReq[0] <= 1'b0;
						FTimWReqCnt <= {16{1'b0}};
						Fifo0Full <= 1'b1;
					end
					else begin
						FTimWReqCnt <= FTimWReqCnt + 1'd1;
					end
				end	
				
				if(WReq[1]) begin
					if(STimWReqCnt == Thr1StopVal - 1'd1) begin
						WReq[1] <= 1'b0;
						STimWReqCnt <= {16{1'b0}};
						Fifo1Full <= 1'b1;
					end
					else begin
						STimWReqCnt <= STimWReqCnt + 1'd1;
					end
				end	
			
			end

			
			/* FIFO full flag reset. */
			if(Fifo0FullCnt == FIFO_FULL_WIDTH - 1'd1) begin
				Fifo0FullCnt <= {8{1'b0}};
				Fifo0Full <= 1'b0;
			end
			else if(Fifo0Full) begin 
				Fifo0FullCnt <= Fifo0FullCnt + 1'd1;		
			end
			
			if(Fifo1FullCnt == FIFO_FULL_WIDTH - 1'd1) begin
				Fifo1FullCnt <= {8{1'b0}};
				Fifo1Full <= 1'b0;
			end
			else if(Fifo1Full) begin 
				Fifo1FullCnt <= Fifo1FullCnt + 1'd1;		
			end

		end		
	end

end

/* FIFO read logic. */
always @(negedge SpiClk or posedge SpiCs) begin

	if(SpiCs) begin
			SpiClkCnt 		<= {SPI_CLK_CNTWIDTH{1'b0}};
			SpiData 		<= {DATA_OUT_WIDTH{1'b0}};
			//SpiData 			<= {8{1'b0}};
			FifoOutSample 	<= {32{1'b0}};
			FifoSampleAct 	<= 1'b0;
			DataOutAct 		<= 1'b0;
			DummyLock		<= 1'b0;
			RReq			<= {4{1'b0}};
			RClk0 			<= 1'b0;
			RClk1 			<= 1'b0;
	end
	else begin
		
		
		if(SpiClkCnt == 32'hFFFFFFFF) begin
			SpiClkCnt <= {32{1'b0}};
		end
		else begin
			SpiClkCnt <= SpiClkCnt + 32'd1;
		end
			
		/* 1 clock ahead to set the output data bit. */
		if(FifoSampleAct) begin
			SpiMiso <= FifoOutSample[31];
			FifoOutSample[31:1] <= FifoOutSample[30:0];
			FifoOutSample[0] <= 1'b0;
			if(SpiClkCnt == 32'd30) begin
				SpiClkCnt <= {32{1'b0}};
				FifoSampleAct <= 1'b0;
			end
		end
//		{ SpiMiso, SpiData[127:0] } <= { SpiData[127:0], 1'b0 };
		if(DataOutAct) begin
			SpiMiso <= SpiData[127];
			SpiData[127:1] <= SpiData[126:0];
			SpiData[0] <= 1'b0;

			if(SpiClkCnt == 32'd125) begin			
			
				if(SpiCmd[SPI_DC_MSB:SPI_DC_LSB] == SPI_RFIFO0) begin
					if(WRUseFifo0[9:0] > 0) begin
						RClk0 <= 1;
					end
				end
				else if(SpiCmd[SPI_DC_MSB:SPI_DC_LSB] == SPI_RFIFO1) begin
					if(WRUseFifo1[9:0] > 0) begin
						RClk1 <= 1;
					end
				end
				
			end
			
			if(SpiClkCnt == 32'd126) begin
			
				if(SpiCmd[SPI_DC_MSB:SPI_DC_LSB] == SPI_ADCRTEST) begin					
					SpiData <= { 32'hABCDEFAB, 32'hABCDEFAB, 32'hABCDEFAB, 32'hABCDEFAB };
				end
				else if(SpiCmd[SPI_DC_MSB:SPI_DC_LSB] == SPI_RFIFO0) begin
					if(WRUseFifo0[9:0] > 0) begin
						RClk0 <= 0;
						SpiData <= Fifo0Out;
					end
					else begin
						SpiData <= 0;
					end
				end
				else if(SpiCmd[SPI_DC_MSB:SPI_DC_LSB] == SPI_RFIFO1) begin
					if(WRUseFifo1[9:0] > 0) begin
						RClk1 <= 0;
						SpiData <= Fifo1Out;
					end
					else begin
						SpiData <= 0;
					end
				end
				
			end
			
			if(SpiClkCnt == 32'd127) begin							
				SpiClkCnt <= {32{1'b0}};				
			end
		end	
		
		/* Dummy 32bit word. 64-th bit to shift to output bit FifoOutSample[31] . */
		if(SpiCmdLock & !DummyLock) begin
			
			if(SpiCmd[SPI_RW] == SPI_READ & SpiCmd[SPI_DC_MSB:SPI_DC_LSB] == SPI_RPIPE) begin				
				
				for(i = 0; i < 8; i = i + 1) begin				
					if(SpiClkCnt == (32'd46 + i)) begin					
						RClk0 <= ~RClk0;
						RClk1 <= ~RClk1;
					end			
				end
				
			end 
			else if(SpiCmd[SPI_RW] == SPI_READ & SpiCmd[SPI_DC_MSB:SPI_DC_LSB] == SPI_RFIFO0) begin
				
				if(SpiClkCnt == 32'd60) begin
					RReq[0] <= 1;				
				end
				
				if(SpiClkCnt == 32'd61) begin
					RClk0 <= 1;
				end
				
				if(SpiClkCnt == 32'd62) begin
					RClk0 <= 0;
					SpiData <= Fifo0Out;
					DataOutAct <= 1'b1;
				end
			end
			else if(SpiCmd[SPI_RW] == SPI_READ & SpiCmd[SPI_DC_MSB:SPI_DC_LSB] == SPI_RFIFO1) begin
				
				if(SpiClkCnt == 32'd60) begin
					RReq[1] <= 1;				
				end
			
				if(SpiClkCnt == 32'd61) begin
					RClk1 <= 1;
				end
				
				if(SpiClkCnt == 32'd62) begin
					RClk1 <= 0;
					SpiData <= Fifo1Out;
					DataOutAct <= 1'b1;
				end
			end
			else if(SpiCmd[SPI_RW] == SPI_READ & SpiCmd[SPI_DC_MSB:SPI_DC_LSB] == SPI_SAMPLECNT) begin
				if(SpiClkCnt == 32'd62) begin
					FifoOutSample <= { {5{1'b0}}, WRUseFifo1, {5{1'b0}}, WRUseFifo0 };
					FifoSampleAct <= 1'b1;
				end			
			end
			else if(SpiCmd[SPI_RW] == SPI_READ & SpiCmd[SPI_DC_MSB:SPI_DC_LSB] == SPI_SCNTTEST) begin
				if(SpiClkCnt == 32'd62) begin
					FifoOutSample <= { 8'hAB, 8'hCD, 8'hEF, 8'hAB };
					FifoSampleAct <= 1'b1;
				end			
			end
			else if(SpiCmd[SPI_RW] == SPI_READ & SpiCmd[SPI_DC_MSB:SPI_DC_LSB] == SPI_ADCRTEST) begin
				if(SpiClkCnt == 32'd62) begin
					SpiData <= { 32'hABCDEFAB, 32'hABCDEFAB, 32'hABCDEFAB, 32'hABCDEFAB };
					DataOutAct <= 1'b1;
				end			
			end
			
			if(SpiClkCnt == 32'd63) begin
				SpiClkCnt <= {32{1'b0}};
				DummyLock <= 1'b1;
			end
		end
		
	end

end

/* WFull states. */
assign Buff0Full = WFull[0] | Fifo0Full;
assign Buff1Full = WFull[1] | Fifo1Full;


endmodule 