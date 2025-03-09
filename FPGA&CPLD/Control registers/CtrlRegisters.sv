/*  */
module CtrlRegisters (

/* Input clk 100MHz. */
input clk,
input rst,
input SpiClk,
input SpiCs,
input SpiMosi,
input DataSync,
output wire SpiCmdLock,
output wire [31:0] SpiCmd,
output reg [15:0][31:0] DataReg

);

/* Parameters. */
parameter SPI_DUMMY_MSB	= 31;
parameter SPI_DUMMY_LSB = 16;
parameter SPI_RW		 	= 15;
parameter SPI_DC_MSB	 	= 14;
parameter SPI_DC_LSB	 	= 11;
parameter SPI_RSVD_MSB 	= 10;
parameter SPI_RSVD_LSB 	= 5;
parameter SPI_ADDR_MSB 	= 4;
parameter SPI_ADDR_LSB 	= 0;

parameter SPI_READ		= 1'b1;
parameter SPI_WRITE		= 1'b0;
parameter SPI_DATA		= 4'b0001;
parameter SPI_COMMAND	= 4'b0010;
parameter SPI_RSVD		= {6{1'b0}};
parameter SPI_DEF_ADDR	= {5{1'b0}};
parameter SPI_DUMMY		= {16{1'b0}};

parameter START_SEL_BIT = 0;
parameter STOP_SEL_BIT	= 1;
parameter THR_START_CH 	= 2;
parameter THR_START_HL	= 3;
parameter PRG_START_BIT	= 4;
parameter SYNC_BIT 		= 30;
parameter RST_BIT 		= 31;

parameter INT_ADDR_MAX			= 8'd10;
parameter SYNC_RST_DELAY 		= 5'd31;
parameter PRG_START_RST_DELAY = 5'd31;
//parameter LIMIT_TIME_100MHz 	= 32'd100_000_000;
parameter LIMIT_TIME_100MHz 	= 32'd50_000;
parameter FAST_LOCK_TIME 		= 8'd15;

/* Registers. */
parameter ControlReg 	= 0;
parameter PeriodLimiter = 1;
parameter FStartTim		= 2;
parameter SStartTim		= 3;
parameter FStopTim		= 4;
parameter SStopTim		= 5;
parameter ThrStartHigh	= 6;
parameter ThrStartLow	= 7;
parameter Thr0Stop		= 8;
parameter Thr1Stop		= 9;

reg [15:0][31:0] RegMap;
reg [31:0] SpiCmdWord 			= { SPI_DUMMY, SPI_READ, SPI_DATA, SPI_RSVD, SPI_DEF_ADDR };
reg [31:0] SpiData				= {32{1'b0}};
reg [7:0] SpiClkCnt				= {8{1'b0}};
reg [7:0] IntADDRCnt				= {8{1'b0}};
reg [4:0] SyncCnt					= {5{1'b0}};
reg [0:0] SyncCntActive 		= 1'b0;
reg [4:0] PrgStartCnt			= {5{1'b0}};
reg [0:0] PrgStartCntActive 	= 1'b0;
reg [0:0] CmdWordLock			= 1'b0;
reg [0:0] IntAddrCntLock 		= 1'b0;

reg [0:0] RWLock					= 1'b0;
reg [4:0] CmdLock					= 5'b0;

reg [0:0] DataWordLock			= 1'b0;
reg [7:0] FastLockCnt			= 8'd0;

/* Reg update. */
always @(posedge clk) begin

	if(rst) begin
		DataReg[ControlReg] 		<= {32{1'b0}};
		DataReg[PeriodLimiter] 	<= LIMIT_TIME_100MHz;
		DataReg[FStartTim] 		<= 32'd10000;
		DataReg[SStartTim] 		<= 32'd12500;
		DataReg[FStopTim] 		<= 32'd2000;
		DataReg[SStopTim] 		<= 32'd2000;
		DataReg[ThrStartHigh] 	<= 32'd3072;
		DataReg[ThrStartLow] 	<= 32'd1024;
		DataReg[Thr0Stop] 		<= 32'd256;
		DataReg[Thr1Stop] 		<= 32'd256;
		DataReg[10] 				<= { {28{1'b0}}, 4'hA };
		DataReg[11] 				<= { {28{1'b0}}, 4'hB };
		DataReg[12] 				<= { {28{1'b0}}, 4'hC };
		DataReg[13] 				<= { {28{1'b0}}, 4'hD };
		DataReg[14] 				<= { {28{1'b0}}, 4'hE };
		DataReg[15] 				<= { {28{1'b0}}, 4'hF };	
		SyncCnt						<= {5{1'b0}};
		SyncCntActive 				<= 1'b0;
		PrgStartCnt					<= {5{1'b0}};
		PrgStartCntActive 		<= 1'b0;
		FastLockCnt 				<= 8'd0;
	end
	else begin
	
			if(DataWordLock) begin
				if(SpiCmdWord[SPI_RW] == SPI_WRITE & SpiCmdWord[SPI_DC_MSB:SPI_DC_LSB] == SPI_COMMAND & FastLockCnt == 8'd15) begin
					RegMap[SpiCmdWord[SPI_ADDR_MSB:SPI_ADDR_LSB]] <= { SpiData[15:0], SpiData[31:16] };
				end
				FastLockCnt <= FastLockCnt + 8'd1;
			end
			else if(SpiCs) begin
				FastLockCnt <= 8'd0;
			end
			
	
			if(RegMap[ControlReg][SYNC_BIT]) begin
				DataReg <= RegMap;				
				SyncCntActive <= 1'b1;
			end
			
			if(SyncCnt == SYNC_RST_DELAY - 5'd1) begin
				SyncCnt <= {5{1'b0}};
				SyncCntActive <= 1'b0;
				RegMap[ControlReg][SYNC_BIT]	<= 1'b0;
				DataReg[ControlReg][SYNC_BIT] <= 1'b0;

			end
			else if(SyncCntActive) begin
				SyncCnt <= SyncCnt + 5'd1;
			end			
			
			
			if(DataReg[ControlReg][PRG_START_BIT]) begin
				PrgStartCntActive <= 1'b1;
			end
			
			if(PrgStartCnt == PRG_START_RST_DELAY - 5'd1) begin
				PrgStartCnt <= {5{1'b0}};
				PrgStartCntActive <= 1'b0;
				RegMap[ControlReg][PRG_START_BIT]	<= 1'b0;
				DataReg[ControlReg][PRG_START_BIT] <= 1'b0;

			end
			else if(PrgStartCntActive) begin
				PrgStartCnt <= PrgStartCnt + 5'd1;
			end
		
	end

end

always @(posedge SpiClk or posedge SpiCs) begin

	if(SpiCs) begin
		SpiClkCnt 		<= {8{1'b0}};
		IntADDRCnt 		<= {8{1'b0}};
		SpiCmdWord 		<= { SPI_DUMMY, SPI_READ, SPI_DATA, SPI_RSVD, SPI_DEF_ADDR };
		SpiData 			<= {32{1'b0}};
		CmdWordLock		<= 1'b0;
		IntAddrCntLock <= 1'b0;
		DataWordLock   <= 1'b0;
	end
	else begin
	
		
		if(!CmdWordLock) begin
			SpiCmdWord[0] <= SpiMosi;
			SpiCmdWord[31:1] <= SpiCmdWord[30:0];
		end
	
		if(!DataWordLock) begin
			SpiData[0] <= SpiMosi;
			SpiData[31:1] <= SpiData[30:0];
		end
	
		if(SpiClkCnt == 8'd31) begin
			SpiClkCnt <= {8{1'b0}};
		end
		else begin
			SpiClkCnt <= SpiClkCnt + 8'd1;
		end
	
		if(SpiClkCnt == 8'd31) begin
			if(!CmdWordLock) begin
				CmdWordLock	<= 1'b1;
			end		
		end
		
		if(SpiClkCnt == 8'd31) begin		
			if(CmdWordLock & !DataWordLock) begin
				DataWordLock	<= 1'b1;
			end				
		end

	end

end 

assign 	SpiCmd = SpiCmdWord,
			SpiCmdLock = CmdWordLock;

endmodule /* END OF MODULE */