/* Period limiter timer and start pulse control. */
module PeriodLimiter (

/* Input clk 100MHz. */
input clk,
input rst,
input StartPulse,
input [31:0] CmdWord,
input [31:0] PeriodLimit,
output reg [0:0] OutStartPulse

);

/* Parameters. */
parameter SYNC_BIT 		= 30;

/* Default limit is about 1 second. */
//parameter LIMIT_TIME_100MHz 	= 32'd100_000_000;
parameter LIMIT_TIME_100MHz 	= 32'd50_000;
/* Length is 10 clocks - 10*10nS = 100nS. */
parameter PULSE_LENGTH 			= 32'd10;


/* Registers. */
reg [31:0] PLimMaxCnt 	= LIMIT_TIME_100MHz;
reg [31:0] TimerLimiter = {32{1'b0}};
reg [0:0] StartDetLock	= 1'b0;


/* Reg update. */
always @(posedge clk) begin

	if(rst) begin
		PLimMaxCnt 		<= LIMIT_TIME_100MHz;
		TimerLimiter 	<= {32{1'b0}};
		OutStartPulse 	<= 1'b0;
		StartDetLock	<= 1'b0;
	end
	else begin
	
		if(CmdWord[SYNC_BIT]) begin
			PLimMaxCnt <= PeriodLimit;
		end
		else begin	
		
			if(StartPulse) begin
				if(!StartDetLock) begin
					OutStartPulse <= 1'b1;
					StartDetLock <= 1'b1;
				end
			end

			if(TimerLimiter == PLimMaxCnt - 1'b1) begin
				StartDetLock <= 1'b0;
				TimerLimiter <= {32{1'b0}};
			end			
			else if(StartDetLock) begin		
				TimerLimiter <= TimerLimiter + 1'b1;	
			end
			
			if(TimerLimiter == PULSE_LENGTH - 1'b1) begin
				OutStartPulse <= 1'b0;
			end
			
			
		end
	end

end

endmodule /* END OF MODULE */