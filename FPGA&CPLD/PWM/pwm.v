module pwm
(
	input wire refClk,		///<ref for in clk 800KHz, t = 1250uS
	input wire enDC,
	input wire writePeriod,
	//input wire writeDuty,
	input wire [15:0] data,
	output reg outPwm,
	output wire outEventCnt

);

reg [15:0] period = 0;
reg [15:0] periodCnt = 0;

reg [15:0] duty = 0;
reg [15:0] dutyCnt = 0;

reg [1:0] sh_writePeriod;




///*************************************************************************
///shift reg for period strob
always @(posedge refClk) begin

sh_writePeriod <= { sh_writePeriod[0], writePeriod };

end

wire strobWritePeriod = (sh_writePeriod == 2'b01);


///*************************************************************************
///period counter 
always @(posedge refClk) begin

	if(strobWritePeriod && enDC) begin 
		period <= data; 
		periodCnt <= 0;  
		end
	else if(periodCnt == period) begin 
				periodCnt <= 0; 
				end
			else begin 
				periodCnt <= periodCnt + 1'b1; 
				end

end

wire strobEventRSTPeriodCnt = (periodCnt == 0);
wire strobEventReloadDuty = (periodCnt == period - 10);



///*************************************************************************
///duty counter 
always @(posedge refClk) begin

	if(strobEventReloadDuty && !enDC) begin duty <= data; end

end


///*************************************************************************
///
always @(posedge refClk) begin

	if(strobEventRSTPeriodCnt) begin 
		outPwm <= 1'b1; 
		end	
	else if(dutyCnt == duty) begin 
				dutyCnt <= 0; 
				outPwm <= 1'b0; 
				end
			else begin 
				dutyCnt <= dutyCnt + 1'b1; 
				end
end

assign outEventCnt = (periodCnt == period - 10);

endmodule