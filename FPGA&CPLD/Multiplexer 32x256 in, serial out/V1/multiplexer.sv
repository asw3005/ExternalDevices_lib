///@brief Module data selector(big multiplexor)
module dataSelector_sv
(
	input wire refClock,							///<clock signal for sinhronisation

	input wire clkReadData,						///<clock for read input data	
	output wire outReadData,					///<output data internal shift's register
	
	input wire latchInputData,					///<clock for transfer input data to output shift's register
	input wire clkSelectorData,				///<clock for configuration selector
	input wire inSelectorData,					///<input selector's data

	input wire [255:0][31:0] inData 			///<massive inputs for data
);

parameter bitNumberInputData = 8;

reg [bitNumberInputData - 1:0] selConfig;
reg [31:0] capturedInData;
reg [31:0] shiftDataToOut;
reg [1:0] sh_clkReadData;
reg [1:0] sh_clkSelectorData;
reg [1:0] sh_latchInputData;

integer i;

//*************************************************************************
///shift reg for create events(extra sinhronisation)
always @(posedge refClock) begin

sh_clkReadData <= { sh_clkReadData[0], clkReadData };
sh_clkSelectorData <= { sh_clkSelectorData[0], clkSelectorData };
sh_latchInputData <= { sh_latchInputData[0], latchInputData };

end


wire eventSelectorData = (sh_clkSelectorData == 1'b01);

//*************************************************************************
///internal selector's register implementation
always @(posedge refClock)
begin

	if(eventSelectorData) begin
	selConfig[0] <= inSelectorData;
	selConfig[7:1] <= selConfig[6:0];			//MSB first
	end
	
end


wire eventLatchInputData = (sh_latchInputData == 1'b01);	///<event read input data
wire eventReadData = (sh_clkReadData == 1'b01);				///<event shift internal data to output 

//*************************************************************************
///
//always @(posedge refClock)
//begin
//
//	if(eventLatchInputData) begin
//		capturedInData[31:0] = inData[0];
//		for(i = 0; i < 256; i++) 
//		begin	
//			if(selConfig == i) begin capturedInData[31:0] = inData[i]; end	
//			//else if(selConfig = 256) begin capturedInData[31:0] <= inData[0]; end
//		end
//	end
//	
//	if(eventReadData) begin
//		outReadData <= capturedInData[31];
//		capturedInData[31:1] <= capturedInData[30:0];
//	end
//
//	
//end

//*************************************************************************
///multiplexer implementation
always @(posedge refClock)
begin

	if(eventLatchInputData) begin
		capturedInData <= inData[selConfig];		
	end 
	else if(eventReadData) begin
				{ outReadData, capturedInData[31:0] } <= { capturedInData[31:0], 1'b0 }; //MSB first
				//outReadData <= capturedInData[31];
				//capturedInData[31:1] <= capturedInData[30:0];
				end
					
end

endmodule