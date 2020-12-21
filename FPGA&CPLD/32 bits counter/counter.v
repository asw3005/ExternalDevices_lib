///@brief Module counter
module cntPixelEvent_v2
(
	input wire refClock,				///<clock signal for sinhronisation
	input wire readDataClock,		///<clock signal for read counter data
	input wire xChannel,				///< X channel input signal
	input wire yChannel,				///< Y channel input signal
	input wire rstCounter,			///<reset counter signal
	input wire enCounter,			///<enable counter signal
	output reg [31:0] cntOutValue	///<output counter's register
);



reg [31:0] cntVal;
reg [1:0] x_sh ;
reg [1:0] strob_sh ;
reg [2:0] out_sh;


//**********************************************************************************************************************************************
///shift reg for create events(extra sinhronisation)
always @(posedge refClock) begin

x_sh <= { x_sh[1:0], xChannel };
strob_sh <= { strob_sh[1:0], ((x_sh == 2'b01) & yChannel) };

end

//**********************************************************************************************************************************************
///shift reg for create event read counter(extra sinhronisation)
always @(posedge refClock) begin

//out_sh <= { out_sh[1:0], readDataClock };///<?
out_sh <= { out_sh[2:0], readDataClock };///<?			11.12.20

end

//**********************************************************************************************************************************************
//
wire strob_x = (x_sh == 2'b01);			///<event come signal X channel
wire out_strob = (out_sh == 3'b011);	///<event come signal read counter data

//**********************************************************************************************************************************************
///counter implementetion
always @(posedge refClock or posedge rstCounter) begin


if(rstCounter) begin cntVal <= 0; end

else 
	if(!enCounter) 
	begin
			if (strob_sh == 2'b01) 
				begin 
				cntVal <= cntVal + 1'b1; 
				end
	end

end


//**********************************************************************************************************************************************
///read counter data implementation
always@(posedge refClock)
begin

if(out_strob) cntOutValue <= cntVal;

end

endmodule