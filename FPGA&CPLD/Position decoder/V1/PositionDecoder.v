//*****************************************************************************************
//Decoder

module DECODER_AB												
(
input wire clock,
input wire [15:0] InputA,						// Inputs grup A
input wire [15:0] InputB,						// Inputs grup B
output reg[4:0] OutputA,						// Outputss grup A
output reg[4:0] OutputB							// Outputss grup B
);

reg [15:0] A_t, B_t;

always @(posedge clock)   begin				
	A_t <= InputA;
	B_t <= InputB;
end

always @(*)							
   begin
   
	case(A_t)														
			16'b0000000000000000: OutputA <= 5'd0; 	
			16'b0000000000000001: OutputA <= 5'd1; 	
			16'b0000000000000010: OutputA <= 5'd2; 	
			16'b0000000000000100: OutputA <= 5'd3; 	
			16'b0000000000001000: OutputA <= 5'd4; 	
			16'b0000000000010000: OutputA <= 5'd5; 	
			16'b0000000000100000: OutputA <= 5'd6; 	
			16'b0000000001000000: OutputA <= 5'd7; 	
			16'b0000000010000000: OutputA <= 5'd8; 	
			16'b0000000100000000: OutputA <= 5'd9; 	
			16'b0000001000000000: OutputA <= 5'd10; 
			16'b0000010000000000: OutputA <= 5'd11; 
			16'b0000100000000000: OutputA <= 5'd12; 
			16'b0001000000000000: OutputA <= 5'd13; 
			16'b0010000000000000: OutputA <= 5'd14; 
			16'b0100000000000000: OutputA <= 5'd15; 
			16'b1000000000000000: OutputA <= 5'd16; 
			default: OutputA <= 5'd17; 				
	endcase
   end
	
always @(*)												
   begin
	case(B_t)										
			16'b0000000000000000: OutputB <= 5'd0; 	
			16'b0000000000000001: OutputB <= 5'd1; 	
			16'b0000000000000010: OutputB <= 5'd2; 	
			16'b0000000000000100: OutputB <= 5'd3; 	
			16'b0000000000001000: OutputB <= 5'd4; 	
			16'b0000000000010000: OutputB <= 5'd5; 	
			16'b0000000000100000: OutputB <= 5'd6; 	
			16'b0000000001000000: OutputB <= 5'd7; 	
			16'b0000000010000000: OutputB <= 5'd8; 	
			16'b0000000100000000: OutputB <= 5'd9; 	
			16'b0000001000000000: OutputB <= 5'd10; 
			16'b0000010000000000: OutputB <= 5'd11; 
			16'b0000100000000000: OutputB <= 5'd12; 
			16'b0001000000000000: OutputB <= 5'd13; 
			16'b0010000000000000: OutputB <= 5'd14; 
			16'b0100000000000000: OutputB <= 5'd15; 
			16'b1000000000000000: OutputB <= 5'd16; 
			default: OutputB <= 5'd17; 				
	endcase
   end

endmodule