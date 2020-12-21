//*****************************************************************************************
//Pulse shift 5nS

module PulseShift_5nS								// Название модуля
(
input wire [9:0] InputDig,					//
input wire ShiftClock,						//
input wire inDataClock,
input wire inData,

output reg [9:0] OutDig						//  

);

reg [9:0] DelaySelect; 
//reg [4:0] DelaySelectY; 

reg [31:0] InputX1;
reg [31:0] InputX2;
reg [31:0] InputX3;
reg [31:0] InputX4;
reg [31:0] InputX5;

reg [31:0] InputY1;
reg [31:0] InputY2;
reg [31:0] InputY3;
reg [31:0] InputY4;
reg [31:0] InputY5;

always @(posedge inDataClock)

	begin
	
	DelaySelect [9] <= inData;
	DelaySelect [8:0] <= DelaySelect [9:1];		
	
	end

always @(posedge ShiftClock)

begin

InputX1 [31] <= InputDig [9];
InputX1 [30:0] <= InputX1 [31:1];

InputX2 [31] <= InputDig [8];
InputX2 [30:0] <= InputX2 [31:1];

InputX3 [31] <= InputDig [7];
InputX3 [30:0] <= InputX3 [31:1];

InputX4 [31] <= InputDig [6];
InputX4 [30:0] <= InputX4 [31:1];

InputX5 [31] <= InputDig [5];
InputX5 [30:0] <= InputX5 [31:1];

InputY1 [31] <= InputDig [4];
InputY1 [30:0] <= InputY1 [31:1];

InputY2 [31] <= InputDig [3];
InputY2 [30:0] <= InputY2 [31:1];

InputY3 [31] <= InputDig [2];
InputY3 [30:0] <= InputY3 [31:1];

InputY4 [31] <= InputDig [1];
InputY4 [30:0] <= InputY4 [31:1];

InputY5 [31] <= InputDig [0];
InputY5 [30:0] <= InputY5 [31:1];

end

//**********************************************************************************************************************************************
//always @(posedge Shift)
always @(*)
   begin
	case(DelaySelect [4:0])
	
				/*5'b00000:begin
            				  OutDig [9:5] <= InputDig[9:5]; 						
            				  end*/
				5'b00001:begin
            				  OutDig [9:5] <= {InputX1 [31], InputX2 [31], InputX3 [31], InputX4 [31], InputX5 [31]}; 
            				  end		
            5'b00010:begin
            				  OutDig [9:5] <= {InputX1 [30], InputX2 [30], InputX3 [30], InputX4 [30], InputX5 [30]}; 
            				  end		
            5'b00011:begin
            				  OutDig [9:5] <= {InputX1 [29], InputX2 [29], InputX3 [29], InputX4 [29], InputX5 [29]}; 
            				  end		
            5'b00100:begin
								  OutDig [9:5] <= {InputX1 [28], InputX2 [28], InputX3 [28], InputX4 [28], InputX5 [28]}; 
            				  end		
            5'b00101:begin
									OutDig [9:5] <= {InputX1 [27], InputX2 [27], InputX3 [27], InputX4 [27], InputX5 [27]}; 
            				  end		
            5'b00110:begin
									OutDig [9:5] <= {InputX1 [26], InputX2 [26], InputX3 [26], InputX4 [26], InputX5 [26]}; 
            				  end	
            5'b00111:begin									
									OutDig [9:5] <= {InputX1 [25], InputX2 [25], InputX3 [25], InputX4 [25], InputX5 [25]}; 
            				  end									 
            5'b01000:begin
									OutDig [9:5] <= {InputX1 [24], InputX2 [24], InputX3 [24], InputX4 [24], InputX5 [24]}; 
            				  end																							
            5'b01001:begin									
									OutDig [9:5] <= {InputX1 [23], InputX2 [23], InputX3 [23], InputX4 [23], InputX5 [23]}; 
            				  end
            5'b01010:begin									
									OutDig [9:5] <= {InputX1 [22], InputX2 [22], InputX3 [22], InputX4 [22], InputX5 [22]}; 
            				  end
            5'b01011:begin									
									OutDig [9:5] <= {InputX1 [21], InputX2 [21], InputX3 [21], InputX4 [21], InputX5 [21]}; 
            				  end
            5'b01100:begin									
									OutDig [9:5] <= {InputX1 [20], InputX2 [20], InputX3 [20], InputX4 [20], InputX5 [20]}; 
            			     end
            5'b01101:begin									
									OutDig [9:5] <= {InputX1 [19], InputX2 [19], InputX3 [19], InputX4 [19], InputX5 [19]}; 
            			     end 
            5'b01110:begin
									OutDig [9:5] <= {InputX1 [18], InputX2 [18], InputX3 [18], InputX4 [18], InputX5 [18]};
            			     end	
            5'b01111:begin
									OutDig [9:5] <= {InputX1 [17], InputX2 [17], InputX3 [17], InputX4 [17], InputX5 [17]};
            				  end	
            5'b10000:begin
									OutDig [9:5] <= {InputX1 [16], InputX2 [16], InputX3 [16], InputX4 [16], InputX5 [16]};
            			     end							
//****************************************************************	
            5'b10001:begin
									OutDig [9:5] <= {InputX1 [15], InputX2 [15], InputX3 [15], InputX4 [15], InputX5 [15]};
            				  end		
            5'b10010:begin
									OutDig [9:5] <= {InputX1 [14], InputX2 [14], InputX3 [14], InputX4 [14], InputX5 [14]};
            				  end		
            5'b10011:begin
									OutDig [9:5] <= {InputX1 [13], InputX2 [13], InputX3 [13], InputX4 [13], InputX5 [13]};
            				  end		
            5'b10100:begin
									OutDig [9:5] <= {InputX1 [12], InputX2 [12], InputX3 [12], InputX4 [12], InputX5 [12]};
            				  end		
            5'b10101:begin
									OutDig [9:5] <= {InputX1 [11], InputX2 [11], InputX3 [11], InputX4 [11], InputX5 [11]};
            				  end		
            5'b10110:begin
									OutDig [9:5] <= {InputX1 [10], InputX2 [10], InputX3 [10], InputX4 [10], InputX5 [10]};
            				  end	
            5'b10111:begin
									OutDig [9:5] <= {InputX1 [9], InputX2 [9], InputX3 [9], InputX4 [9], InputX5 [9]};
            				  end									 
            5'b11000:begin
									OutDig [9:5] <= {InputX1 [8], InputX2 [8], InputX3 [8], InputX4 [8], InputX5 [8]};
            				  end																							
            5'b11001:begin
									OutDig [9:5] <= {InputX1 [7], InputX2 [7], InputX3 [7], InputX4 [7], InputX5 [7]};
            				  end
            5'b11010:begin
									OutDig [9:5] <= {InputX1 [6], InputX2 [6], InputX3 [6], InputX4 [6], InputX5 [6]};
            				  end
            5'b11011:begin
									OutDig [9:5] <= {InputX1 [5], InputX2 [5], InputX3 [5], InputX4 [5], InputX5 [5]};
            				  end
            5'b11100:begin
									OutDig [9:5] <= {InputX1 [4], InputX2 [4], InputX3 [4], InputX4 [4], InputX5 [4]};
            			     end
            5'b11101:begin
									OutDig [9:5] <= {InputX1 [3], InputX2 [3], InputX3 [3], InputX4 [3], InputX5 [3]};
            			     end 
            5'b11110:begin
									OutDig [9:5] <= {InputX1 [2], InputX2 [2], InputX3 [2], InputX4 [2], InputX5 [2]};
            			     end	
            5'b11111:begin
									OutDig [9:5] <= {InputX1 [1], InputX2 [1], InputX3 [1], InputX4 [1], InputX5 [1]};
            				  end						
            default:begin           				
                          OutDig [9:5] <= InputDig[9:5]; 						
             			     end
	endcase
   end
	
//**********************************************************************************************************************************************
//always @(posedge Shift)
always @(*)
   begin
	case(DelaySelect [9:5])
	
				/*5'b00001:begin									
								  OutDig [4:0] <= InputDig[4:0]; 						
            				  end	*/
				5'b00001:begin
									OutDig [4:0] <= {InputY1 [31], InputY2 [31], InputY3 [31], InputY4 [31], InputY5 [31]};
            				  end		
            5'b00010:begin
									OutDig [4:0] <= {InputY1 [30], InputY2 [30], InputY3 [30], InputY4 [30], InputY5 [30]};
            				  end		
            5'b00011:begin
									OutDig [4:0] <= {InputY1 [29], InputY2 [29], InputY3 [29], InputY4 [29], InputY5 [29]};
            				  end		
            5'b00100:begin
									OutDig [4:0] <= {InputY1 [28], InputY2 [28], InputY3 [28], InputY4 [28], InputY5 [28]};
            				  end		
            5'b00101:begin
									OutDig [4:0] <= {InputY1 [27], InputY2 [27], InputY3 [27], InputY4 [27], InputY5 [27]};
            				  end		
            5'b00110:begin
									OutDig [4:0] <= {InputY1 [26], InputY2 [26], InputY3 [26], InputY4 [26], InputY5 [26]};
            				  end	
            5'b00111:begin
									OutDig [4:0] <= {InputY1 [25], InputY2 [25], InputY3 [25], InputY4 [25], InputY5 [25]};
            				  end									 
            5'b01000:begin
									OutDig [4:0] <= {InputY1 [24], InputY2 [24], InputY3 [24], InputY4 [24], InputY5 [24]};
            				  end																							
            5'b01001:begin
									OutDig [4:0] <= {InputY1 [23], InputY2 [23], InputY3 [23], InputY4 [23], InputY5 [23]};
            				  end
            5'b01010:begin
									OutDig [4:0] <= {InputY1 [22], InputY2 [22], InputY3 [22], InputY4 [22], InputY5 [22]};
            				  end
            5'b01011:begin
									OutDig [4:0] <= {InputY1 [21], InputY2 [21], InputY3 [21], InputY4 [21], InputY5 [21]};
            				  end
            5'b01100:begin
									OutDig [4:0] <= {InputY1 [20], InputY2 [20], InputY3 [20], InputY4 [20], InputY5 [20]};
            			     end
            5'b01101:begin
									OutDig [4:0] <= {InputY1 [19], InputY2 [19], InputY3 [19], InputY4 [19], InputY5 [19]};
            			     end 
            5'b01110:begin
									OutDig [4:0] <= {InputY1 [18], InputY2 [18], InputY3 [18], InputY4 [18], InputY5 [18]};
            			     end	
            5'b01111:begin
									OutDig [4:0] <= {InputY1 [17], InputY2 [17], InputY3 [17], InputY4 [17], InputY5 [17]};
            				  end	
            5'b10000:begin
									OutDig [4:0] <= {InputY1 [16], InputY2 [16], InputY3 [16], InputY4 [16], InputY5 [16]};
            			     end							
//****************************************************************	
            5'b10001:begin
									OutDig [4:0] <= {InputY1 [15], InputY2 [15], InputY3 [15], InputY4 [15], InputY5 [15]};
            				  end		
            5'b10010:begin
									OutDig [4:0] <= {InputY1 [14], InputY2 [14], InputY3 [14], InputY4 [14], InputY5 [14]};
            				  end		
            5'b10011:begin
									OutDig [4:0] <= {InputY1 [13], InputY2 [13], InputY3 [13], InputY4 [13], InputY5 [13]};
            				  end		
            5'b10100:begin
									OutDig [4:0] <= {InputY1 [12], InputY2 [12], InputY3 [12], InputY4 [12], InputY5 [12]};
            				  end		
            5'b10101:begin
									OutDig [4:0] <= {InputY1 [11], InputY2 [11], InputY3 [11], InputY4 [11], InputY5 [11]};
            				  end		
            5'b10110:begin
									OutDig [4:0] <= {InputY1 [10], InputY2 [10], InputY3 [10], InputY4 [10], InputY5 [10]};
            				  end	
            5'b10111:begin
									OutDig [4:0] <= {InputY1 [9], InputY2 [9], InputY3 [9], InputY4 [9], InputY5 [9]};
            				  end									 
            5'b11000:begin
									OutDig [4:0] <= {InputY1 [8], InputY2 [8], InputY3 [8], InputY4 [8], InputY5 [8]};
            				  end																							
            5'b11001:begin
									OutDig [4:0] <= {InputY1 [7], InputY2 [7], InputY3 [7], InputY4 [7], InputY5 [7]};
            				  end
            5'b11010:begin
									OutDig [4:0] <= {InputY1 [6], InputY2 [6], InputY3 [6], InputY4 [6], InputY5 [6]};
            				  end
            5'b11011:begin
									OutDig [4:0] <= {InputY1 [5], InputY2 [5], InputY3 [5], InputY4 [5], InputY5 [5]};
            				  end
            5'b11100:begin
									OutDig [4:0] <= {InputY1 [4], InputY2 [4], InputY3 [4], InputY4 [4], InputY5 [4]};
            			     end
            5'b11101:begin
									OutDig [4:0] <= {InputY1 [3], InputY2 [3], InputY3 [3], InputY4 [3], InputY5 [3]};
            			     end 
            5'b11110:begin
									OutDig [4:0] <= {InputY1 [2], InputY2 [2], InputY3 [2], InputY4 [2], InputY5 [2]};
            			     end	
            5'b11111:begin
									OutDig [4:0] <= {InputY1 [1], InputY2 [1], InputY3 [1], InputY4 [1], InputY5 [1]};
            				  end					
            default:begin				
            			     OutDig [4:0] <= InputDig[4:0]; 						
            			     end
	endcase
   end


endmodule