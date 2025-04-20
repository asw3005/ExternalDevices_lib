//*****************************************************************************************
//XY decoder.

module XYPosDec	#(

    localparam IN_WIDTH = 32,
    localparam OUT_WIDTH = 6,
    localparam EER_CODE = 45

    )											
    (
    input wire clock,
    input wire [IN_WIDTH - 1:0] InputX,						 
    input wire [IN_WIDTH - 1:0] InputY,						 
    output reg [OUT_WIDTH - 1:0] OutputX,						 
    output reg [OUT_WIDTH - 1:0] OutputY							 
    );


integer  ix = 0, jx = 0, iy = 0, jy = 0;
reg [IN_WIDTH - 1:0] X_t, Y_t;

always @(posedge clock)   begin				 
	X_t <= InputX;
	Y_t <= InputY;
end
							
always @(posedge clock)											
begin

    if(X_t == 0) begin
        OutputX <= 0;
    end
    else if(X_t > 32'h8000_0000) begin
        OutputX <= EER_CODE;
    end
    else begin        
        for(ix = 0; ix < 32; ix = ix + 1) begin    
            if(X_t == (32'd1 << ix)) begin
                OutputX <= ix + 1;
            end 
            else if( X_t & (32'd1 << ix) ) begin             
                jx <= jx + 1;            
            end
        end       
    end

    if (jx > 1) begin
        OutputX <= EER_CODE;
        jx <= 0;
    end 
             
end
						 
always @(posedge clock)											
begin

    if(Y_t == 0) begin
        OutputY <= 0;
    end
    else if(Y_t > 32'h8000_0000) begin
        OutputY <= EER_CODE;
    end
    else begin        
        for(iy = 0; iy < 32; iy = iy + 1) begin    
            if(Y_t == (32'd1 << iy)) begin
                OutputY <= iy + 1;
            end 
            else if( Y_t & (32'd1 << iy) ) begin             
                jy <= jy + 1;            
            end
        end       
    end

    if (jy > 1) begin
        OutputY <= EER_CODE;
        jy <= 0;
    end 
    
end    
    
endmodule