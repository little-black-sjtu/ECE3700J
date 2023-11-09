// Register File
module RegisterFile(input [4:0] readreg1,
    input [4:0] readreg2,
    input [4:0] writereg,
    input [31:0] writedata,
    input regwrite,
    output  [31:0] readdata1,
    output  [31:0] readdata2);
    integer i;
    reg [31:0] registers [31:0];
    
    initial 
        begin  
           for(i=0;i<32;i=i+1)  
                registers[i] <= 0;  
        end  
    
    always @ (*) begin
        if (regwrite) registers[writereg] <= writedata;
    end
    assign readdata1 = ( readreg1 == 0)? 32'b0 : registers[readreg1];  
    assign readdata2 = ( readreg2 == 0)? 32'b0 : registers[readreg2];   
endmodule//done
