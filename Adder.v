// Adder
module ADDshift(in1,in2,out1);
input [31:0] in1,in2;
output [31:0] out1;
reg [31:0] out1,temp;
always@(*)
begin
    begin
         temp[31:1] = in2[30:0];
         temp[0] = 0;
         out1=temp+in1;
    end
end
endmodule 

module ADDconst(in1,out1);
input [31:0] in1;
output [31:0] out1;
reg [31:0] out1;
always@(*)
begin
    out1=in1+4;
end
endmodule 
