// Adder
module ADDshift(
    input [31:0] in1, in2,
    output reg [31:0] out1
);
reg [31:0] temp;
always@(*)
begin
    temp[31:1] = in2[30:0];
    temp[0] = 0;
    out1=temp+in1;
end
endmodule 

module ADDconst(
    input [31:0] in1,
    output reg [31:0] out1
);
always@(*)
begin
    out1=in1+4;
end
endmodule 
