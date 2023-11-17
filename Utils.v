// Utils
module MUX2x1(
    input       [31:0]         in1, in2, 
    output reg  [31:0]         out,
    input                      control
);
always@(*)
begin
    case (control)
        1'b0: out = in1; 
        1'b1: out = in2; 
        default: out = 0; 
    endcase
end
endmodule 

module MUX4x1
(
    input       [1:0]          sel,
    input       [31:0]         data1, data2, data3, data4,
    output reg  [31:0]         out
);

    always @ (*) begin
        case (sel)
            2'b00:   out = data1;
            2'b01:   out = data2;
            2'b10:   out = data3;
            2'b11:   out = data4;
            default:    out = 0;
        endcase
    end
endmodule

module MUX3x1(in1,in2,in3,out1,control);
input [31:0] in1,in2,in3;
input [1:0] control;
output [31:0] out1;
reg [31:0] out1;
always@(*)
begin
    case(control)
        2'b00: out1 = in1;
        2'b01: out1 = in2;
        2'b10: out1 = in3;
        default: out1 = 0;
     endcase
end
endmodule
