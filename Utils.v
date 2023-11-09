// Utils
module MUX2x1(in1, in2, out1, control);
input [31:0] in1, in2;
input control;
output [31:0] out1;
reg [31:0] out1;
always@(*)
begin
    case (control)
        1'b0: out1 = in1; 
        1'b1: out1 = in2; 
        default: out1 = 0; 
    endcase
end
endmodule 

module MUX4x2
(
    input       [1:0]          sel, 
    input       [31:0]         data1, data2, data3, data4, 
    output reg  [31:0]         result
);

    always @ (*) begin
        case (sel)
            2'b00:   result = data1;
            2'b01:   result = data2;
            2'b10:   result = data3;
            2'b11:   result = data4;
            default:    result = 0;
        endcase
    end
endmodule
