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
    input       [31:0]         in1, in2, // in3, in4, 
    output reg  [31:0]         out,
    input       [1:0]          control
);

    always @ (*) begin
        case (control)
            2'b00:   out = in1;
            2'b01:   out = in2;
            // 2'b10:   out = in3;
            // 2'b11:   out = in4;
            default:    out = 0;
        endcase
    end
endmodule
