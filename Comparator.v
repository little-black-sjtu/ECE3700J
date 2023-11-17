// Comparator
module Comparator 
(
    input       [6:0]           opcode,
    input       [3:0]           func,
    input       [31:0]          in1, in2,
    output reg                  isZero
);
    initial isZero = 1'b0;
    always @ (*) begin
        case (func)
                4'b0000: isZero = (in1==in2)?1'b1:1'b0; //SUB_BEQ
                4'b0001: isZero = (in1==in2)?1'b0:1'b1; //SUB_NEQ
                4'b0100: isZero = ($signed(in1) < $signed(in2))?1'b1:1'b0; //SUB_BLT
                4'b0101: isZero = ($signed(in1) < $signed(in2))?1'b0:1'b1; //SUB_BGE
                default: isZero = 1'b0;
        endcase
        if(opcode == 7'b1100111) isZero = 1'b1;
    end
endmodule