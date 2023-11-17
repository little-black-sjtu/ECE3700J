// Control
module Control(opcode, allControl);
input [6:0] opcode;
output [9:0] allControl;
//  allControl = {ALUOp, MemtoReg, Branch, MemRead, MemWrite, ALUSrc, Jump, RegWrite}
reg [9:0] allControl;
initial begin
            allControl <= 0;
    end
   
    always @ (opcode) begin
         case (opcode)
        //I-type,load
            7'b0000011: begin allControl[5] <= 0; allControl[2] <= 1; allControl[9:8] <= 2'b00; allControl[3] <= 0; allControl[4] <= 1; allControl[7:6] <= 2'b00; allControl[1] <= 0; allControl[0] <= 1; end 
//          7'b0001111: 
        //I-type,Imm
            7'b0010011: begin allControl[5] <= 0; allControl[2] <= 1; allControl[9:8] <= 2'b11; allControl[3] <= 0; allControl[4] <= 0; allControl[7:6] <= 2'b11; allControl[1] <= 0; allControl[0] <= 1; end 
//          7'b0010111: 
        //S-type,save
            7'b0100011: begin allControl[5] <= 0; allControl[2] <= 1; allControl[9:8] <= 2'b00; allControl[3] <= 1; allControl[4] <= 0; allControl[7:6] <= 2'b11; allControl[1] <= 0; allControl[0] <= 0; end 
//          7'b0110111: 
        //B-type,branch
            7'b1100011: begin allControl[5] <= 1; allControl[2] <= 0; allControl[9:8] <= 2'b01; allControl[3] <= 0; allControl[4] <= 0; allControl[7:6] <= 2'b11; allControl[1] <= 0; allControl[0] <= 0; end 
        //I-type,jalr
            7'b1100111: begin allControl[5] <= 1; allControl[2] <= 1; allControl[9:8] <= 2'b00; allControl[3] <= 0; allControl[4] <= 0; allControl[7:6] <= 2'b01; allControl[1] <= 1; allControl[0] <= 1; end 
        //J-type,jal
            7'b1101111: begin allControl[5] <= 0; allControl[2] <= 1; allControl[9:8] <= 2'b00; allControl[3] <= 0; allControl[4] <= 0; allControl[7:6] <= 2'b01; allControl[1] <= 1; allControl[0] <= 1; end 
//          7'b1110011: 
        //R-type,calc
            7'b0110011: begin allControl[5] <= 0; allControl[2] <= 0; allControl[9:8] <= 2'b10; allControl[3] <= 0; allControl[4] <= 0; allControl[7:6] <= 2'b11; allControl[1] <= 0; allControl[0] <= 1; end 
            default:    begin allControl    <= 0; end
        endcase
end
endmodule //done