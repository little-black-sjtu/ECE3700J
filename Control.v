// Control
module Control(opcode, branch, memread, memtoreg, aluop, memwrite, alusrc, regwrite, jump);
input [6:0] opcode;
output branch, memread, memwrite, alusrc, regwrite, jump;
output [1:0] aluop, memtoreg;
reg branch, memread, memwrite, alusrc, regwrite, jump;
reg [1:0] aluop, memtoreg;
initial begin
         aluop <= 0;
         memtoreg <= 0; 
         branch <= 0; 
         memread <= 0; 
         memwrite <= 0; 
         alusrc <= 0; 
         jump <= 0; 
         regwrite <= 0;
   end
   
   always @ (opcode) begin
        case (opcode)
        //I-type, lw, lb, lbu
            7'b0000011: begin 
         aluop <= 2'b00;
         memtoreg <= 2'b01; 
         branch <= 0; 
         memread <= 1; 
         memwrite <= 0; 
         alusrc <= 1; 
         jump <= 0; 
         regwrite <= 1;
            end 
//          7'b0001111: 
        //I-type, addi, andi, slli, srli
            7'b0010011: begin 
         aluop <= 2'b11;
         memtoreg <= 2'b00; 
         branch <= 0; 
         memread <= 0; 
         memwrite <= 0; 
         alusrc <= 1; 
         jump <= 0; 
         regwrite <= 1;
            end 
//          7'b0010111: 
        //S-type, sw, sb
            7'b0100011: begin 
         aluop <= 2'b00;
         memtoreg <= 2'b00; 
         branch <= 0; 
         memread <= 0; 
         memwrite <= 1; 
         alusrc <= 1; 
         jump <= 0; 
         regwrite <= 0;
            end 
//          7'b0110111: 
        //B-type, branch
            7'b1100011: begin 
         aluop <= 2'b01;
         memtoreg <= 2'b00; 
         branch <= 1; 
         memread <= 0; 
         memwrite <= 0; 
         alusrc <= 0; 
         jump <= 0; 
         regwrite <= 0;
            end 
        //I-type, jalr
            7'b1100111: begin 
         aluop <= 2'b00;
         memtoreg <= 2'b10; 
         branch <= 1; 
         memread <= 0; 
         memwrite <= 0; 
         alusrc <= 1; 
         jump <= 1; 
         regwrite <= 1;
            end 
        //J-type, jal
            7'b1101111: begin 
         aluop <= 2'b00;
         memtoreg <= 2'b10; 
         branch <= 1; 
         memread <= 0; 
         memwrite <= 0; 
         alusrc <= 1; 
         jump <= 1; 
         regwrite <= 1;
            end 
//          7'b1110011: 
        //R-type, add, and, or, sub, srl, sll, sra
            7'b0110011: begin 
         aluop <= 2'b10;
         memtoreg <= 2'b00; 
         branch <= 0; 
         memread <= 0; 
         memwrite <= 0; 
         alusrc <= 0; 
         jump <= 0; 
         regwrite <= 1;
            end 
            default:    begin 
         aluop <= 0;
         memtoreg <= 0; 
         branch <= 0; 
         memread <= 0; 
         memwrite <= 0; 
         alusrc <= 0; 
         jump <= 0; 
         regwrite <= 0;
            end
        endcase
end
endmodule //done
