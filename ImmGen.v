// Immediate Generation
module ImmGen(instr, imm);
    input [31:0] instr;
    output [31:0] imm;
    
    reg [31:0] imm;
    always @ (*) begin
        case (instr[6:0])
            //I-type: lw
            7'b0000011: begin 
                            imm = {{20{instr[31]}}, instr[31:20]}; 
                        end 
            //I-type: addi, slli, srli, andi
            7'b0010011: begin 
                            imm = {{20{instr[31]}}, instr[31:20]}; 
                        end  
            //S-type: sw, sb
            7'b0100011: begin 
                            imm = {{20{instr[31]}}, instr[31:25], instr[11:7]}; 
                        end 
            //B-type: beq, bne, bge, blt
            7'b1100011: begin 
                            imm = {{21{instr[31]}}, instr[7], instr[30:25], instr[11:8]}; 
                        end 
            //I-type: jalr
            7'b1100111: begin 
                            imm = {{20{instr[31]}}, instr[31:20]}; 
                        end 
            //J-type: jal
            7'b1101111: begin 
                            imm = {{21{instr[31]}}, instr[19:12], instr[20], instr[30:21]};
                        end 

            default:    begin 
                            imm[31:0]  = 0; 
                        end
        endcase
    end 
endmodule//done
