//ALU
module ALU(in1,in2,control,result);
input [31:0] in1,in2;
input [3:0] control;
output zero;
output [31:0] result;
reg [31:0] result;
always@(*)
begin
    case (control)
            4'b0000: begin result = in1&in2; end//and
            4'b0110: begin result = in1|in2; end//or
            4'b0010: begin result = in1+in2; end//add and for jal jalr
            4'b0001: begin result = in1<<in2; end//sll
            4'b0101: begin result = in1>>in2; end//srl
            4'b1101: begin result = $signed(($signed(in1))>>>in2); end//sra
            4'b1000: begin//sub
                        result = in1-in2;
                     end
            4'b1001: begin // bne
                        result = in1-in2;
                     end
            4'b1111: begin // beq
                        result = in1-in2;
                     end
            4'b1110: begin // blt
                        result = in1-in2;
                     end
             4'b1100: begin // bge
                        result = in1-in2;
                     end     
                     
            default begin result = 0; end
        endcase
end
endmodule //done

//ALUcontrol
module ALUcontrol(in1,aluop,out1);
input [3:0] in1;
input[1:0] aluop;
output [3:0] out1;
reg [3:0] out1;
always@(*)
begin
//lw, sw, lb, lbu, sb, jal.jalr=0010
    if (aluop==2'b00) begin
        out1=4'b0010;
    end
//addi,andi,slli,srli   
    if (aluop==2'b11) begin
         if (in1[2:0] == 0) begin out1 = 4'b0010; end //addi=0010
         else if(in1[2:0]==3'b111) out1 = 4'b0000; //andi=0000
         else if(in1[2:0]==3'b001) out1 = 4'b0001; //slli=0001
         else if(in1[2:0]==3'b101) out1 = 4'b0101;//srli=0101
    end
//add,sub,and,or,sll,srl
    if (aluop==2'b10) begin
        if(in1[3]==0&&in1[2:0]==3'b000) out1=4'b0010;//add=0010
        else if(in1[3]==1&&in1[2:0]==3'b000) out1=4'b1000;//sub=1000
        else if(in1[3]==0&&in1[2:0]==3'b111) out1=4'b0000;//and=0000
        else if(in1[3]==0&&in1[2:0]==3'b110) out1=4'b0110;//or=0110
        else if(in1==4'b0001) out1 = 4'b0001; //sll=0001
        else if(in1==4'b0101) out1 = 4'b0101;//srl=0101
        else if(in1==4'b1101) out1 = 4'b1101;//sra=1101
    end
//beq,bne,blt,bge
    if (aluop==2'b01) begin
        if(in1[2:0]==3'b000) out1=4'b1111;//beq=1111
        else if(in1[2:0]==3'b001) out1=4'b1001;//bne=1001
        else if(in1[2:0]==3'b100) out1=4'b1110;//blt=1110
        else if(in1[2:0]==3'b101) out1=4'b1100;//bge=1100
    end
end
endmodule //done