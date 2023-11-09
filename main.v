`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2023/10/22 15:10:16
// Design Name: 
// Module Name: lab3
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////
//Pipeline processor
module PipelineProcessor(
    input clk
);
    reg [31:0] PCAd=0;
    wire [31:0] nextPCconst,PCbranch;
    wire [31:0] instruction;
    wire [31:0] readdata1, readdata2,regwritedata,aluin2;
    wire [31:0] alu_result;
    wire [31:0] data_out;
    wire [31:0] sign_ext_imm;
    wire regwrite, memread, memwrite, branch, memtoreg, alusrc,PCselect;
    wire [1:0] aluop;
    wire [3:0] alu_control_input;
    wire zero;
    wire [31:0] temp_PCAd; // Temporary signal for MUX2x1 output

    InstructionMemory IM(.address(PCAd/4), .instruction(instruction));
    ADDconst ac(.in1(PCAd),.out1(nextPCconst));
    ADDshift as(.in1(PCAd),.in2(sign_ext_imm),.out1(PCbranch));
    assign PCselect = (branch & zero);
    MUX2x1 pcmux(.in1(nextPCconst),.in2(PCbranch),.out1(temp_PCAd),.control(PCselect)); 
    MUX2x1 alumux(.in1(readdata2),.in2(sign_ext_imm),.out1(aluin2),.control(alusrc));
    MUX2x1 memmux(.in1(alu_result),.in2(data_out),.out1(regwritedata),.control(memtoreg));
    RegisterFile RF1(.readreg1(instruction[19:15]), .readreg2(instruction[24:20]), .writereg(instruction[11:7]), .writedata(regwritedata), .regwrite(regwrite), .readdata1(readdata1), .readdata2(readdata2),.clk(clk));
    Control CU(.in1(instruction[6:0]), .branch(branch), .memread(memread), .memtoreg(memtoreg), .aluop(aluop), .memwrite(memwrite), .alusrc(alusrc), .regwrite(regwrite));
    ALUcontrol ALC(.in1({instruction[30], instruction[14:12]}), .aluop(aluop), .out1(alu_control_input));
    immgen img(.instr(instruction),.imm(sign_ext_imm));
    ALU ALU1(.in1(readdata1), .in2(aluin2), .control(alu_control_input), .zero(zero), .result(alu_result));
    DataMemory DataMem1(.address(alu_result), .writedata(readdata2), .memread(memread), .memwrite(memwrite), .readdata(data_out),.clk(clk));
    
    always @(posedge clk) begin
        PCAd = temp_PCAd; // Update PCAd at every clock edge
    end
endmodule

// InstructionMemory
module InstructionMemory(
    input [31:0] address,
    output reg [31:0] instruction
);
    reg [31:0] instructions [0:63];
    initial begin
        $readmemb("D:\\370\\LAB3\\testcase.txt", instructions);
    end
    always @(*)
        instruction = instructions[address];
endmodule//done

// RegisterFileÄ£¿é
module RegisterFile(input [4:0] readreg1,
    input [4:0] readreg2,
    input [4:0] writereg,
    input [31:0] writedata,
    input regwrite,
    output  [31:0] readdata1,
    output  [31:0] readdata2);
    integer i;
    reg [31:0] registers [31:0];
    
    initial 
        begin  
           for(i=0;i<32;i=i+1)  
                registers[i] <= 0;  
        end  
    
    always @ (*) begin
        if (regwrite) registers[writereg] <= writedata;
    end
    assign readdata1 = ( readreg1 == 0)? 32'b0 : registers[readreg1];  
    assign readdata2 = ( readreg2 == 0)? 32'b0 : registers[readreg2];   
endmodule//done

// DataMemoryÄ£¿é
module DataMemory(
    input [31:0] address,
    input [31:0] writedata,
    input [2:0] fun3,
    input memread,
    input memwrite,
    output reg [31:0] readdata
);
    integer i;
    reg [7:0] data [127:0];
    initial 
        begin  
           for(i=0;i<128;i=i+1)  
                data[i] <= 8'b00000000;  
        end  
        always @(*)
        begin  
           if (memwrite) //write 
            begin
                case(fun3)
                  3'b010:begin
                   data[address] <= writedata[7:0];  
                   data[address+1] <= writedata[15:8];  
                   data[address+2] <= writedata[23:16];  
                   data[address+3] <= writedata[31:24];  
                  end
                  3'b000:begin
                    data[address]<=writedata[7:0];
                  end
                  default:data[address]<=data[address];
                endcase
             end
           end
           
           always @ (*) begin
           if (memwrite) begin//read
            case (fun3)
                3'b010: begin //load word
                    readdata <= {data[address+3], data[address+2], data[address+1], data[address]};
                end
                
                3'b000: begin //load byte
                    readdata <= {{24{data[address][7]}}, data[address]};
                end
                3'b100: begin //load byte unsigned
                    readdata <= {{24{1'b0}}, data[address]};
                end
                default: readdata<= readdata;
            endcase
         end
        end
endmodule//done


module MUX2x1(in1,in2,out1,control);
input [31:0] in1,in2;
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

module Control(opcode,branch,memread,memtoreg,aluop,memwrite,alusrc,regwrite,jump);
input [6:0] opcode;
output branch,memread,memwrite,alusrc,regwrite,jump;
output [1:0] aluop,memtoreg;
reg branch,memread,memwrite,alusrc,regwrite,jump;
reg [1:0] aluop,memtoreg;
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
        //I-type,lw,lb,lbu
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
        //I-type,addi,andi,slli,srli
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
        //S-type,sw,sb
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
        //B-type,branch
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
        //I-type,jalr
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
        //J-type,jal
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
        //R-type,add,and,or,sub,srl,sll,sra
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

module ALUcontrol(in1,aluop,out1);
input [3:0] in1;
input[1:0] aluop;
output [3:0] out1;
reg [3:0] out1;
always@(*)
begin
//lw,sw, lb, lbu, sb,jal.jalr=0010
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
        else if(in1[3]==1&&in1[2:0]==3'b000) out1=4'b0110;//sub=0110
        else if(in1[3]==0&&in1[2:0]==3'b111) out1=4'b0000;//and=0000
        else if(in1[3]==0&&in1[2:0]==3'b110) out1=4'b0001;//or=0001
        else if(in1==4'b1001) out1 = 4'b0001; //sll=0001
        else if(in1==4'b1101) out1 = 4'b0101;//srl=0101
        else if(in1==4'b1101) out1 = 4'b1101;//sra=1101
    end
//beq,bne,blt,bge
    if (aluop==2'b01) begin
        if(in1[2:0]==3'b000) out1=4'b0110;//beq=0110
        else if(in1[2:0]==3'b001) out1=4'b1001;//bne=1001
        else if(in1[2:0]==3'b100) out1=4'b1110;//bge=1110
        else if(in1[2:0]==3'b101) out1=4'b1100;//blt=1100
    end
end
endmodule //done

module ALU(in1,in2,control,zero,result);
input [31:0] in1,in2;
input [3:0] control;
output zero;
output [31:0] result;
reg [31:0] result;
reg zero;
always@(*)
begin
    case (control)
            4'b0000: begin result = in1&in2; zero = 1'b0; end//and
            4'b0001: begin result = in1|in2; zero = 1'b0; end//or
            4'b0010: begin result = in1+in2; zero = 1'b0; end//add
            4'b0001: begin result = in1<<in2; zero = 1'b0; end//sll
            4'b0101: begin result = in1>>in2; zero = 1'b0; end//srl
            4'b1101: begin result = $signed(($signed(in1))>>>in2); zero = 1'b0; end//sra
            4'b0110: begin//sub
                        result = in1-in2;
                        if (result==0) zero = 1'b1;
                        else zero = 1'b0;
                     end
            4'b1001: begin // bne
                        result = in1-in2;
                        if (result!=0) zero = 1'b1;
                        else zero = 1'b0;
                     end
            4'b1110: begin // bge
                        result = in1-in2;
                        if ($signed(in1) < $signed(in2)) zero = 1'b0;
                        else zero = 1'b1;
                     end
             4'b1100: begin // blt
                        result = in1-in2;
                        if ($signed(in1) < $signed(in2)) zero = 1'b1;
                        else zero = 1'b0;
                     end     
                     
            default begin result = 0; zero = 1'b0; end
        endcase
end
endmodule //done

module immgen(instr,imm);
    input [31:0] instr;
    output [31:0] imm;
    
    reg [31:0] imm;
    always @ (*) begin
        case (instr[6:0])
            //I-type: lw
            7'b0000011: begin 
                            imm = {{20{instr[31]}}, instr[31:20]}; 
                        end 
            //I-type: addi,slli,srli,andi
            7'b0010011: begin 
                            imm = {{20{instr[31]}}, instr[31:20]}; 
                        end  
            //S-type: sw,sb
            7'b0100011: begin 
                            imm = {{20{instr[31]}}, instr[31:25], instr[11:7]}; 
                        end 
            //B-type: beq, bne,bge,blt
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

module IF_ID_State_Reg(
    input                   clock,
    input       [31:0]      currPC,
    input       [31:0]      nextPC,
    input       [31:0]      Instruct,
    output reg  [31:0]      currPC_out,
    output reg  [31:0]      nextPC_out,
    output reg  [31:0]      Instruct_out
);

    initial begin
        currPC_out = 0; 
        nextPC_out = 0; 
        Instruct_out = 0;
    end

    always @ (posedge clock) begin
        currPC_out = currPC;
        nextPC_out<= nextPC;
        Instruct_out = Instruct;
    end
    
endmodule

module ID_EX_State_Reg(
    input                   clock,
    //control unit
    input                   RegWrite,   //WB
    input       [1:0]       MemtoReg,   //WB
    input                   MemRead,    //MEM
    input                   MemWrite,   //MEM
    input                   Branch,     //MEM
    input                   Jump,       //EX
    input                   ALUSrc,     //EX
    input       [1:0]       ALUOp,      //EX
    input       [31:0]      nextPC,
    input       [31:0]      crntPC,
    input       [31:0]      Read_rs1,
    input       [31:0]      Read_rs2,
    input       [31:0]      Imm_Gen,
    input       [4:0]       Write_rd,
    input       [3:0]       ALU_Instruct,
    output reg              RegWrite_out,   //WB
    output reg  [1:0]       MemtoReg_out,   //WB
    output reg              MemRead_out,    //MEM
    output reg              MemWrite_out,   //MEM
    output reg              Branch_out,     //MEM
    output reg              Jump_out,       //EX
    output reg              ALUSrc_out,     //EX
    output reg  [1:0]       ALUOp_out,      //EX
    output reg  [31:0]      crntPC_out,
    output reg  [31:0]      nextPC_out,
    output reg  [31:0]      Read_rs1_out,
    output reg  [31:0]      Read_rs2_out,
    output reg  [31:0]      Imm_Gen_out,
    output reg  [4:0]       write_rd_out,
    output reg  [3:0]       ALU_Instruct_out
);

    initial begin
        RegWrite_out = 0; MemtoReg_out = 0; MemRead_out = 0; MemWrite_out = 0; Branch_out = 0; Jump_out = 0; ALUSrc_out = 0; 
        ALUOp_out = 0; crntPC_out = 0; nextPC_out = 0; Read_rs1_out = 0; Read_rs2_out = 0; Imm_Gen_out = 0; write_rd_out = 0; ALU_Instruct_out = 0;
    end

    always @ (posedge clock) begin
        RegWrite_out    <= RegWrite;
        MemtoReg_out    <= MemtoReg;
        MemRead_out     <= MemRead;
        MemWrite_out    <= MemWrite;
        Branch_out      <= Branch;
        Jump_out        <= Jump;
        ALUSrc_out      <= ALUSrc;
        ALUOp_out       <= ALUOp;
        crntPC_out      <= crntPC;
        nextPC_out      <= nextPC;
        Read_rs1_out     <= Read_rs1;
        Read_rs2_out     <= Read_rs2;
        write_rd_out      <= Write_rd;
        Imm_Gen_out     <= Imm_Gen;
        ALU_Instruct_out <= ALU_Instruct;
    end

endmodule

module EX_MEM_State_Reg(
    input                   clock,
    input                   RegWrite,   //WB
    input       [1:0]       MemtoReg,   //WB
    input                   MemRead,    //MEM
    input                   MemWrite,   //MEM
    input                   Branch,     //MEM
    input                   Zero,
    input       [31:0]      nextPC,
    input       [31:0]      ALUResult,
    input       [31:0]      AddSum,
    input       [31:0]      Read_rs2,
    input       [2:0]       Funct3,
    input       [4:0]       Write_rd,
    output reg              RegWrite_out,   //WB
    output reg  [1:0]       MemtoReg_out,   //WB
    output reg              MemRead_out,    //MEM
    output reg              MemWrite_out,   //MEM
    output reg              Branch_out,     //MEM
    output reg              Zero_out,
    output reg  [31:0]      nextPC_out,
    output reg  [31:0]      ALUResult_out,
    output reg  [31:0]      AddSum_out,
    output reg  [31:0]      Read_rs2_out,
    output reg  [2:0]       Funct3_out,
    output reg  [4:0]       write_rd_out
);

    initial begin
        RegWrite_out = 0; MemtoReg_out = 0; MemRead_out = 0; MemWrite_out = 0; Branch_out = 0; Zero_out = 0; nextPC_out = 0;
        ALUResult_out = 0; AddSum_out = 0; Read_rs2_out = 0; Funct3_out = 0;write_rd_out = 0;
    end

    always @ (posedge clock) begin
        RegWrite_out    <= RegWrite;
        MemtoReg_out    <= MemtoReg;
        MemRead_out     <= MemRead;
        MemWrite_out    <= MemWrite;
        Branch_out      <= Branch;
        Zero_out        <= Zero;
        nextPC_out      <= nextPC;
        ALUResult_out   <= ALUResult;
        AddSum_out      <= AddSum;
        Read_rs2_out     <= Read_rs2;
        Funct3_out      <= Funct3;
        write_rd_out      <= Write_rd;
    end

endmodule

module MEM_WB_State_Reg(
    input                   clock,
    input                   RegWrite,   //WB
    input       [1:0]       MemtoReg,   //WB
    input       [31:0]      nextPC,
    input       [31:0]      ReadData,
    input       [31:0]      ALUResult,
    input       [4:0]       Write_rd,
    output reg              RegWrite_out,   //WB
    output reg  [1:0]       MemtoReg_out,   //WB
    output reg  [31:0]      nextPC_out,
    output reg  [31:0]      ReadData_out,
    output reg  [31:0]      ALUResult_out,
    output reg  [4:0]       write_rd_out
);

    initial begin
        RegWrite_out = 0; MemtoReg_out = 0; ReadData_out = 0; ALUResult_out = 0; write_rd_out = 0;
    end
    
    always @ (posedge clock) begin
        RegWrite_out    <= RegWrite;
        MemtoReg_out    <= MemtoReg;
        nextPC_out      <= nextPC;
        ReadData_out    <= ReadData;
        ALUResult_out   <= ALUResult;
        write_rd_out      <= Write_rd;
    end

endmodule



