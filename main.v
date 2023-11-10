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
module Pipelineprocessor(input clock); //Top Module

    wire            [31:0]          Instrct, Imm, data_1, data_2, MUX_ALU, loadPC, W_data, Add_1, Add_2, ALUResult, Read_Mem, JumpTo;
    wire            [31:0]          IF_ID_nextPC, IF_ID_crntPC, IF_ID_Instrct;
    wire            [31:0]          ID_EX_nextPC, ID_EX_crntPC, ID_EX_data_1, ID_EX_data_2, ID_EX_Imm;
    wire            [31:0]          EX_MEM_nextPC, EX_MEM_JumpTo, EX_MEM_ALUResult, EX_MEM_data_2;
    wire            [31:0]          MEM_WB_nextPC, MEM_WB_Read_Mem, MEM_WB_ALUResult;
    wire            [4:0]           ID_EX_Reg_rd, EX_MEM_Reg_rd, MEM_WB_Reg_rd;
    wire            [3:0]           ALUControl, ID_EX_ALUInstrct;
    wire            [2:0]           EX_MEM_Funct3;
    wire            [1:0]           ALUOp, MemtoReg, ID_EX_ALUOp, ID_EX_MemtoReg, EX_MEM_MemtoReg, MEM_WB_MemtoReg;
    wire                            Branch, isBranch, MemRead, MemWrite, ALUSrc, isJump, RegWrite, isZero;
    wire                            ID_EX_Branch, ID_EX_MemRead, ID_EX_MemWrite, ID_EX_ALUSrc, ID_EX_isJump, ID_EX_RegWrite;
    wire                            EX_MEM_Branch, EX_MEM_MemRead, EX_MEM_MemWrite, EX_MEM_RegWrite, EX_MEM_isZero, MEM_WB_RegWrite;                         
    reg             [31:0]          PC;  
        
    initial PC = 0;   
      
 
    always @ (posedge clock ) PC <= loadPC;
    
    InstructionMemory IM (
        .instruction (Instrct),
        .address (PC/4)
    );
    Control CU (
        .opcode (IF_ID_Instrct[6:0]),
        .branch (Branch),
        .memread (MemRead),
        .memwrite (MemWrite),
        .memtoreg (MemtoReg),
        .aluop (ALUOp),
        .alusrc (ALUSrc),
        .jump (isJump),
        .regwrite (RegWrite)
    );
    RegisterFile RF (
        .regwrite (MEM_WB_RegWrite),
        .writedata (W_data),
        .readreg1 (IF_ID_Instrct[19:15]),
        .readreg2 (IF_ID_Instrct[24:20]),
        .writereg (MEM_WB_Reg_rd),
        .readdata1 (data_1),
        .readdata2 (data_2)
    );
    immgen ImmGen (
        .instr (IF_ID_Instrct),
        .imm (Imm)
    );
    ALU ALU (     
        .in1 (ID_EX_data_1),
        .in2 (MUX_ALU),
        .control (ALUControl),
        .zero (isZero),
        .result (ALUResult)
    ); 
    ALUcontrol ALU_Ctrl (
        .aluop (ID_EX_ALUOp),
        .in1 (ID_EX_ALUInstrct),
        .out1 (ALUControl)
    );
    DataMemory DM (
        .memwrite (EX_MEM_MemWrite),
        .memread (EX_MEM_MemRead),
        .fun3 (EX_MEM_Funct3),
        .address (EX_MEM_ALUResult),
        .writedata (EX_MEM_data_2),
        .readdata (Read_Mem)        
    );
    
    IF_ID_State_Reg IF_ID(
        .clock (clock),
        .currPC (PC),
        .nextPC (Add_1),
        .Instruct (Instrct),
        .currPC_out (IF_ID_crntPC),
        .nextPC_out (IF_ID_nextPC),
        .Instruct_out (IF_ID_Instrct)
    );
    ID_EX_State_Reg ID_EX(
        .clock (clock),
        .RegWrite (RegWrite),
        .MemtoReg (MemtoReg),
        .MemRead (MemRead),
        .MemWrite (MemWrite),
        .Branch (Branch),
        .Jump (isJump),
        .ALUSrc (ALUSrc),
        .ALUOp (ALUOp),
        .crntPC (IF_ID_crntPC),
        .nextPC (IF_ID_nextPC),
        .Read_rs1 (data_1),
        .Read_rs2 (data_2),
        .Imm_Gen (Imm),
        .Write_rd (IF_ID_Instrct[11:7]),
        .ALU_Instruct ({IF_ID_Instrct[30],IF_ID_Instrct[14:12]}),
        .RegWrite_out (ID_EX_RegWrite),
        .MemtoReg_out (ID_EX_MemtoReg),
        .MemRead_out (ID_EX_MemRead),
        .MemWrite_out (ID_EX_MemWrite),
        .Branch_out (ID_EX_Branch),
        .Jump_out (ID_EX_isJump),
        .ALUSrc_out (ID_EX_ALUSrc),
        .ALUOp_out (ID_EX_ALUOp),
        .crntPC_out (ID_EX_crntPC),
        .nextPC_out (ID_EX_nextPC),
        .Read_rs1_out (ID_EX_data_1),
        .Read_rs2_out (ID_EX_data_2),
        .Imm_Gen_out (ID_EX_Imm),
        .write_rd_out (ID_EX_Reg_rd),
        .ALU_Instruct_out (ID_EX_ALUInstrct)
    );
    EX_MEM_State_Reg EX_MEM(
        .clock (clock),
        .RegWrite (ID_EX_RegWrite),
        .MemtoReg (ID_EX_MemtoReg),
        .MemRead (ID_EX_MemRead),
        .MemWrite (ID_EX_MemWrite),
        .Branch (ID_EX_Branch),
        .Zero (isZero),
        .nextPC (ID_EX_nextPC),
        .ALUResult (ALUResult),
        .AddSum (JumpTo),
        .Read_rs2 (ID_EX_data_2),
        .Funct3 (ID_EX_ALUInstrct[2:0]),
        .Write_rd (ID_EX_Reg_rd),
        .RegWrite_out (EX_MEM_RegWrite),
        .MemtoReg_out (EX_MEM_MemtoReg),
        .MemRead_out (EX_MEM_MemRead),
        .MemWrite_out (EX_MEM_MemWrite),
        .Branch_out (EX_MEM_Branch),
        .Zero_out (EX_MEM_isZero),
        .nextPC_out (EX_MEM_nextPC),
        .ALUResult_out (EX_MEM_ALUResult),
        .AddSum_out (EX_MEM_JumpTo),
        .Read_rs2_out (EX_MEM_data_2),
        .Funct3_out (EX_MEM_Funct3),
        .write_rd_out (EX_MEM_Reg_rd)
    );
    MEM_WB_State_Reg MEM_WB(
        .clock (clock),
        .RegWrite (EX_MEM_RegWrite),
        .MemtoReg (EX_MEM_MemtoReg),
        .nextPC (EX_MEM_nextPC),
        .ReadData (Read_Mem),
        .ALUResult (EX_MEM_ALUResult),
        .Write_rd (EX_MEM_Reg_rd),
        .RegWrite_out (MEM_WB_RegWrite),
        .MemtoReg_out (MEM_WB_MemtoReg),
        .nextPC_out (MEM_WB_nextPC),
        .ReadData_out (MEM_WB_Read_Mem),
        .ALUResult_out (MEM_WB_ALUResult),
        .write_rd_out (MEM_WB_Reg_rd)
    );
    
    //Mux: Input_1, Input_2, (Input_3, Input_4,) Select, Output.
//    _32_bit_2to1_MUX  Mux_1(.data1(Add_1), .data2(EX_MEM_JumpTo), .sel(isBranch), .result(loadPC));
    MUX2x1  Mux_1(.in1(Add_1), .in2(JumpTo), .control(isBranch), .out1(loadPC));
    MUX2x1  Mux_2(.in1(ID_EX_data_2), .in2(ID_EX_Imm), .control(ID_EX_ALUSrc), .out1(MUX_ALU));
    MUX2x1  Mux_3(.in1(Add_2), .in2(ALUResult), .control(ID_EX_isJump), .out1(JumpTo)); //Lab4_latest
    MUX4x1  Mux_4(.data1(MEM_WB_ALUResult), .data2(MEM_WB_Read_Mem), .data3(MEM_WB_nextPC), .data4(MEM_WB_nextPC),
                            .sel(MEM_WB_MemtoReg), .result(W_data)); //Lab4_latest
    
    //Adder: Input_1, Input_2, Output.
    ADDconst Adder_1(.in1(PC), .out1(Add_1)); //PC+4
    ADDshift Adder_2(.in1(ID_EX_crntPC), .in2(ID_EX_Imm), .out1(Add_2)); //PC+Imm
    
//    and (isBranch, EX_MEM_Branch, EX_MEM_isZero);
    and (isBranch, ID_EX_Branch, isZero);     
endmodule

// InstructionMemory
module InstructionMemory(
    input [31:0] address,
    output reg [31:0] instruction
);
    reg [31:0] instructions [0:128];
    initial begin
        $readmemb("D:\\370\\Lab4\\Lab4_testcase.txt", instructions);
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
    integer i=0;
    reg [7:0] data [127:0];
    initial 
        begin  
           readdata<=0;
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
                default: readdata<= 0;
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

module MUX4x1
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
         jump <= 0; 
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
            4'b0110: begin result = in1|in2; zero = 1'b0; end//or
            4'b0010: begin result = in1+in2; zero = 1'b1; end//add and for jal jalr
            4'b0001: begin result = in1<<in2; zero = 1'b0; end//sll
            4'b0101: begin result = in1>>in2; zero = 1'b0; end//srl
            4'b1101: begin result = $signed(($signed(in1))>>>in2); zero = 1'b0; end//sra
            4'b1000: begin//sub
                        result = in1-in2;
                        if (result==0) zero = 1'b1;
                        else zero = 1'b0;
                     end
            4'b1001: begin // bne
                        result = in1-in2;
                        if (result!=0) zero = 1'b1;
                        else zero = 1'b0;
                     end
            4'b1111: begin // beq
                        result = in1-in2;
                        if (result==0) zero = 1'b1;
                        else zero = 1'b0;
                     end
            4'b1110: begin // blt
                        result = in1-in2;
                        if ($signed(in1) >= $signed(in2)) zero = 1'b0;
                        else zero = 1'b1;
                     end
             4'b1100: begin // bge
                        result = in1-in2;
                        if ($signed(in1) >= $signed(in2)) zero = 1'b1;
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
        RegWrite_out = 0; MemtoReg_out = 0; ReadData_out = 0; ALUResult_out = 0; write_rd_out = 0; nextPC_out=0;
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



