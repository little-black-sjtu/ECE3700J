`timescale 1ns / 1ps
`include "InstructionMemory.v"
`include "RegisterFile.v"
`include "DataMemory.v"
`include "ALU.v"
`include "Adder.v"
`include "Control.v"
`include "ImmGen.v"
`include "Utils.v"
`include "StateRegs.v"

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
        .address (PC/4),
        .instruction (Instrct)
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
    ImmGen ImmGen (
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
    MUX2x1  Mux_1(.in1(Add_1), .in2(JumpTo), .control(isBranch), .out(loadPC));
    MUX2x1  Mux_2(.in1(ID_EX_data_2), .in2(ID_EX_Imm), .control(ID_EX_ALUSrc), .out(MUX_ALU));
    MUX2x1  Mux_3(.in1(Add_2), .in2(ALUResult), .control(ID_EX_isJump), .out(JumpTo)); //Lab4_latest
    MUX4x1  Mux_4(.in1(MEM_WB_ALUResult), .in2(MEM_WB_Read_Mem), .in3(MEM_WB_nextPC), .in4(MEM_WB_nextPC),
                            .control(MEM_WB_MemtoReg), .out(W_data)); //Lab4_latest
    
    //Adder: Input_1, Input_2, Output.
    ADDconst Adder_1(.in1(PC), .out1(Add_1)); //PC+4
    ADDshift Adder_2(.in1(ID_EX_crntPC), .in2(ID_EX_Imm), .out1(Add_2)); //PC+Imm
    
//    and (isBranch, EX_MEM_Branch, EX_MEM_isZero);
    and (isBranch, ID_EX_Branch, isZero);     
endmodule