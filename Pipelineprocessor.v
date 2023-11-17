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
`include "Forward_unit.v"
`include "Comparator.v"
`include "Hazard_detection.v"


module Pipelineprocessor(input clock); //Top Module

    wire            [31:0]          Instrct, Imm, Imm_shift, data_1, data_2, MUX_ALU, loadPC, W_data, Add_1, Add_2,Add_3,Add_4, ALUResult, Read_Mem ,ALUin1,ALUin2;
    wire            [31:0]          IF_ID_nextPC, IF_ID_crntPC, IF_ID_Instrct;
    wire            [31:0]          ID_EX_nextPC, ID_EX_crntPC, ID_EX_data_1, ID_EX_data_2, ID_EX_Imm, EX_MEM_Imm, MEM_WB_Imm;
    wire            [31:0]          EX_MEM_nextPC,  EX_MEM_ALUResult,mem_write_content,forwardBout;
    wire            [31:0]          MEM_WB_nextPC, MEM_WB_Read_Mem, MEM_WB_ALUResult,MEM_WB_data_1,MEM_WB_data_2,DMwriteData;
    wire            [31:0]          mux2_out, mux3_out, cmp_data_1, cmp_data_2;
    wire            [4:0]           EX_MEM_Rs2_Addr, ID_EX_Reg_rd, EX_MEM_Reg_rd, MEM_WB_Reg_rd, ID_EX_Rs1_Addr, ID_EX_Rs2_Addr;
    wire            [3:0]           ALUControl, ID_EX_ALUInstrct;
    wire            [2:0]           EX_MEM_Funct3;
    wire            [1:0]           ALUOp, MemtoReg, ID_EX_ALUOp, ID_EX_MemtoReg, EX_MEM_MemtoReg, MEM_WB_MemtoReg,forwarda_control,forwardb_control;
    wire                            PC_write, Branch, isBranch, MemRead, MemWrite, ALUSrc, isJump, RegWrite, isZero,whether_hazard,ForwardEq1,ForwardEq2;
    wire                            if_flush, IF_ID_write, ID_EX_Branch, ID_EX_MemRead, ID_EX_MemWrite, ID_EX_ALUSrc, ID_EX_isJump, ID_EX_RegWrite;
    wire                            EX_MEM_Branch, EX_MEM_MemRead, EX_MEM_MemWrite, EX_MEM_RegWrite, EX_MEM_isZero, MEM_WB_RegWrite, MEM_WB_MemRead;
    wire            [9:0]           allControlIn, allControlOut;              
    reg             [31:0]          PC;  
        
    initial PC = 0;   

    assign Imm_shift = {Imm[30:0],1'b0}; //NewBug_22
 //  allControl = {ALUOp, MemtoReg, Branch, MemRead, MemWrite, ALUSrc, Jump, RegWrite}
    assign RegWrite = allControlOut[0];
    assign isJump   = allControlOut[1]; //Bug_11
    assign ALUSrc   = allControlOut[2];
    assign MemWrite = allControlOut[3];
    assign MemRead  = allControlOut[4];
    assign Branch   = allControlOut[5];
    assign MemtoReg = allControlOut[7:6];
    assign ALUOp    = allControlOut[9:8];
 

    always @ (posedge clock ) 
        if (PC_write == 1'b1)  PC <= loadPC;
    
    InstructionMemory IM (
        .address (PC/4),
        .instruction (Instrct)
    );
    Control CU (
        .opcode (IF_ID_Instrct[6:0]),
        .allControl (allControlIn)
    );
    RegisterFile RF (
        .clock(clock),
        .regwrite (MEM_WB_RegWrite),
        .writedata (W_data),
        .readreg1 (IF_ID_Instrct[19:15]),
        .readreg2 (IF_ID_Instrct[24:20]),
        .writereg (MEM_WB_Reg_rd),
        .readdata1 (data_1),
        .readdata2 (data_2)
    );
    Comparator cmp (
        .opcode(IF_ID_Instrct[6:0]),
        .func ({1'b0, IF_ID_Instrct[14:12]}),
        .in1 (cmp_data_1),
        .in2 (cmp_data_2),
        .isZero (isZero)
    );
    ImmGen ImmGen (
        .instr (IF_ID_Instrct),
        .imm (Imm)
    );
    ALU ALU (     
        .in1 (ALUin1),
        .in2 (MUX_ALU),
        .control (ALUControl),
        .result (ALUResult)
    ); 
    ALUcontrol ALU_Ctrl (
        .aluop (ID_EX_ALUOp),
        .in1 (ID_EX_ALUInstrct),
        .out1 (ALUControl)
    );
    DataMemory DM (
        .clock (clock),
        .memwrite (EX_MEM_MemWrite),
        .memread (EX_MEM_MemRead),
        .fun3 (EX_MEM_Funct3),
        .address (EX_MEM_ALUResult),
        .writedata (DMwriteData),
        .readdata (Read_Mem)        
    );
    
    IF_ID_State_Reg IF_ID(
        .clock (clock),
        .IF_Flush(if_flush),
        .IF_ID_Write(IF_ID_write), 
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
        //.Branch (Branch),
        .Jump (isJump),
        .ALUSrc (ALUSrc),
        .ALUOp (ALUOp),
        .crntPC (IF_ID_crntPC),
        .nextPC (IF_ID_nextPC),
        .Read_rs1 (data_1),
        .Read_rs2 (data_2),
        .Reg_rs1_addr (IF_ID_Instrct[19:15]),
        .Reg_rs2_addr (IF_ID_Instrct[24:20]),
        .Imm_Gen (Imm),
        .Write_rd (IF_ID_Instrct[11:7]),
        .ALU_Instruct ({IF_ID_Instrct[30],IF_ID_Instrct[14:12]}),
        .RegWrite_out (ID_EX_RegWrite),
        .MemtoReg_out (ID_EX_MemtoReg),
        .MemRead_out (ID_EX_MemRead),
        .MemWrite_out (ID_EX_MemWrite),
        //.Branch_out (ID_EX_Branch),
        .Jump_out (ID_EX_isJump),
        .ALUSrc_out (ID_EX_ALUSrc),
        .ALUOp_out (ID_EX_ALUOp),
        .crntPC_out (ID_EX_crntPC),
        .nextPC_out (ID_EX_nextPC),
        .Read_rs1_out (ID_EX_data_1),
        .Read_rs2_out (ID_EX_data_2),
        .Reg_rs1_addr_out (ID_EX_Rs1_Addr),
        .Reg_rs2_addr_out (ID_EX_Rs2_Addr),
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
        //.Branch (ID_EX_Branch),
        .ForwardB (ALUin2),
        .imm_out (EX_MEM_Imm),
        .Reg_rs2_addr (ID_EX_Rs2_Addr),
        //.imm_out (EX_MEM_Imm),
        .Reg_rs2_addr_out (EX_MEM_Rs2_Addr),        

        //.Zero (isZero),
        .nextPC (ID_EX_nextPC),
        .ALUResult (ALUResult),
        .Funct3 (ID_EX_ALUInstrct[2:0]),
        .Write_rd (ID_EX_Reg_rd),
        .RegWrite_out (EX_MEM_RegWrite),
        .MemtoReg_out (EX_MEM_MemtoReg),
        .MemRead_out (EX_MEM_MemRead),
        .MemWrite_out (EX_MEM_MemWrite),
        .ForwardB_out (forwardBout),
        //.Branch_out (EX_MEM_Branch),
        //.Zero_out (EX_MEM_isZero),
        .nextPC_out (EX_MEM_nextPC),
        .ALUResult_out (EX_MEM_ALUResult),
        //.AddSum_out (EX_MEM_JumpTo),
        .Funct3_out (EX_MEM_Funct3),
        .write_rd_out (EX_MEM_Reg_rd)
    );
    MEM_WB_State_Reg MEM_WB(
        .clock (clock),
        .MemRead (EX_MEM_MemRead),
        .RegWrite (EX_MEM_RegWrite),
        .MemtoReg (EX_MEM_MemtoReg),
        .imm (EX_MEM_Imm),
        .nextPC (EX_MEM_nextPC),
        .ReadData (Read_Mem),
        .ALUResult (EX_MEM_ALUResult),
        .Write_rd (EX_MEM_Reg_rd),
        .MemRead_out (MEM_WB_MemRead),
        .RegWrite_out (MEM_WB_RegWrite),
        .MemtoReg_out (MEM_WB_MemtoReg),
        .imm_out (MEM_WB_Imm),
        .nextPC_out (MEM_WB_nextPC),
        .ReadData_out (MEM_WB_Read_Mem),
        .ALUResult_out (MEM_WB_ALUResult),
        .write_rd_out (MEM_WB_Reg_rd)
    );
    Forward_unit FU (
        .idex_r1 (ID_EX_Rs1_Addr),
        .idex_r2 (ID_EX_Rs2_Addr),
        .ifid_r1 (IF_ID_Instrct[19:15]),
        .ifid_r2 (IF_ID_Instrct[24:20]),
        .exmem_r2 (EX_MEM_Rs2_Addr),
        .memwb_reg_rd (MEM_WB_Reg_rd),
        .exmem_reg_rd (EX_MEM_Reg_rd),
        .idex_reg_rd (ID_EX_Reg_rd),
        .memwb_reg_write (MEM_WB_RegWrite),
        .exmem_reg_write (EX_MEM_RegWrite),
        .exmem_mem_write (EX_MEM_MemWrite),
        .exmem_mem_read (EX_MEM_MemRead),
        .memwb_mem_read (MEM_WB_MemRead),
        .idex_mem_read (ID_EX_MemRead),
        .idex_mem_write (ID_EX_MemWrite),
        .idex_reg_write (ID_EX_RegWrite),
        .ifid_branch (Branch),
        .o_forward_a (forwarda_control),
        .o_forward_b (forwardb_control),
        .o_forward_eq1 (ForwardEq1),
        .o_forward_eq2 (ForwardEq2),
        .o_mem_src (MemSrc)
    );

    Hazard_detection HD (
        .ifid_r1 (IF_ID_Instrct[19:15]),
        .ifid_r2 (IF_ID_Instrct[24:20]),
        .idex_rd (ID_EX_Reg_rd),
        .exmem_rd (EX_MEM_Reg_rd),
        .idex_mem_read (ID_EX_MemRead),
        .exmem_mem_read (EX_MEM_MemRead),
        .id_branch (allControlIn[5]),
        .id_mem_write (allControlIn[3]),
        .idex_regwrite (ID_EX_RegWrite),
        .ifid_write (IF_ID_write),
        .PC_write (PC_write),
        .whether_hazard (whether_hazard)
    );

    //Mux: Input_1, Input_2, (Input_3, Input_4,) Select, Output.
//    _32_bit_2to1_MUX  Mux_1(.data1(Add_1), .data2(EX_MEM_JumpTo), .sel(isBranch), .result(loadPC));
    MUX4x1  PCmux(.in1(Add_1), .in2(Add_2),.in3(Add_3),.in4(Add_4), .control({isJump,isBranch}), .out(loadPC));
    MUX2x1  Alumux(.in1(ALUin2), .in2(ID_EX_Imm), .control(ID_EX_ALUSrc), .out(MUX_ALU));
    MUX2x1  Memsrcmux(.in1(forwardBout), .in2(W_data), .control(MemSrc), .out(DMwriteData));
    MUX4x1  Regmux(.in1(MEM_WB_Read_Mem), .in2(MEM_WB_nextPC), .in3(MEM_WB_Imm), .in4(MEM_WB_ALUResult),
                                    .control(MEM_WB_MemtoReg), .out(W_data)); //Lab4_latest
    bit10MUX2x1  Stallmux(.in1(10'b0000000000), .in2(allControlIn), .control(whether_hazard), .out(allControlOut));
    MUX3x1 Mux_forwarda(.in1(ID_EX_data_1),.in2(W_data),.in3(EX_MEM_ALUResult),.out(ALUin1),.control(forwarda_control));
    MUX3x1 Mux_forwardb(.in1(ID_EX_data_2),.in2(W_data),.in3(EX_MEM_ALUResult),.out(ALUin2),.control(forwardb_control));
    MUX2x1 Mux_comparatora(.in1(data_1), .in2(EX_MEM_ALUResult), .out(cmp_data_1), .control(ForwardEq1));
    MUX2x1 Mux_comparatorb(.in1(data_2), .in2(EX_MEM_ALUResult), .out(cmp_data_2), .control(ForwardEq2));
    
    //Adder: Input_1, Input_2, Output.
    ADDconst Adder_1(.in1(PC), .out1(Add_1)); //PC+4
    ADDshift Adder_2(.in1(IF_ID_crntPC), .in2(Imm), .out1(Add_2)); //PC+Imm
    ADDshift Adder_3(.in1(IF_ID_crntPC), .in2(Imm), .out1(Add_3));  //j
    ADDshift Adder_4(.in1(data_1), .in2(Imm), .out1(Add_4));  //jalr don't whether shift imm
//    and (isBranch, EX_MEM_Branch, EX_MEM_isZero);
    and (isBranch, Branch, isZero); 
    or  (if_flush, isBranch, isJump);      
endmodule