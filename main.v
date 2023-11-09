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

//Pipeline processor
module PipelineProcessor(
    input clk
);
    reg [31:0] PCAd=0;
    wire [31:0] nextPCconst, PCbranch;
    wire [31:0] instruction;
    wire [31:0] readdata1, readdata2, regwritedata, aluin2;
    wire [31:0] alu_result;
    wire [31:0] data_out;
    wire [31:0] sign_ext_imm;
    wire regwrite, memread, memwrite, branch, memtoreg, alusrc, PCselect;
    wire [1:0] aluop;
    wire [3:0] alu_control_input;
    wire zero;
    wire [31:0] temp_PCAd; // Temporary signal for MUX2x1 output

    InstructionMemory IM(.address(PCAd/4), .instruction(instruction));
    ADDconst ac(.in1(PCAd), .out1(nextPCconst));
    ADDshift as(.in1(PCAd), .in2(sign_ext_imm), .out1(PCbranch));
    assign PCselect = (branch & zero);
    MUX2x1 pcmux(.in1(nextPCconst), .in2(PCbranch), .out1(temp_PCAd), .control(PCselect)); 
    MUX2x1 alumux(.in1(readdata2), .in2(sign_ext_imm), .out1(aluin2), .control(alusrc));
    MUX2x1 memmux(.in1(alu_result), .in2(data_out), .out1(regwritedata), .control(memtoreg));
    RegisterFile RF1(.readreg1(instruction[19:15]), .readreg2(instruction[24:20]), .writereg(instruction[11:7]), .writedata(regwritedata), .regwrite(regwrite), .readdata1(readdata1), .readdata2(readdata2));
    Control CU(.opcode(instruction[6:0]), .branch(branch), .memread(memread), .memtoreg(memtoreg), .aluop(aluop), .memwrite(memwrite), .alusrc(alusrc), .regwrite(regwrite));
    ALUcontrol ALC(.in1({instruction[30], instruction[14:12]}), .aluop(aluop), .out1(alu_control_input));
    ImmGen img(.instr(instruction), .imm(sign_ext_imm));
    ALU ALU1(.in1(readdata1), .in2(aluin2), .control(alu_control_input), .zero(zero), .result(alu_result));
    DataMemory DataMem1(.address(alu_result), .writedata(readdata2), .memread(memread), .memwrite(memwrite), .readdata(data_out));
    
    always @(posedge clk) begin
        PCAd = temp_PCAd; // Update PCAd at every clock edge
    end
endmodule
