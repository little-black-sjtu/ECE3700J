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

    wire            [31:0]          Instrct, Imm, Imm_shift, data_1, data_2, MUX_ALU, loadPC, W_data, Add_1, Add_2,Add_3, ALUResult, Read_Mem, JumpTo,ALUin1,ALUin2;
    wire            [31:0]          IF_ID_nextPC, IF_ID_crntPC, IF_ID_Instrct;
    wire            [31:0]          ID_EX_nextPC, ID_EX_crntPC, ID_EX_data_1, ID_EX_data_2, ID_EX_Imm, EX_MEM_Imm, MEM_WB_Imm;
    wire            [31:0]          EX_MEM_nextPC, EX_MEM_JumpTo, EX_MEM_ALUResult,EX_MEM_data_1, EX_MEM_data_2,mem_write_content;
    wire            [31:0]          MEM_WB_nextPC, MEM_WB_Read_Mem, MEM_WB_ALUResult,MEM_WB_data_1,MEM_WB_data_2;
    wire            [31:0]          mux2_out, mux3_out;
    wire            [4:0]           ID_EX_Reg_rd, EX_MEM_Reg_rd, MEM_WB_Reg_rd, ID_EX_Rs1_Addr, ID_EX_Rs2_Addr;
    wire            [3:0]           ALUControl, ID_EX_ALUInstrct;
    wire            [2:0]           EX_MEM_Funct3;
    wire            [1:0]           ALUOp, MemtoReg, ID_EX_ALUOp, ID_EX_MemtoReg, EX_MEM_MemtoReg, MEM_WB_MemtoReg,forwarda_control,forwardb_control;
    wire                            Branch, isBranch, MemRead, MemWrite, ALUSrc, isJump, RegWrite, isZero,whether_hazard,ForwardEq1,ForwardEq2;
    wire                            if_flush, ID_EX_Branch, ID_EX_MemRead, ID_EX_MemWrite, ID_EX_ALUSrc, ID_EX_isJump, ID_EX_RegWrite;
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
        .instruction (Instrct),
        .address (PC/4)
    );
    Control CU (
        .opcode (IF_ID_Instrct[6:0]),
//        .branch (Branch),
//        .memread (MemRead),
//        .memwrite (MemWrite),
//        .memtoreg (MemtoReg),
//        .aluop (ALUOp),
//        .alusrc (ALUSrc),
//        .jump (isJump),
//        .regwrite (RegWrite)
          .allControl (allControlOut)
    );
    RegisterFile RF (
        .clock (clock),
        .regwrite (MEM_WB_RegWrite),
        .writedata (W_data),
        .readreg1 (IF_ID_Instrct[19:15]),
        .readreg2 (IF_ID_Instrct[24:20]),
        .writereg (MEM_WB_Reg_rd),
        .readdata1 (data_1),
        .readdata2 (data_2)
    );

    Comparator cmp (
        .func ({IF_ID_Instrct[2], IF_ID_Instrct[14:12]}),
        .in1 (mux2_out),
        .in2 (mux3_out),
        .isZero (isZero)
    );
    immgen ImmGen (
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
        .writedata (EX_MEM_data_2),
        .readdata (Read_Mem)        
    );
    
    IF_ID_State_Reg IF_ID(
        .clock (clock),
        .IF_Flush (if_flush),
        //.IF_ID_write (IF_ID_write),
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

        .imm_out (EX_MEM_Imm),
        .Reg_rs2_addr (ID_EX_Rs2_Addr),
        //.imm_out (EX_MEM_Imm),
        .Reg_rs2_addr_out (EX_MEM_Rs2_Addr),        

        //.Zero (isZero),
        .nextPC (ID_EX_nextPC),
        .ALUResult (ALUResult),
        .AddSum (JumpTo),
        .Read_rs1 (ID_EX_data_1),
        .Read_rs2 (ID_EX_data_2),
        .Funct3 (ID_EX_ALUInstrct[2:0]),
        .Write_rd (ID_EX_Reg_rd),
        .RegWrite_out (EX_MEM_RegWrite),
        .MemtoReg_out (EX_MEM_MemtoReg),
        .MemRead_out (EX_MEM_MemRead),
        .MemWrite_out (EX_MEM_MemWrite),
        //.Branch_out (EX_MEM_Branch),
        //.Zero_out (EX_MEM_isZero),
        .nextPC_out (EX_MEM_nextPC),
        .ALUResult_out (EX_MEM_ALUResult),
        //.AddSum_out (EX_MEM_JumpTo),
        .Read_rs1_out (EX_MEM_data_1),
        .Read_rs2_out (EX_MEM_data_2),
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
        .Read_rs1 (EX_MEM_data_1),
        .Read_rs2 (EX_MEM_data_2),
        .ALUResult (EX_MEM_ALUResult),
        .Write_rd (EX_MEM_Reg_rd),
        .MemRead_out (MEM_WB_MemRead),
        .RegWrite_out (MEM_WB_RegWrite),
        .MemtoReg_out (MEM_WB_MemtoReg),
        .imm_out (MEM_WB_Imm),
        .nextPC_out (MEM_WB_nextPC),
        .ReadData_out (MEM_WB_Read_Mem),
        .Read_rs1_out (MEM_WB_data_1),
        .Read_rs2_out (MEM_WB_data_2),
        .ALUResult_out (MEM_WB_ALUResult),
        .write_rd_out (MEM_WB_Reg_rd)
        );
        
 Forwardunit FU (
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
    .id_mem_write (ID_EX_MemWrite),
    .idex_regwrite (ID_EX_RegWrite),
    .ifid_write (IF_ID_write),
    .PC_write (PC_write),
    .whether_hazard (whether_hazard)
);
    
    //Mux: Input_1, Input_2, (Input_3, Input_4,) Select, Output.
//    _32_bit_2to1_MUX  Mux_1(.data1(Add_1), .data2(EX_MEM_JumpTo), .sel(isBranch), .result(loadPC));
    MUX3X1  Mux_1(.in1(Add_1), .in2(Add_2),.in3(Add_3) ,.control({isJump,isBranch}), .out1(loadPC));
    MUX2x1  Mux_2(.in1(ALUin2), .in2(ID_EX_Imm), .control(ID_EX_ALUSrc), .out1(MUX_ALU));
    MUX2x1  Mux_3(.in1(Add_2), .in2(ALUResult), .control(ID_EX_isJump), .out1(JumpTo)); //Lab4_latest
    MUX4x1  Mux_4(.data1(MEM_WB_Read_Mem), .data2(MEM_WB_nextPC), .data3(MEM_WB_Imm), .data4(MEM_WB_ALUResult),
                                    .sel(MEM_WB_MemtoReg), .result(W_data)); //Lab4_latest
    MUX3X1 Mux_forwarda(.in1(ID_EX_data_1),.in2(EX_MEM_data_1),.in3(MEM_WB_data_1),.out1(ALUin1),.control(forwarda_control));
    MUX3X1 Mux_forwardb(.in1(ID_EX_data_2),.in2(EX_MEM_data_2),.in3(MEM_WB_data_2),.out1(ALUin2),.control(forwardb_control));
    
    //Adder: Input_1, Input_2, Output.
    ADDconst Adder_1(.in1(PC), .out1(Add_1)); //PC+4
    ADDshift Adder_2(.in1(ID_EX_crntPC), .in2(ID_EX_Imm), .out1(Add_2)); //PC+Imm
    Adder Adder_3(.data1(data_1), .data2(Imm), .result(Add_3));  //j
    
//    and (isBranch, EX_MEM_Branch, EX_MEM_isZero);
    and (isBranch, Branch, isZero); 
    or  (IF_Flush, isBranch, isJump);   
endmodule

// InstructionMemory
module InstructionMemory(
    input [31:0] address,
    output reg [31:0] instruction
);
    reg [31:0] instructions [0:128];
    initial begin
        $readmemb("D:\\370\\Lab5\\Lab5_testcase.txt", instructions);
    end
    always @(*)
        instruction = instructions[address];
endmodule//done

// RegisterFileÄ£¿é
module RegisterFile(input [4:0] readreg1,
    input [4:0] readreg2,
    input [4:0] writereg,
    input [31:0] writedata,
    input regwrite,clock,
    output  [31:0] readdata1,
    output  [31:0] readdata2);
    integer i;
    reg [31:0] registers [31:0];
    
    initial 
        begin  
           for(i=0;i<32;i=i+1)  
                registers[i] <= 0;  
        end  
    
    always @ (negedge clock) begin
        if (regwrite&&writereg!=0) registers[writereg] <= writedata;
    end
    assign readdata1 = ( readreg1 == 0)? 32'b0 : registers[readreg1];  
    assign readdata2 = ( readreg2 == 0)? 32'b0 : registers[readreg2];   
endmodule//done

module Comparator 
(
    input       [3:0]           func,
    input       [31:0]          in1, in2,
    output reg                  isZero
);
    initial isZero = 1'b1;
    always @ (*) begin
        case (func)
                4'b0000: isZero = (in1==in2)?1'b1:1'b0; //SUB_BEQ
                4'b0001: isZero = (in1==in2)?1'b0:1'b1; //SUB_NEQ
                4'b0100: isZero = ($signed(in1) < $signed(in2))?1'b1:1'b0; //SUB_BLT
                4'b0101: isZero = ($signed(in1) < $signed(in2))?1'b0:1'b1; //SUB_BGE
                default: isZero = 1'b1;
        endcase
    end
endmodule


// DataMemoryÄ£¿é
module DataMemory(
    input [31:0] address,
    input [31:0] writedata,
    input [2:0] fun3,
    input memread,clock,
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
        always @(negedge clock)
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
           if (memread) begin//read
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

module MUX3X1(in1,in2,in3,out1,control);
input [31:0] in1,in2,in3;
input [1:0] control;
output [31:0] out1;
reg [31:0] out1;
always@(*)
begin
    case(control)
        2'b00: out1 = in1;
        2'b01: out1 = in2;
        2'b10: out1 = in3;
        default: out1 = 0;
     endcase
end
endmodule

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

module Adder
(
    input       [31:0]          data1, data2,
    output reg  [31:0]          result
);
    
    always @ (*) begin
        result = data1 + data2;
    end
endmodule

module Control(opcode,allControl);
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
            7'b1100111: begin allControl[5] <= 0; allControl[2] <= 1; allControl[9:8] <= 2'b00; allControl[3] <= 0; allControl[4] <= 0; allControl[7:6] <= 2'b01; allControl[1] <= 1; allControl[0] <= 1; end 
        //J-type,jal
            7'b1101111: begin allControl[5] <= 1; allControl[2] <= 1; allControl[9:8] <= 2'b00; allControl[3] <= 0; allControl[4] <= 0; allControl[7:6] <= 2'b01; allControl[1] <= 1; allControl[0] <= 1; end 
//          7'b1110011: 
        //R-type,calc
            7'b0110011: begin allControl[5] <= 0; allControl[2] <= 0; allControl[9:8] <= 2'b10; allControl[3] <= 0; allControl[4] <= 0; allControl[7:6] <= 2'b11; allControl[1] <= 0; allControl[0] <= 1; end 
            default:    begin allControl    <= 0; end
        endcase
end
endmodule //done

module ALUcontrol(in1,aluop,out1);
input [3:0] in1;
input[1:0] aluop;
output [3:0] out1;
reg [3:0] out1;

initial 
out1=4'b0000;

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
    input                   clock,IF_Flush, IF_ID_Write,
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
        if(IF_Flush == 1'b1)begin
            Instruct_out<=0;
            currPC_out <= 0; 
            nextPC_out <= 0; 
        end
        currPC_out <= currPC;
        nextPC_out<= nextPC;
        Instruct_out <= Instruct;
    end
    
endmodule

module ID_EX_State_Reg(
    input                   clock,
    //control unit
    input                   RegWrite,   //WB
    input       [1:0]       MemtoReg,   //WB
    input                   MemRead,    //MEM
    input                   MemWrite,   //MEM
    input                   Jump,       //EX
    input                   ALUSrc,     //EX
    input       [1:0]       ALUOp,      //EX
    input       [31:0]      nextPC,
    input       [31:0]      crntPC,
    input       [31:0]      Read_rs1,
    input       [31:0]      Read_rs2,
    input       [4:0]       Reg_rs1_addr,
    input       [4:0]       Reg_rs2_addr,
    input       [31:0]      Imm_Gen,
    input       [4:0]       Write_rd,
    input       [3:0]       ALU_Instruct,
    output reg              RegWrite_out,   //WB
    output reg  [1:0]       MemtoReg_out,   //WB
    output reg              MemRead_out,    //MEM
    output reg              MemWrite_out,   //MEM
    output reg              Jump_out,       //EX
    output reg              ALUSrc_out,     //EX
    output reg  [1:0]       ALUOp_out,      //EX
    output reg  [31:0]      crntPC_out,
    output reg  [31:0]      nextPC_out,
    output reg  [31:0]      Read_rs1_out,
    output reg  [31:0]      Read_rs2_out,
    output reg  [4:0]       Reg_rs1_addr_out, //Bug_01
    output reg  [4:0]       Reg_rs2_addr_out, //Bug_01
    output reg  [31:0]      Imm_Gen_out,
    output reg  [4:0]       write_rd_out,
    output reg  [3:0]       ALU_Instruct_out
);

    initial begin
        RegWrite_out = 0; MemtoReg_out = 0; MemRead_out = 0; MemWrite_out = 0; Jump_out = 0; ALUSrc_out = 0; Reg_rs1_addr_out=0;Reg_rs2_addr_out=0;
        ALUOp_out = 0; crntPC_out = 0; nextPC_out = 0; Read_rs1_out = 0; Read_rs2_out = 0; Imm_Gen_out = 0; write_rd_out = 0; ALU_Instruct_out = 0;
    end

    always @ (posedge clock) begin
        RegWrite_out    <= RegWrite;
        MemtoReg_out    <= MemtoReg;
        MemRead_out     <= MemRead;
        MemWrite_out    <= MemWrite;
        Reg_rs1_addr_out<= Reg_rs1_addr;
        Reg_rs2_addr_out<= Reg_rs2_addr;
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
    input       [31:0]      imm,
    input       [31:0]      nextPC,
    input       [31:0]      ALUResult,
    input       [31:0]      AddSum,
    input       [31:0]      Read_rs1,
    input       [31:0]      Read_rs2,
    input       [4:0]       Reg_rs2_addr,
    input       [2:0]       Funct3,
    input       [4:0]       Write_rd,
    output reg              RegWrite_out,   //WB
    output reg  [1:0]       MemtoReg_out,   //WB
    output reg              MemRead_out,    //MEM
    output reg              MemWrite_out,   //MEM
    output reg  [31:0]      imm_out,
    output reg  [31:0]      nextPC_out,
    output reg  [31:0]      ALUResult_out,
    output reg  [31:0]      Read_rs2_out,
    output reg  [31:0]      Read_rs1_out,
    output reg  [2:0]       Funct3_out,
    output reg  [4:0]       write_rd_out,
    output reg  [4:0]       Reg_rs2_addr_out
);

    initial begin
        RegWrite_out = 0; MemtoReg_out = 0; MemRead_out = 0; MemWrite_out = 0;  nextPC_out = 0;imm_out=0;
        ALUResult_out = 0;  Read_rs2_out = 0; Read_rs1_out = 0; Funct3_out = 0;write_rd_out = 0;Reg_rs2_addr_out=0;
    end

    always @ (posedge clock) begin
        RegWrite_out    <= RegWrite;
        MemtoReg_out    <= MemtoReg;
        MemRead_out     <= MemRead;
        MemWrite_out    <= MemWrite;
        Reg_rs2_addr_out <= Reg_rs2_addr;
        nextPC_out      <= nextPC;
        ALUResult_out   <= ALUResult;
        imm_out <= imm;
        Read_rs1_out     <= Read_rs1;
        Read_rs2_out     <= Read_rs2;
        Funct3_out      <= Funct3;
        write_rd_out      <= Write_rd;
    end

endmodule

module MEM_WB_State_Reg(
    input                   clock,
    input                   MemRead,
    input                   RegWrite,   //WB
    input       [1:0]       MemtoReg,   //WB
    input       [31:0]      nextPC,
    input       [31:0]      imm,
    input       [31:0]      Read_rs1,
    input       [31:0]      Read_rs2,
    input       [31:0]      ReadData,
    input       [31:0]      ALUResult,
    input       [4:0]       Write_rd,
    output reg              MemRead_out,
    output reg              RegWrite_out,   //WB
    output reg  [1:0]       MemtoReg_out,   //WB
    output reg  [31:0]      nextPC_out,
    output reg  [31:0]      imm_out,
    output reg  [31:0]      ReadData_out,
    output reg  [31:0]      Read_rs2_out,
    output reg  [31:0]      Read_rs1_out,
    output reg  [31:0]      ALUResult_out,
    output reg  [4:0]       write_rd_out
);

    initial begin
        RegWrite_out = 0; MemtoReg_out = 0; ReadData_out = 0; ALUResult_out = 0; write_rd_out = 0; nextPC_out=0;Read_rs2_out = 0; Read_rs1_out = 0;MemRead_out=0;imm_out=0;
    end
    
    always @ (posedge clock) begin
        MemRead_out     <= MemRead;
        RegWrite_out    <= RegWrite;
        MemtoReg_out    <= MemtoReg;
        nextPC_out      <= nextPC;
        imm_out         <= imm;
        ReadData_out    <= ReadData;
        Read_rs1_out     <= Read_rs1;
        Read_rs2_out     <= Read_rs2;
        ALUResult_out   <= ALUResult;
        write_rd_out      <= Write_rd;
    end

endmodule

module Forwardunit
(
    input [4:0] idex_r1,
    input [4:0] idex_r2,
    input [4:0] ifid_r1,
    input [4:0] ifid_r2,
    input [4:0] exmem_r2, //New_Bug_05
    
    input [4:0] memwb_reg_rd,
    input [4:0] exmem_reg_rd,
    input [4:0] idex_reg_rd,
    
    input memwb_reg_write, 
    input exmem_reg_write,
    input exmem_mem_write,

    input exmem_mem_read,
    input memwb_mem_read,
    
    input idex_mem_read,
    input idex_mem_write,
    input idex_reg_write,
    input ifid_branch,
    
    output reg [1:0] o_forward_a,
    output reg [1:0] o_forward_b,
    output reg o_forward_eq1,
    output reg o_forward_eq2,
    output reg o_mem_src
    );
    
    initial begin
        o_forward_a = 2'b00;
        o_forward_b = 2'b00;
        o_forward_eq1 = 1'b0;
        o_forward_eq2 = 1'b0;
        o_mem_src = 1'b0; //NewBug_08
    end
    
    always @(*) begin
    //ForwardA
        //EX
        if (exmem_reg_write && (exmem_reg_rd!=0) && exmem_reg_rd == idex_r1 
            && !exmem_mem_read && !idex_mem_read && !idex_mem_write) //NewBug_16
            o_forward_a = 2'b10;
        //MEM
        else if (memwb_reg_write && (memwb_reg_rd!=0) && memwb_reg_rd == idex_r1)
            o_forward_a = 2'b01;
        else
            o_forward_a = 2'b00;
    
    //ForwardB
         //EX
        if (exmem_reg_write && (exmem_reg_rd!=0) && exmem_reg_rd == idex_r2 
            && !exmem_mem_read && !idex_mem_read && !idex_mem_write) //NewBug_06
            o_forward_b = 2'b10;
         //MEM
        else if (memwb_reg_write && (memwb_reg_rd!=0) && memwb_reg_rd == idex_r2) 
            o_forward_b = 2'b01;
        else
            o_forward_b = 2'b00;
   
    //Memsrc
        if (memwb_reg_rd == exmem_r2 && exmem_mem_write) //NewBug_03
            o_mem_src = 1'b1;
        else
            o_mem_src = 1'b0;

    //ForwardEq
        if (exmem_reg_write && (exmem_reg_rd!=0) && exmem_reg_rd == ifid_r1 && ifid_branch)
            o_forward_eq1 = 1'b1;
        else
            o_forward_eq1 = 1'b0;
        
        if (exmem_reg_write && (exmem_reg_rd!=0) && exmem_reg_rd == ifid_r2 && ifid_branch)
            o_forward_eq2 = 1'b1; //NewBug_07
        else
            o_forward_eq2 = 1'b0; //NewBug_07

    end

endmodule

module Hazard_detection(
    input [4:0]ifid_r1,
    input [4:0]ifid_r2,
    input [4:0]idex_rd,
    input [4:0]exmem_rd,
    input idex_mem_read,
    //input idex_mem_write,
    input exmem_mem_read,
    input id_branch,
    input id_mem_write,
    input idex_regwrite,

    output reg ifid_write,
    // output reg whether_hazard,
    output reg PC_write,
    output reg whether_hazard
);

initial begin
        PC_write=1'b1; ifid_write=1'b1; whether_hazard=1'b1;
    end

    always @(*) begin 
        if (idex_mem_read && !id_mem_write) begin
            if (idex_rd == ifid_r1 || idex_rd == ifid_r2) begin
                PC_write=1'b0; ifid_write=1'b0; whether_hazard=1'b1;
            end
            else begin
                PC_write=1'b1; ifid_write=1'b1; whether_hazard=1'b0;
            end  
        end
        else if (id_branch && idex_regwrite) begin
            if ((idex_rd!=0) && (ifid_r1 == idex_rd || ifid_r2 == idex_rd)) begin
                PC_write=1'b0; ifid_write=1'b0; whether_hazard=1'b1;
            end
            else begin
                PC_write=1'b1; ifid_write=1'b1; whether_hazard=1'b0;
            end  
        end
        else if (id_branch && exmem_mem_read) begin
            if ((exmem_rd!=0) && (ifid_r1 == exmem_rd || ifid_r2 == exmem_rd)) begin
                PC_write=1'b0; ifid_write=1'b0; whether_hazard=1'b1;
            end
            else begin
                PC_write=1'b1; ifid_write=1'b1; whether_hazard=1'b0;
            end  
        end
        else begin
            PC_write=1'b1; ifid_write=1'b1; whether_hazard=1'b0;
        end
    end
endmodule


