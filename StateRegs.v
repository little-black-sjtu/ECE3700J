// State Registers
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
