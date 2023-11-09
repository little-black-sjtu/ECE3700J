// Instruction Memory
module InstructionMemory(
    input [31:0] address, 
    output reg [31:0] instruction
);
    reg [31:0] instructions [0:63];
    initial begin
        $readmemb("testcase.txt", instructions);
    end
    always @(*)
        instruction = instructions[address];
endmodule//done
