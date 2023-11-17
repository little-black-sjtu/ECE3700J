`include "MAIN.v"
`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2021/11/02 20:40:21
// Design Name: 
// Module Name: Test_Branch
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

module Test_Branch;
    reg clk;
    
	PipelineprocessorPro P (
		.clock(clk)
	);

    integer file;

	initial begin
		clk = 0; 
        file = $fopen("results/result.txt", "w");
        $dumpfile("tb_PipelineprocessorPro.vcd");
        $dumpvars(0, Test_Branch);

	end


    initial begin
        while ($time <100) @(posedge clk)begin
            $display("===============================================");
        $fdisplay(file, "PC:%d, ID:%d", $signed(P.PC), $signed(P.PC/4+1));
        $fdisplay(file, "Ins:%b", P.Instrct);
        $fdisplay(file, "ra:%h (hex), %d (int), %b (bit)", P.RF.registers[1], $signed(P.RF.registers[1]), P.RF.registers[1]);
        $fdisplay(file, "t0:%h (hex), %d (int), %b (bit)", P.RF.registers[5], $signed(P.RF.registers[5]), P.RF.registers[5]);
        $fdisplay(file, "t1:%h (hex), %d (int), %b (bit)", P.RF.registers[6], $signed(P.RF.registers[6]), P.RF.registers[6]);
        $fdisplay(file, "t2:%h (hex), %d (int), %b (bit)", P.RF.registers[7], $signed(P.RF.registers[7]), P.RF.registers[7]);
        $fdisplay(file, "t3:%h (hex), %d (int), %b (bit)", P.RF.registers[28], $signed(P.RF.registers[28]), P.RF.registers[28]);
        $fdisplay(file, "t4:%h (hex), %d (int), %b (bit)", P.RF.registers[29], $signed(P.RF.registers[29]), P.RF.registers[29]);
        $fdisplay(file, "\n");
            $display("===============================================");
        end
        $finish();
    end
    always #1 clk = ~clk;

endmodule
