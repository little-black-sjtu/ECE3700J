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
    
	Pipelineprocessor test (
		.clock(clk)
	);

	initial begin
		clk = 0; 
	end

    initial begin
        while ($time <100) @(posedge clk)begin
            $display("===============================================");
            $display("Clock cycle %d, PC = %H", $time/2, test.PC);
            $display("ra = %H, t0 = %H, t1 = %H", test.RF.registers[1], test.RF.registers[5], test.RF.registers[6]);
            $display("t2 = %H, t3 = %H, t4 = %H", test.RF.registers[7], test.RF.registers[28], test.RF.registers[29]);
            $display("===============================================");
        end
        $finish();
    end
    always #1 clk = ~clk;

endmodule
