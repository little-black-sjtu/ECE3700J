`include "Pipelineprocessor.v"
`default_nettype none

module tb_Pipelineprocessor;
reg clk;

Pipelineprocessor P
(
    .clock (clk)
);

localparam CLK_PERIOD = 4;
always #(CLK_PERIOD/2) clk=~clk;
integer file;

initial begin
    file = $fopen("results/result_lab5.txt", "w");
    $dumpfile("tb_Pipelineprocessor.vcd");
    $dumpvars(0, tb_Pipelineprocessor);
end

initial begin
    #2;
    forever #CLK_PERIOD begin
        $fdisplay(file, "PC:%d, ID:%d", $signed(P.PC), $signed(P.PC/4+1));
        $fdisplay(file, "Ins:%b", P.Instrct);
        $fdisplay(file, "ra:%h (hex), %d (int), %b (bit)", P.RF.registers[1], $signed(P.RF.registers[1]), P.RF.registers[1]);
        $fdisplay(file, "t0:%h (hex), %d (int), %b (bit)", P.RF.registers[5], $signed(P.RF.registers[5]), P.RF.registers[5]);
        $fdisplay(file, "t1:%h (hex), %d (int), %b (bit)", P.RF.registers[6], $signed(P.RF.registers[6]), P.RF.registers[6]);
        $fdisplay(file, "t2:%h (hex), %d (int), %b (bit)", P.RF.registers[7], $signed(P.RF.registers[7]), P.RF.registers[7]);
        $fdisplay(file, "t3:%h (hex), %d (int), %b (bit)", P.RF.registers[28], $signed(P.RF.registers[28]), P.RF.registers[28]);
        $fdisplay(file, "t4:%h (hex), %d (int), %b (bit)", P.RF.registers[29], $signed(P.RF.registers[29]), P.RF.registers[29]);
        $fdisplay(file, "\n");
    end
end
initial begin
    #CLK_PERIOD clk<=0;
    #(CLK_PERIOD*100) $finish(0);
end

endmodule
`default_nettype wire