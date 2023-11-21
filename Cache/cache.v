//cache
module cache
(
    input readIn,writeIn,doneFromMain,
    input [9:0] TargetAddressIn,
    input [31:0] WriteDataIn,
    input [31:0] ReadDataFromMain,
    output reg readOut,writeOut,hit,miss,
    output reg [9:0] TargetAdressOut,
    output reg [31:0] ReadDataOut,WriteDataOut 
);
    integer i;
    reg [133:0] CacheReg [3:0]; // [133]  [132]  [131-128]  [127:96]  [95:64]  [63:32]  [31:0]
                                //   V      D        TAG      Word3    Word2    Word1   Word0
    initial begin
        ReadDataOut<=0;
        WriteDataOut<=0;
        hit<=0;
        miss<=0;
        readOut<=0;
        writeOut<=0;
        TargetAdressOut<=0;
        for(i=0;i<1024;i=i+1) mainDataReg[i]<=0;
    end

endmodule
