//main Data Memory
module mainDataReg
(input read_writeIn,
 input [9:0] TargetAddress,
 input [31:0] WriteData,
 output reg [31:0] ReadData,
 output reg done
);
    integer i;
    reg [7:0] mainDataReg [1023:0];
    initial begin
        ReadData<=0;
        done<=0;
        for(i=0;i<1024;i=i+1) mainDataReg[i]<=0;
    end

    always @(*)   begin
        if(read_writeIn==0)
            begin
                ReadData[7:0]<=mainDataReg[TargetAddress];
                ReadData[15:8]<=mainDataReg[TargetAddress+1];
                ReadData[23:16]<=mainDataReg[TargetAddress+2];
                ReadData[31:24]<=mainDataReg[TargetAddress+3];
                done=1;
            end
        else if(read_writeIn==1)
            begin
                mainDataReg[TargetAddress]<=WriteData[7:0];
                mainDataReg[TargetAddress+1]<=WriteData[15:8];
                mainDataReg[TargetAddress+2]<=WriteData[23:16];
                mainDataReg[TargetAddress+3]<=WriteData[31:24];
                done=1;
            end
    end
endmodule