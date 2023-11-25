//main Data Memory
module mainDataReg
(input read_writeIn,
 input [9:0] TargetAddress,
 input [127:0] WriteData,
 output reg [127:0] ReadData,
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

                ReadData[39:32]<=mainDataReg[TargetAddress+4];
                ReadData[47:40]<=mainDataReg[TargetAddress+5];
                ReadData[55:48]<=mainDataReg[TargetAddress+6];
                ReadData[63:56]<=mainDataReg[TargetAddress+7];

                ReadData[71:64]<=mainDataReg[TargetAddress+8];
                ReadData[79:72]<=mainDataReg[TargetAddress+9];
                ReadData[87:80]<=mainDataReg[TargetAddress+10];
                ReadData[95:88]<=mainDataReg[TargetAddress+11];

                ReadData[103:96]<=mainDataReg[TargetAddress+12];
                ReadData[111:104]<=mainDataReg[TargetAddress+13];
                ReadData[119:112]<=mainDataReg[TargetAddress+14];
                ReadData[127:120]<=mainDataReg[TargetAddress+15];

                done=1;
            end
        else if(read_writeIn==1)
            begin
                mainDataReg[TargetAddress]<=WriteData[7:0];
                mainDataReg[TargetAddress+1]<=WriteData[15:8];
                mainDataReg[TargetAddress+2]<=WriteData[23:16];
                mainDataReg[TargetAddress+3]<=WriteData[31:24];

                mainDataReg[TargetAddress+4]<=WriteData[39:32];
                mainDataReg[TargetAddress+5]<=WriteData[47:40];
                mainDataReg[TargetAddress+6]<=WriteData[55:48];
                mainDataReg[TargetAddress+7]<=WriteData[63:56];

                mainDataReg[TargetAddress+8]<=WriteData[71:64];
                mainDataReg[TargetAddress+9]<=WriteData[79:72];
                mainDataReg[TargetAddress+10]<=WriteData[87:80];
                mainDataReg[TargetAddress+11]<=WriteData[95:88];

                mainDataReg[TargetAddress+12]<=WriteData[103:96];
                mainDataReg[TargetAddress+13]<=WriteData[111:104];
                mainDataReg[TargetAddress+14]<=WriteData[119:112];
                mainDataReg[TargetAddress+15]<=WriteData[127:120];
                
                done=1;
            end
    end
endmodule