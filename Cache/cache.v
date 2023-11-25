//cache
module cache
(
    input read_writeIn,doneFromMain,
    input [9:0] TargetAddressIn,
    input [31:0] WriteDataIn,
    input [127:0] ReadDataFromMain,
    output reg read_writeOut,hit_miss,
    output reg [9:0] TargetAdressOut,
    output reg [31:0] ReadDataOut,
    output reg [127:0] WriteDataOut 
);
    integer i;
    reg [133:0] CacheReg [3:0]; // [133]  [132]  [131-128]  [127:96]  [95:64]  [63:32]  [31:0]
                                //   V      D        TAG      Word3    Word2    Word1   Word0
    initial begin
        ReadDataOut<=0;
        WriteDataOut<=0;
        hit_miss<=0;
        read_writeOut<=0;
        TargetAdressOut<=0;
        for(i=0;i<1024;i=i+1) CacheReg[i]<=0;
    end

    assign index = TargetAddressIn[5:4];
    assign tag = TargetAddressIn[9:6];
    assign word_offset = TargetAddressIn[3:2];

    always @(*) begin
        if(read_writeIn==1) begin   // if write
            if (CacheReg [index][133]==1 && CacheReg [index][131:128]==tag) begin  // if valid and hit
                hit_miss<=1;
                read_writeOut<=0;
                // CacheReg [index][(word_offset*32)+:31]<=WriteDataIn;
                // CacheReg [index][132]<=1;
            end
            else begin    // if not valid or not hit
                if (CacheReg [index][132]==1) begin // if dirty, write back
                    read_writeOut<=1;
                    TargetAdressOut<=TargetAddressIn;
                    WriteDataOut<=CacheReg [index][127:0];
                    #100;
                end
                hit_miss<=0;
                //fetch
                CacheReg [index][127:0]<=ReadDataFromMain;
                CacheReg [index][133]<=1;
                CacheReg [index][132]<=0;
                CacheReg [index][131:128]<=tag;
                #100;
                //////
            end
            //write
            if (CacheReg [index][(word_offset*32)+:31]!=WriteDataIn) begin
                CacheReg [index][(word_offset*32)+:31]<=WriteDataIn;
                CacheReg [index][132]<=1;
            end
        end
        else begin // if read
            if (CacheReg [index][133]==1 && CacheReg [index][131:128]==tag) begin // if valid and hit
                hit_miss<=1;
                read_writeOut<=0;
            end
            else begin // if not valid or not hit
                hit_miss<=0;
                //fetch
                CacheReg [index][127:0]<=ReadDataFromMain;
                CacheReg [index][133]<=1;
                CacheReg [index][132]<=0;
                CacheReg [index][131:128]<=tag;
                #100;
                //////
            end
            ReadDataOut<=CacheReg [index][(word_offset*32)+:31];
        end
    end

endmodule
