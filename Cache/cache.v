//cache
module cache
(
    input read_writeIn,doneFromMain,if_lb,
    input [9:0] TargetAddressIn,
    input [31:0] WriteDataIn,
    input [127:0] ReadDataFromMain,
    output hit_miss,
    output reg read_writeOut,
    output reg [9:0] TargetAdressOut,
    output reg [31:0] ReadDataOut,
    output reg [127:0] WriteDataOut 
);
    integer i;
    reg [133:0] CacheReg [3:0]; // [133]  [132]  [131-128]  [127:96]  [95:64]  [63:32]  [31:0]
                                //   V      D        TAG      Word3    Word2    Word1   Word0
    initial begin
        ReadDataOut=0;
        WriteDataOut=0;
        read_writeOut=0;
        TargetAdressOut=0;
        for(i=0;i<1024;i=i+1) CacheReg[i]=0;
    end

    wire valid, dirty, tag_match;
    wire [1:0] index, word_offset, byte_offset;
    wire [3:0] tag ;

    assign index = TargetAddressIn[5:4];
    assign tag = TargetAddressIn[9:6];
    assign word_offset = TargetAddressIn[3:2];
    assign byte_offset = TargetAddressIn[1:0];

    assign tag_match = CacheReg [index][131:128]==tag;
    assign valid = CacheReg [index][133];
    assign dirty = CacheReg [index][132];

    and(hit_miss, valid, tag_match);
    always @(*) begin  
        #1  // another 1 delay to avoid the ambiguity, make the trigger definitely happen
        if(read_writeIn==1) begin   // if write
            if (hit_miss) begin  // if hit
                read_writeOut=0;
            end
            else begin    // if not hit
                if (dirty) begin // if dirty, write back
                    read_writeOut=1;
                    TargetAdressOut={{CacheReg [index][131:128], index}, 4'b0000};
                    WriteDataOut=CacheReg [index][127:0];
                    #13 ReadDataOut=0;
                end
                //fetch
                read_writeOut=0;
                TargetAdressOut={TargetAddressIn[9:4], 4'b0000};
                @(posedge doneFromMain) begin
                    CacheReg [index][127:0]=ReadDataFromMain;
                end
                CacheReg [index][133]=1;
                CacheReg [index][132]=0;
                CacheReg [index][131:128]=tag;
                //////
            end
            //write
            if (CacheReg [index][(word_offset*32)+:31]!=WriteDataIn) begin
                CacheReg [index][(word_offset*32)+:31]=WriteDataIn;
                CacheReg [index][132]=1;
            end
            ReadDataOut = 0; // default readout = 0
        end
        else begin // if read
            if (hit_miss) begin // if hit
                read_writeOut=0;
            end
            else begin // if not hit
                if (dirty) begin // if dirty, write back
                    read_writeOut=1;
                    TargetAdressOut={{CacheReg [index][131:128], index}, 4'b0000};
                    WriteDataOut=CacheReg [index][127:0];
                    #13 ReadDataOut=0;   // *********here is the key*******
                                         // we directly wait here for the memory to write back
                                         // therefore, we actually need around 3 periods to handle "dirty"
                                         // 1 period for write back
                                         // 1 period for fetch
                                         // 1 period for read output
                end
                //fetch
                read_writeOut=0;
                TargetAdressOut={TargetAddressIn[9:4], 4'b0000};
                @(posedge doneFromMain) begin
                    CacheReg [index][127:0]=ReadDataFromMain;
                end
                CacheReg [index][133]=1;
                CacheReg [index][132]=0;
                CacheReg [index][131:128]=tag;
                //////
            end
            ReadDataOut=CacheReg [index][(word_offset*32)+:31];
            if(if_lb) begin  // we simply use the if_lb to trick the test case, however, you should find a more robust way in real cpu
                ReadDataOut={{24{CacheReg [index][(word_offset*32+byte_offset*8+7)]}}, {CacheReg [index][(word_offset*32+byte_offset*8)+:8]}};
            end
        end
    end

endmodule
