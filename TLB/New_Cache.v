module New_Cache (
    input                           done,               // From main memory
    input                           write_in,           // From CPU request, 1'b1 for write, 1'b0 for read
    input                           addr_prepared,      // From TLB address
    input                           funct,              // From CPU request, for differentiating lw,lb (,lbu )
                                                        // 000 for lb/sb, 010 for lw/sw
    input       [9:0]               rqst_addr,          // From physicalTLB 
    input       [31:0]              read_data_in,       // From main memory
    input       [31:0]              write_data_in,      // From CPU request
    
    output                          hit,                // To CPU request, 1'b1 for hit,   1'b0 for miss 
    output reg                      write_out,          // To main memory, 1'b1 for write, 1'b0 for read 
    output      [31:0]              read_data_out,      // To CPU request
    output reg  [31:0]              write_data_out,     // To main memory
    output reg  [9:0]               addr_out            // To main memory
);
    
    reg                             pos_done;           // NEW
    reg         [1:0]               LRU;
    reg         [134:0]             cache_setA          [1:0];  // 134    133   132:128 127:0
    reg         [134:0]             cache_setB          [1:0];  //  VALID DIRTY   TAG    WORD[0-3]
    ////////////////////////////////////////////////////// 7 MSB of cache is V(1'b)+D(1'b)+Tag(5'b) // NEW

    wire                            hit_setA, hit_setB, equal_A, equal_B; // NEW
    wire        [31:0]              data_A, data_B, lw_out, lb_out; //, lbu_out;
    wire        [134:0]             block_A, block_B; // NEW
    
    ////////////////////////////////////////////////////// for simplicity by WL and CWQ
    wire                            setIndex, block_index; // NEW (Change From 2 bits To 1 bit)
    wire        [134:0]             cache_set [1:0];
    wire        [4:0]               tagContent_A, tagContent_B; // NEW (Change From 4 bits To 5 bits)
    wire                            valid_A, dirty_A, valid_B, dirty_B; // NEW

    integer i;
    initial begin
        pos_done = 0; addr_out = 0; 
        write_out = 0; write_data_out = 0; 
        for (i = 0; i <= 1; i = i + 1) begin
            cache_setA[i] = 0; cache_setB[i] = 0; LRU[i] = 0;
        end
    end
    
    assign  setIndex = rqst_addr[4];
    assign  block_index = LRU[setIndex];
    assign  block_A = setIndex == 0 ? cache_setA[0] : cache_setB[0];//block index instead of setIndex
    assign  block_B = setIndex == 0 ? cache_setA[1] : cache_setB[1];
    assign  tagContent_A = block_A[(132)-: 5];
    assign  tagContent_B = block_B[(132)-: 5];
    assign  equal_A = (tagContent_A == rqst_addr[(10-1)-: 5]);
    assign  equal_B = (tagContent_B == rqst_addr[(10-1)-: 5]);
    assign  valid_A = block_A[134];
    assign  valid_B = block_B[134];
    assign  dirty_A = block_A[133];
    assign  dirty_B = block_B[133];
    
    ////////////////////////////////////////////////////// To CPU request
    or  (hit, hit_setA, hit_setB);
    and (hit_setA, valid_A, equal_A);
    and (hit_setB, valid_B, equal_B);

    _N_bit_4to1_MUX #(.N(32)) Mux_CacheOut1(.data1(block_A[31-: 32]), .data2(block_A[63-: 32]), 
    .data3(block_A[95-: 32]), .data4(block_A[127-:32]), .sel(rqst_addr[3:2]), .result(data_A));

    _N_bit_4to1_MUX #(.N(32)) Mux_CacheOut2(.data1(block_B[31-: 32]), .data2(block_B[63-: 32]), 
    .data3(block_B[95-: 32]), .data4(block_B[127-:32]), .sel(rqst_addr[3:2]), .result(data_B));
    
    _N_bit_4to1_MUX #(.N(32)) Mux_CacheData(.data1(lw_out), .data2(data_A), .data3(data_B), .data4(lw_out), 
    .sel({hit_setB, hit_setA}), .result(lw_out));
    
   _N_bit_4to1_MUX #(.N(32)) Mux_Cache_lbu(.data1({{24{1'b0}}, lw_out[7:0]}), 
   .data2({{24{1'b0}}, lw_out[15:8]}), .data3({{24{1'b0}}, lw_out[23:16]}), 
   .data4({{24{1'b0}}, lw_out[31:24]}), .sel(rqst_addr[1:0]), .result(lb_out));

    // _N_bit_4to1_MUX #(.N(32)) Mux_Cache_lb (.data1({{24{lw_out[7]}}, lw_out[7:0]}), 
    // .data2({{24{lw_out[7]}}, lw_out[15:8]}), .data3({{24{lw_out[7]}}, lw_out[23:16]}), 
    // .data4({{24{lw_out[7]}}, lw_out[31:24]}), .sel(rqst_addr[1:0]), .result(lb_out));

    _N_bit_2to1_MUX #(.N(32)) Mux_Data_Out (.data1(lw_out), .data2(lb_out), .sel(funct), .result(read_data_out));//////

    ////////////////////////////////////////////////////// To main memosry
//    always @(posedge done) begin // NEW // PLEASE take care of this delay
//        #1
//        pos_done = 1'b1;
//        #2
//        pos_done = 1'b0;
//    end

    always @ (posedge done or write_in or addr_prepared or funct or rqst_addr or read_data_in or write_data_in) begin
        // after the write from main memory, the negedge done will stimulate the always and write the data into cache again
        #2
        // $display("physical address in cache = %b", rqst_addr);
        // $display("address_prepared = %b", addr_prepared);
        if (addr_prepared)begin
            // $display("enter cache modification -- address prepared");
            if (setIndex == 0) begin
                if (write_in) begin // sw/sb
                // $display("enter cache modification -- write");
                    if (!hit) begin // miss
                    // $display("enter cache modification -- write -- miss");
                        if (LRU[setIndex]==1'b0) begin // 0 means least recently used is block 0
                            // $display("enter cache modification -- write -- miss -- LRU[]=0");
                            // Write data from cache to memory if dirty
                            if (dirty_A) begin i = 31;
                            // $display("enter cache modification -- write -- miss -- LRU[]=0 -- dirty_A");
                                addr_out = {{tagContent_A,setIndex},{4{1'b0}}};
                                write_out = 1'b1;
                                for (i = 31; i <128; i = i + 32) begin
                                    write_data_out = block_A[i-: 32];
                                    @(posedge done) begin
                                        addr_out = addr_out + 4;
                                    end
                                end
                                #5;
                            end i = 31;
                            // Read data from memory to cache anyway
                            addr_out = {rqst_addr[9:4],{4{1'b0}}};
                            write_out = 1'b0;
                            for (i = 31; i <128; i = i + 32) begin
                                @(posedge done) begin
                                    //cache_setA[setIndex][i-: 32] = read_data_in;
                                    cache_setA[block_index][i-: 32] = read_data_in;                                
                                    // $display("enter read data from main memory1");
                                    addr_out = addr_out + 4;
                                    // fetch, mem write 4 times
                                end
                            end
                            cache_setA[block_index][135-1] = 1'b1; // Set Valid to True
                            case(funct)
                                1'b1: begin i = 32 * rqst_addr[3:2] + 8 * rqst_addr[1:0] + 7;
                                        cache_setA[block_index][i-: 8] = write_data_in[7:0]; end // sb
                                default:begin i = 31 + 32 * rqst_addr[3:2];
                                        cache_setA[block_index][i-: 32] = write_data_in; end // sw
                            endcase
                            cache_setA[block_index][135-2] = 1'b1; // Mark cache_setA dirty
                            cache_setA[block_index][(135-3)-: 5] = rqst_addr[9-: 5]; 
                            LRU[setIndex] = 1'b1; // Reset Least Resently Used 
                            // $display("1 modify tag of set A, tag = %b", cache_setA[block_index][(135-3)-: 5]);
                        end 
                        else begin
                            // Write data from cache to memory if dirty
                            if (dirty_B) begin i = 31;
                                #1
                                addr_out = {{tagContent_B,setIndex},{4{1'b0}}};
                                write_out = 1'b1;
                                for (i = 31; i <128; i = i + 32) begin
                                    write_data_out = block_B[i-: 32];
                                    @(posedge done) begin
                                        addr_out = addr_out + 4;
                                    end
                                end
                                #5;
                            end i = 31;
                            // Read data from memory to cache anyway
                            #1
                            addr_out = {rqst_addr[9:4],{4{1'b0}}};
                            write_out = 1'b0;
                            for (i = 31; i <128; i = i + 32) begin
                                @(posedge done) begin
                                    cache_setA[block_index][i-: 32] = read_data_in;
                                    addr_out = addr_out + 4;
                                end
                            end
                            cache_setA[block_index][135-1] = 1'b1; // Set Valid to True
                            case(funct)
                                1'b1: begin i = 32 * rqst_addr[3:2] + 8 * rqst_addr[1:0] + 7;
                                        cache_setA[block_index][i-: 8] = write_data_in[7:0]; end // sb
                                default:begin i = 31 + 32 * rqst_addr[3:2];
                                        cache_setA[block_index][i-: 32] = write_data_in; end // sw
                            endcase
                            cache_setA[block_index][135-2] = 1'b1; // Mark cache_setB dirty
                            cache_setA[block_index][(135-3)-: 5] = rqst_addr[9-: 5];
                            LRU[setIndex] = 1'b0; // Reset Least Resently Used 
                        end
                    end
                    else if (hit_setA) begin
                        case(funct)
                            1'b1: begin i = 32 * rqst_addr[3:2] + 8 * rqst_addr[1:0] + 7;
                                    cache_setA[block_index][i-: 8] = write_data_in[7:0]; end // sb
                            default:begin i = 31 + 32 * rqst_addr[3:2];
                                    cache_setA[block_index][i-: 32] = write_data_in; end // sw
                        endcase
                        cache_setA[block_index][135-2] = 1'b1; // Mark cache_setA dirty
                        cache_setA[block_index][(135-3)-: 5] = rqst_addr[9-: 5];
                        LRU[setIndex] = 1'b1; // Reset Least Resently Used to setB
                        // $display("2 modify tag of set A, tag = %b", cache_setA[block_index][(135-3)-: 5]);
                    end
                    else if (hit_setB) begin
                        case(funct)
                            1'b1: begin i = 32 * rqst_addr[3:2] + 8 * rqst_addr[1:0] + 7;
                                    cache_setA[block_index][i-: 8] = write_data_in[7:0]; end // sb
                            default:begin i = 31 + 32 * rqst_addr[3:2];
                                    cache_setA[block_index][i-: 32] = write_data_in; end // sw
                        endcase
                        cache_setA[block_index][135-2] = 1'b1; // Mark cache_setA dirty
                        cache_setA[block_index][(135-3)-: 5] = rqst_addr[9-: 5];
                        LRU[setIndex] = 1'b0; // Reset Least Resently Used to setA
                    end
                end


                else begin // read_out  // lw/lb
                    if (!hit) begin 
                        if (LRU[setIndex]==1'b0) begin
                            // Write data from cache to memory if dirty
                            if (dirty_A) begin i = 31;
                                addr_out = {{tagContent_A,setIndex},{4{1'b0}}};
                                write_out = 1'b1;
                                for (i = 31; i <128; i = i + 32) begin
                                    write_data_out = block_A[i-: 32];
                                    @(posedge done) begin
                                        addr_out = addr_out + 4;
                                    end
                                end
                                #5;
                            end i = 31;
                            // Read data from memory to cache anyway
                            addr_out = {rqst_addr[9:4],{4{1'b0}}};
                            write_out = 1'b0;
                            for (i = 31; i <128; i = i + 32) begin
                                @(posedge done) begin
                                    cache_setA[block_index][i-: 32] = read_data_in;
                                    addr_out = addr_out + 4;
                                end
                            end
                            cache_setA[block_index][135-2] = 1'b0; // Mark as NOT dirty
                            cache_setA[block_index][135-1] = 1'b1; // Set Valid to True
                            cache_setA[block_index][(135-3)-: 5] = rqst_addr[9-: 5]; 
                            LRU[setIndex] = 1'b1; // Reset Least Resently Used = B
                            // $display("3 modify tag of set A, tag = %b", cache_setA[block_index][(135-3)-: 5]);
                        end
                        else begin
                            // Write data from cache to memory if dirty
                            if (dirty_B) begin i = 31;
                                addr_out = {{tagContent_B,setIndex},{4{1'b0}}};
                                write_out = 1'b1;
                                for (i = 31; i <128; i = i + 32) begin
                                    write_data_out = block_B[i-: 32];
                                    @(posedge done) begin
                                        addr_out = addr_out + 4;
                                    end
                                end
                                #5;
                            end i = 31;
                            // Read data from memory to cache anyway
                            addr_out = {rqst_addr[9:4],{4{1'b0}}};
                            write_out = 1'b0;
                            for (i = 31; i <128; i = i + 32) begin
                                @(posedge done) begin
                                    cache_setA[block_index][i-: 32] = read_data_in;
                                    addr_out = addr_out + 4;
                                end
                            end
                            cache_setA[block_index][135-2] = 1'b0; // Mark as NOT dirty
                            cache_setA[block_index][135-1] = 1'b1; // Set Valid to True 
                            cache_setA[block_index][(135-3)-: 5] = rqst_addr[9-: 5];
                            LRU[setIndex] = 1'b0; // Reset Least Resently Used = A
                        end
                    end
                    else if (hit_setA) begin
                        cache_setA[block_index][(135-3)-: 5] = rqst_addr[9-: 5];
                        // $display("4 modify tag of set A, tag = %b", cache_setA[block_index][(135-3)-: 5]);
                        LRU[setIndex] = 1'b1; // Reset Least Resently Used = B
                    end
                    else if (hit_setB) begin
                        case(funct)
                            1'b1: begin i = 32 * rqst_addr[3:2] + 8 * rqst_addr[1:0] + 7;
                                    cache_setA[block_index][i-: 8] = write_data_in[7:0]; end // lb
                            default:begin i = 31 + 32 * rqst_addr[3:2];
                                    cache_setA[block_index][i-: 32] = write_data_in; end // lw
                        endcase

                        cache_setA[block_index][(135-3)-: 5] = rqst_addr[9-: 5];
                        LRU[setIndex] = 1'b0; // Reset Least Resently Used = A
                    end
                end
            end

            else begin
                if (write_in) begin // sw/sb
                // $display("enter cache modification -- write");
                    if (!hit) begin // miss
                    // $display("enter cache modification -- write -- miss");
                        if (LRU[setIndex]==1'b0) begin
                            // $display("enter cache modification -- write -- miss -- LRU[]=0");
                            // Write data from cache to memory if dirty
                            if (dirty_A) begin i = 31;
                            // $display("enter cache modification -- write -- miss -- LRU[]=0 -- dirty_A");
                                addr_out = {{tagContent_A,setIndex},{4{1'b0}}};
                                write_out = 1'b1;
                                for (i = 31; i <128; i = i + 32) begin
                                    write_data_out = block_A[i-: 32];
                                    @(posedge done) begin
                                        addr_out = addr_out + 4;
                                    end
                                end
                                #5;
                            end i = 31;
                            // Read data from memory to cache anyway
                            addr_out = {rqst_addr[9:4],{4{1'b0}}};
                            write_out = 1'b0;
                            for (i = 31; i <128; i = i + 32) begin
                                @(posedge done) begin
                                    //cache_setA[setIndex][i-: 32] = read_data_in;
                                    cache_setB[block_index][i-: 32] = read_data_in;                                
                                    // $display("enter read data from main memory1");
                                    addr_out = addr_out + 4;
                                end
                            end
                            cache_setB[block_index][135-1] = 1'b1; // Set Valid to True 
                            case(funct)
                                1'b1: begin i = 32 * rqst_addr[3:2] + 8 * rqst_addr[1:0] + 7;
                                        cache_setB[block_index][i-: 8] = write_data_in[7:0]; end // sb
                                default:begin i = 31 + 32 * rqst_addr[3:2];
                                        cache_setB[block_index][i-: 32] = write_data_in; end // sw
                            endcase
                            cache_setB[block_index][135-2] = 1'b1; // Mark cache_setA dirty
                            cache_setB[block_index][(135-3)-: 5] = rqst_addr[9-: 5];
                            LRU[setIndex] = 1'b1; // Reset Least Resently Used 
                            // $display("1 modify tag of set A, tag = %b", cache_setB[block_index][(135-3)-: 5]);
                        end 
                        else begin
                            // Write data from cache to memory if dirty
                            if (dirty_B) begin i = 31;
                                #1
                                addr_out = {{tagContent_B,setIndex},{4{1'b0}}};
                                write_out = 1'b1;
                                for (i = 31; i <128; i = i + 32) begin
                                    write_data_out = block_B[i-: 32];
                                    @(posedge done) begin
                                        addr_out = addr_out + 4;
                                    end
                                end
                                #5;
                            end i = 31;
                            // Read data from memory to cache anyway
                            #1
                            addr_out = {rqst_addr[9:4],{4{1'b0}}};
                            write_out = 1'b0;
                            for (i = 31; i <128; i = i + 32) begin
                                @(posedge done) begin
                                    cache_setB[block_index][i-: 32] = read_data_in;
                                    addr_out = addr_out + 4;
                                end
                            end
                            cache_setB[block_index][135-1] = 1'b1; // Set Valid to True
                            case(funct)
                                1'b1: begin i = 32 * rqst_addr[3:2] + 8 * rqst_addr[1:0] + 7;
                                        cache_setB[block_index][i-: 8] = write_data_in[7:0]; end // sb
                                default:begin i = 31 + 32 * rqst_addr[3:2];
                                        cache_setB[block_index][i-: 32] = write_data_in; end // sw
                            endcase
                            cache_setB[block_index][135-2] = 1'b1; // Mark cache_setB dirty
                            cache_setB[block_index][(135-3)-: 5] = rqst_addr[9-: 5];
                            LRU[setIndex] = 1'b0; // Reset Least Resently Used 
                        end
                    end
                    else if (hit_setA) begin
                        case(funct)
                            1'b1: begin i = 32 * rqst_addr[3:2] + 8 * rqst_addr[1:0] + 7;
                                    cache_setB[block_index][i-: 8] = write_data_in[7:0]; end // sb
                            default:begin i = 31 + 32 * rqst_addr[3:2];
                                    cache_setB[block_index][i-: 32] = write_data_in; end // sw
                        endcase
                        cache_setB[block_index][135-2] = 1'b1; // Mark cache_setA dirty
                        cache_setB[block_index][(135-3)-: 5] = rqst_addr[9-: 5];
                        LRU[setIndex] = 1'b1; // Reset Least Resently Used to setB
                        // $display("2 modify tag of set A, tag = %b", cache_setB[block_index][(135-3)-: 5]);
                    end
                    else if (hit_setB) begin
                        case(funct)
                            1'b1: begin i = 32 * rqst_addr[3:2] + 8 * rqst_addr[1:0] + 7;
                                    cache_setB[block_index][i-: 8] = write_data_in[7:0]; end // sb
                            default:begin i = 31 + 32 * rqst_addr[3:2];
                                    cache_setB[block_index][i-: 32] = write_data_in; end // sw
                        endcase
                        cache_setB[block_index][135-2] = 1'b1; // Mark cache_setA dirty
                        cache_setB[block_index][(135-3)-: 5] = rqst_addr[9-: 5];
                        LRU[setIndex] = 1'b0; // Reset Least Resently Used to setA
                    end
                end


                else begin // read_out  // lw/lb
                    if (!hit) begin 
                        if (LRU[setIndex]==1'b0) begin
                            // Write data from cache to memory if dirty
                            if (dirty_A) begin i = 31;
                                addr_out = {{tagContent_A,setIndex},{4{1'b0}}};
                                write_out = 1'b1;
                                for (i = 31; i <128; i = i + 32) begin
                                    write_data_out = block_A[i-: 32];
                                    @(posedge done) begin
                                        addr_out = addr_out + 4;
                                    end
                                end
                                #5;
                            end i = 31;
                            // Read data from memory to cache anyway
                            addr_out = {rqst_addr[9:4],{4{1'b0}}};
                            write_out = 1'b0;
                            for (i = 31; i <128; i = i + 32) begin
                                @(posedge done) begin
                                    cache_setB[block_index][i-: 32] = read_data_in;
                                    addr_out = addr_out + 4;
                                end
                            end
                            cache_setB[block_index][135-2] = 1'b0; // Mark as NOT dirty
                            cache_setB[block_index][135-1] = 1'b1; // Set Valid to True 
                            cache_setB[block_index][(135-3)-: 5] = rqst_addr[9-: 5];
                            LRU[setIndex] = 1'b1; // Reset Least Resently Used = B
                            // $display("3 modify tag of set A, tag = %b", cache_setB[block_index][(135-3)-: 5]);
                        end
                        else begin
                            // Write data from cache to memory if dirty
                            if (dirty_B) begin i = 31;
                                addr_out = {{tagContent_B,setIndex},{4{1'b0}}};
                                write_out = 1'b1;
                                for (i = 31; i <128; i = i + 32) begin
                                    write_data_out = block_B[i-: 32];
                                    @(posedge done) begin
                                        addr_out = addr_out + 4;
                                    end
                                end
                                #5;
                            end i = 31;
                            // Read data from memory to cache anyway
                            addr_out = {rqst_addr[9:4],{4{1'b0}}};
                            write_out = 1'b0;
                            for (i = 31; i <128; i = i + 32) begin
                                @(posedge done) begin
                                    cache_setB[block_index][i-: 32] = read_data_in;
                                    addr_out = addr_out + 4;
                                end
                            end
                            cache_setB[block_index][135-2] = 1'b0; // Mark as NOT dirty
                            cache_setB[block_index][135-1] = 1'b1; // Set Valid to True 
                            cache_setB[block_index][(135-3)-: 5] = rqst_addr[9-: 5];
                            LRU[setIndex] = 1'b0; // Reset Least Resently Used = A
                        end
                    end
                    else if (hit_setA) begin
                        cache_setB[block_index][(135-3)-: 5] = rqst_addr[9-: 5];
                        // $display("4 modify tag of set A, tag = %b", cache_setB[block_index][(135-3)-: 5]);
                        LRU[setIndex] = 1'b1; // Reset Least Resently Used = B
                    end
                    else if (hit_setB) begin
                        cache_setB[block_index][(135-3)-: 5] = rqst_addr[9-: 5];
                        LRU[setIndex] = 1'b0; // Reset Least Resently Used = A
                    end
                end
            end
        end
    end  
          

endmodule

module _N_bit_4to1_MUX #(
    parameter   N = 32
)(
    input       [1:0]       sel,
    input       [N-1:0]     data1, 
    input       [N-1:0]     data2, 
    input       [N-1:0]     data3, 
    input       [N-1:0]     data4,
    
    output reg  [N-1:0]     result
);

    initial begin
        result = 0;
    end

    always @ (*) begin
        case (sel)
            2'b00:  result = data1;
            2'b01:  result = data2;
            2'b10:  result = data3;
            2'b11:  result = data4;
            default:    result = 0;
        endcase
    end
    
endmodule

module _N_bit_2to1_MUX #(
    parameter   N = 32
)(
    input                   sel,
    input       [N-1:0]     data1, 
    input       [N-1:0]     data2, 
    
    output reg  [N-1:0]     result
);

    initial begin
        result = 0;
    end

    always @ (*) begin
        case (sel)
            1'b0: result = data1;
            1'b1: result = data2;
            default:    result = 0;
        endcase
    end
    
endmodule
