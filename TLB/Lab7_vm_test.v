`timescale 1ns / 1ps
`include "New_Cache.v"
`include "Translation_Look_Aside_Buffer.v"
`include "Page_Table.v"
`include "Main_Mem.v"

module vm_test;
    reg          clock;
    
    // interface between cache and CPU
    wire         write_read;
    wire [31:0]  write_data_in_cache, read_data_out_cache;

    // interface between cache and main memory
    wire         done_tlb, done_cache, write_read_mem ;
    wire [31:0]  write_data_out_mem, read_data_in_mem;
    wire [9:0]   address_mem;

    //tlb
    wire         Addr_prepared, write_to_table;
    wire [5:0]   V_addr_PT_out;
    wire [1:0]   P_addr_PT_out;
    wire [9:0]   physical_address;
    //page table
    wire [1:0]   P_page_num;
    //cpu
    wire [13:0]  virtual_address;

    processor                     CPU(
        .hit_miss(hit),
        .clock(clock),
        .read_write(write_read),
        .address(virtual_address),
        .write_data(write_data_in_cache)
    );
    /*
        You can only modify the following parts:
        CACHE, TLB, MAIN_MEMORY, and PAGE_TABLE
        to adapt modules you designed to this testbench
    */
    New_Cache   cache(
        .done(done_cache),
        .write_in(write_read),
        .addr_prepared(Addr_prepared),//tlb
        .funct(physical_address[0]),
        .rqst_addr(physical_address),//tlb
        .read_data_in(read_data_in_mem),
        .write_data_in(write_data_in_cache),
                
        .hit(hit),
        .write_out(write_read_mem),
        .read_data_out(read_data_out_cache), 
        .write_data_out(write_data_out_mem),
        .addr_out(address_mem)         
    );
    translation_look_aside_buffer TLB(
        .done(done_tlb),
        .Virtual_addr(virtual_address),
        .P_addr_PT_in(P_page_num),
        
        .TLB_hit(TLB_hit),
        .write_to_table(write_to_table),
        .V_addr_PT_out(V_addr_PT_out),
        .P_addr_PT_out(P_addr_PT_out),
        .Physical_addr(physical_address),
        .Addr_prepared(Addr_prepared)
    );

    main_mem                      memory(
        .read_or_write(write_read_mem),
        .address(address_mem),
        .write_data(write_data_out_mem),
        .read_data(read_data_in_mem),
        .done(done_cache)         
    );
    
    page_table                    PT(
        .read_from_TLB(write_to_table),
        .phy_page_num_in(P_addr_PT_out),
        .vir_page_num_in(V_addr_PT_out),
        
        .done(done_tlb),
        .phy_page_num(P_page_num),
        .page_fault(page_fault)
    );

    /*
        Do not modify the following code!!!
    */
    always #5 clock = ~clock;

    integer file;

    initial begin
        file = $fopen("result_lab7.txt", "w");
        $dumpfile("vm_test.vcd");
        $dumpvars(0, vm_test);
    end

    always @(posedge clock) begin
        $fdisplay(file,"Request %d: ", CPU.request_num);
        $fdisplay(file,"page fault: %b", PT.page_fault);
        $fdisplay(file,"data read posedge: %H", read_data_out_cache);
        $fdisplay(file,"contents in TLB: ");
        $fdisplay(file,"block 00: tag: %2d, valid: %b, dirty: %b, reference: %b, VPN: %1d", TLB.TLB[0][7:2], TLB.TLB[0][11], TLB.TLB[0][10], TLB.TLB[0][9:8], TLB.Virtual_addr[13:8]);
        $fdisplay(file,"block 01: tag: %2d, valid: %b, dirty: %b, reference: %b, VPN: %1d", TLB.TLB[1][7:2], TLB.TLB[1][11], TLB.TLB[1][10], TLB.TLB[1][9:8], TLB.Virtual_addr[13:8]);
        $fdisplay(file,"block 10: tag: %2d, valid: %b, dirty: %b, reference: %b, VPN: %1d", TLB.TLB[2][7:2], TLB.TLB[2][11], TLB.TLB[2][10], TLB.TLB[2][9:8], TLB.Virtual_addr[13:8]);
        $fdisplay(file,"block 11: tag: %2d, valid: %b, dirty: %b, reference: %b, VPN: %1d", TLB.TLB[3][7:2], TLB.TLB[3][11], TLB.TLB[3][10], TLB.TLB[3][9:8], TLB.Virtual_addr[13:8]);
        $fdisplay(file,"contents in cache: ");
        $fdisplay(file,"block 00: tag: %b, valid: %b, dirty: %b, word0: %H, word1: %H, word2: %H, word3: %H", cache.cache_setA[0][132:128], cache.cache_setA[0][134], cache.cache_setA[0][133], cache.cache_setA[0][31-:32], cache.cache_setA[0][63-:32], cache.cache_setA[0][95-:32], cache.cache_setA[0][127-:32]);
        $fdisplay(file,"block 01: tag: %b, valid: %b, dirty: %b, word0: %H, word1: %H, word2: %H, word3: %H", cache.cache_setA[1][132:128], cache.cache_setA[1][134], cache.cache_setA[1][133], cache.cache_setA[1][31-:32], cache.cache_setA[1][63-:32], cache.cache_setA[1][95-:32], cache.cache_setA[1][127-:32]);
        $fdisplay(file,"block 10: tag: %b, valid: %b, dirty: %b, word0: %H, word1: %H, word2: %H, word3: %H", cache.cache_setB[0][132:128], cache.cache_setB[0][134], cache.cache_setB[0][133], cache.cache_setB[0][31-:32], cache.cache_setB[0][63-:32], cache.cache_setB[0][95-:32], cache.cache_setB[0][127-:32]);
        $fdisplay(file,"block 11: tag: %b, valid: %b, dirty: %b, word0: %H, word1: %H, word2: %H, word3: %H", cache.cache_setB[1][132:128], cache.cache_setB[1][134], cache.cache_setB[1][133], cache.cache_setB[1][31-:32], cache.cache_setB[1][63-:32], cache.cache_setB[1][95-:32], cache.cache_setB[1][127-:32]);
    end
    
    initial begin
        clock = 0;
        #400 $finish;
    end
endmodule
/*
    Do not modify the following code!!!
*/
module processor (
    input  hit_miss,
    input  clock,
    output read_write,
    output [13:0] address,
    output [31:0] write_data
);
    parameter  request_total = 14; // change this number to how many requests you want in your testbench
    reg [4:0]  request_num;
    reg        read_write_test[request_total-1:0];
    reg [13:0]  address_test[request_total-1:0];
    reg [31:0] write_data_test[request_total-1:0]; 
    initial begin
        request_num = 0;
        read_write_test[0]  = 1; address_test[0]  = 14'b000100_100_0_1000; write_data_test[0]  = 1;       // sw, virtual page  4, TLB miss, mapped to physical page 2, physical tag 10100, cache miss in set 0 block 0,
        read_write_test[1]  = 1; address_test[1]  = 14'b000000_100_1_1100; write_data_test[1]  = 12'hdac; // sw, virtual page  0, TLB miss, mapped to physical page 1, physical tag 01100, cache miss in set 1 block 0,
        read_write_test[2]  = 1; address_test[2]  = 14'b000001_100_1_1000; write_data_test[2]  = 12'hfac; // sw, virtual page  1, TLB miss, mapped to physical page 3, physical tag 11100, cache miss in set 1 block 1,
        read_write_test[3]  = 1; address_test[3]  = 14'b000000_100_1_0101; write_data_test[3]  = 12'hfac; // sb, virtual page  0, TLB hit,  mapped to physical page 1, physical tag 01100, cache hit  in set 1 block 0,
        read_write_test[4]  = 0; address_test[4]  = 14'b000111_100_1_0101; write_data_test[4]  = 0;       // lb, virtual page  7, TLB miss, mapped to physical page 1, physical tag 01100, cache hit  in set 1 block 0,
        read_write_test[5]  = 0; address_test[5]  = 14'b001000_110_1_0101; write_data_test[5]  = 0;       // lb, virtual page  8, TLB miss, mapped to physical page 1, virtual page 4 replaced, write back entry with virtual tag 4,
                                                                                                          //                                                           physical tag 01110, cache miss in set 1, set 1 block 1 replaced and write back
        read_write_test[6]  = 0; address_test[6]  = 14'b000001_110_1_0100; write_data_test[6]  = 0;       // lw, virtual page  1, TLB hit,  mapped to physical page 3, physical tag 11110, cache miss in set 1, set 1 block 0 replaced and write back
        read_write_test[7]  = 1; address_test[7]  = 14'b000111_100_1_0111; write_data_test[7]  = 12'h148; // sb, virtual page  7, TLB hit,  mapped to physical page 1, physical tag 01100, cache miss in set 1, set 1 block 1 replaced
        read_write_test[8]  = 0; address_test[8]  = 14'b000000_100_1_1000; write_data_test[8]  = 0;       // lw, virtual page  0, TLB hit,  mapped to physical page 1, physical tag 01100, cache hit  in set 1 block 1,
        read_write_test[9]  = 0; address_test[9]  = 14'b001010_100_1_0100; write_data_test[9]  = 0;       // lw, virtual page 10, TLB miss, mapped to physical page 1, virtual page 8 replaced, write back entry with virtual tag 8,
                                                                                                          //                                                           physical tag 01100, cache hit  in set 1 block 1,
        read_write_test[10] = 0; address_test[10] = 14'b000000_110_1_0100; write_data_test[10] = 0;       // lw, virtual page  0, TLB hit,  mapped to physical page 1, physical tag 01110, cache miss in set 1, set 1 block 1 replaced
        read_write_test[11] = 0; address_test[11] = 14'b000100_100_0_1000; write_data_test[11] = 0;       // lw, virtual page  4, TLB miss, mapped to physical page 2, virtual page 1 replaced, write back entry with virtual tag 1,
                                                                                                          //                                                           physical tag 10100, cache hit  in set 1 block 0
        read_write_test[12] = 0; address_test[12] = 14'b000010_110_1_0100; write_data_test[12] = 0;       // lw, virtual page  2, TLB miss, page fault

        /* extra test for fun; it is acceptable that you have different result after the request below */
        read_write_test[13] = 0; address_test[13] = 14'b000111_100_1_1100; write_data_test[13] = 0;       // lw, virtual page  10, TLB hit, mapped to physical page 1, physcial tag 01100, cache hit in set 1 block 1
        // Notes: actually in this lab you are not required to handle page fault, but ideally you may just skip the request with page fault and deal with the next request normally (it only applies to this lab!!!)
        // In other words, nothing should be changed when there is a page fault in this lab, including TLB, page table, cache and memory.
        // But such requirement is cancelled considering your workload :)
    end
    always @(posedge clock) begin
        #1
        if (hit_miss == 1) request_num = request_num + 1;
        else request_num = request_num;
    end
    assign address      = address_test[request_num];
    assign read_write   = read_write_test[request_num];
    assign write_data   = write_data_test[request_num]; 
endmodule

