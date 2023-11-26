`timescale 1ns / 1ps
`include "cache.v"
`include "mainDataMemory.v"
`include "Lab6_cpu.v"

module top( input clk);
    wire read_write_cache; /* 1 if write, 0 if read */
    wire hit_miss; /* 1 if hit, 0 if miss */
    wire [9:0]   address_cache;
    wire [31:0]  read_data_cache, write_data_cache;
    // interface between cache and main memory
    wire [127:0]  write_data_mem,read_data_mem;
    wire [9:0]   address_mem;
    wire read_write_mem,Done,if_lb;
    // You may add the signal you need. However, you cannot change the signals above.

    cache   cache(.read_writeIn(read_write_cache), .TargetAddressIn(address_cache), .WriteDataIn(write_data_cache), .doneFromMain(Done), .ReadDataFromMain(read_data_mem)
                  , .ReadDataOut(read_data_cache), .hit_miss(hit_miss), .read_writeOut(read_write_mem), .TargetAdressOut(address_mem), .WriteDataOut (write_data_mem), .if_lb(if_lb));
    mainDataReg            mem_db(.read_writeIn(read_write_mem), .TargetAddress(address_mem), .WriteData(write_data_mem), .ReadData(read_data_mem), .done(Done));
    CPU                CPU_db(.hit_miss(hit_miss), .clock(clk),.Read_Data(read_data_cache),.read_write(read_write_cache), .address(address_cache), .write_data(write_data_cache), .if_lb(if_lb));
endmodule

module CacheTest;
    reg clock;

    parameter half_period = 5;
    integer t = 0;
    
    top test(clock);
    
    initial begin
        #0 clock = 0;
    end
    
    always #half_period begin
        clock = ~clock;
        t=t+1;
    end
    integer file;

    initial begin
        file = $fopen("result_lab6.txt", "w");
        $dumpfile("CacheTest.vcd");
        $dumpvars(0, CacheTest);
    end

    always @(posedge clock) begin
            $fdisplay(file, "===============================================");
            $fdisplay(file, "Clock cycle %d", t/2);
            $fdisplay(file, "Read data = %H", test.read_data_cache);
            $fdisplay(file, "hit_miss = %d", test.hit_miss);
            $fdisplay(file, "read_write_cache = %d", test.read_write_cache);
            $fdisplay(file, "Request number = %d", test.CPU_db.request_num);
            $fdisplay(file, "Cache[2][133:128] = %b", test.cache.CacheReg[2][133:128]);
            $fdisplay(file, "===============================================");
    end
    initial
        #300 $stop;


endmodule