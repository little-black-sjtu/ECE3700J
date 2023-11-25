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
    wire [31:0]  write_data_mem,read_data_mem;
    wire [9:0]   address_mem;
    wire read_write_mem,Done;
    // You may add the signal you need. However, you cannot change the signals above.

    cache   Cache(.read_writeIn(read_write_cache), .TargetAddressIn(address_cache), .WriteDataIn(write_data_cache), .doneFromMain(Done), .ReadDataFromMain(read_data_mem)
                  , .ReadDataOut(read_data_cache), .hit_miss(hit_miss), .read_writeOut(read_write_mem), .TargetAdressOut(address_mem), .WriteDataOut (write_data_mem));
    mainDataMemory            mem_db(.read_writeIn(read_write_mem), .TargetAddress(address_mem), .WriteData(write_data_mem), .ReadData(read_data_mem), .done(Done));
    lab6_cpu                 CPU_db(.hit_miss(hit_miss), .clock(clk),.Read_Data(read_data_cache),.read_write(read_write_cache), .address(address_cache), .write_data(write_data_cache));
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
    
    always @(posedge clock) begin
            $display("===============================================");
            $display("Clock cycle %d", t/2);
            $display("Read data = %H", test.read_data_cache);
            $display("hit_miss = %d", test.hit_miss);
            $display("===============================================");
    end
    initial
        #200 $stop;

endmodule