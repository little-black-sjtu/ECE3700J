`timescale 1ns / 1ps

module CPU (
    input  hit_miss, // 1 for hit
    input  clock,
    input  [31:0]Read_Data,
    output read_write, // 1 for write
    output if_lb, // 1 for lb
    output [9:0] address,
    output [31:0] write_data
);
    parameter  request_total = 12; 
    reg [4:0]  request_num;
    reg        read_write_test[request_total-1:0];
    reg        if_lbs[request_total-1:0];
    reg [9:0]  address_test[request_total-1:0];
    reg [31:0] write_data_test[request_total-1:0]; 
    initial begin
        /* test cases */
        #21  // we wait for around 2 periods to start, because we should wait the initialization of memory 
        request_num = 0;
        read_write_test[0] = 1; if_lbs[0] = 0; address_test[0] = 10'b0110101000; write_data_test[0] = 10'h3ab; //write miss + empty dirty=0
        read_write_test[1] = 0; if_lbs[1] = 0; address_test[1] = 10'b0110101000; write_data_test[1] = 0; //                                                h3ab
        read_write_test[2] = 1; if_lbs[2] = 0; address_test[2] = 10'b0110101000; write_data_test[2] = 10'h3ac;//write hit
        read_write_test[3] = 0; if_lbs[3] = 0; address_test[3] = 10'b0110101000; write_data_test[3] = 0;//read hit                                         h3ac
        read_write_test[4] = 0; if_lbs[4] = 0; address_test[4] = 10'b0100001000; write_data_test[4] = 0;//read miss + empty dirty=0   block 0              0000
        read_write_test[5] = 0; if_lbs[5] = 0; address_test[5] = 10'b0100101000; write_data_test[5] = 0;//read miss + write back dirty=1                   0000
        read_write_test[6] = 0; if_lbs[6] = 0; address_test[6] = 10'b0110101000; write_data_test[6] = 0;//check write back dirty=0                         h3ac 
        read_write_test[7] = 1; if_lbs[7] = 0; address_test[7] = 10'b0110101000; write_data_test[7] = 10'h3ad;//write hit
        read_write_test[8] = 1; if_lbs[8] = 0; address_test[8] = 10'b0101101000; write_data_test[8] = 10'h3ae;//write miss + write back dirty=1
        read_write_test[9] = 0; if_lbs[9] = 0; address_test[9] = 10'b0101101000; write_data_test[9] = 0;//check write                                      h3ae
        read_write_test[10] = 0; if_lbs[10] = 0; address_test[10] = 10'b0110101000; write_data_test[10] = 0;//check write back dirty=0                      h3ad
        read_write_test[11] = 0; if_lbs[11] = 1; address_test[11] = 10'b0110101001; write_data_test[11] = 0;//check load byte                               h003              
    end
    always @(posedge clock) begin
        #1  // we give 1 delay to asynchronize edges, this is to avoid the "before edge" or "after edge" ambiguity
        if (hit_miss == 1) request_num <= request_num + 1;
        else request_num <= request_num;
    end
    assign address      = address_test[request_num];
    assign read_write   = read_write_test[request_num];
    assign if_lb        = if_lbs[request_num];
    assign write_data   = write_data_test[request_num]; 
endmodule