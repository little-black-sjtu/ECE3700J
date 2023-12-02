module main_mem (
    //1'b1 for read, 1'b0 for write
	input			    read_or_write,
	input       [9:0]	address,
	input       [31:0]	write_data,
                   
    output reg          done,
	output reg  [31:0]  read_data

);
	reg 	    [31:0] 	main_memory [255:0];

	integer i;
	initial begin
		//wait for initial data
		for(i = 0; i < 256; i = i + 1) begin 
            main_memory[i] = 4*i+1; 
        end
		done = 1'b0;
	end
	
	//first read
	always @(read_or_write or address) begin
	    #3
	    repeat (4) begin
            if (read_or_write == 1'b1) begin
                main_memory[address[9:2]] = write_data[31:0];
            end
	
    //then write
            if (read_or_write == 1'b0) begin
                read_data[31:0] = main_memory[address[9:2]];
            end
		    #2 done = 1'b1;
		    #2 done = 1'b0;
        end 
	end

endmodule
