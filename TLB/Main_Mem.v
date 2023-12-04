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
            main_memory[i] = 0; 
        end
		main_memory[0] = 32'h28d;
		// modify from 55555555 to 28d
		main_memory[1] = 32'hF;
		main_memory[2] = 32'hE;
		main_memory[3] = 32'hA;
		main_memory[16] = 32'hCCCCCCCC;
		main_memory[17] = 32'hEEEEEEEE;
		main_memory[18] = 32'h55555555;
		main_memory[19] = 32'hBBBBBBBB;
		main_memory[67] = 32'h11111111; 
		main_memory[66] = 32'h22222222; 
		main_memory[65] = 32'h33333333; 
		main_memory[64] = 32'h44444444;
		main_memory[83] = 32'h281; 
		main_memory[82] = 32'h285; 
		main_memory[81] = 32'h0; 
		main_memory[80] = 32'h28d;
		main_memory[87] = 32'h281;
		main_memory[86] = 32'h285;
		main_memory[85] = 32'h0;
		main_memory[84] = 32'h28d;


		main_memory[100] = 32'h191;
		main_memory[101] = 32'h199;
		main_memory[102] = 32'h195;
		main_memory[103] = 32'h191;

		main_memory[116] = 32'h1dd;
		main_memory[117] = 32'h1d9;
		main_memory[118] = 32'h1d5;
		main_memory[119] = 32'h1d1;

		main_memory[132] = 32'h88888888;
		main_memory[133] = 32'h77777777;
		main_memory[134] = 32'h66666666;
		main_memory[135] = 32'h55555555;

		main_memory[160] = 32'h28d;
		main_memory[161] = 32'h285;
		main_memory[162] = 32'h0;
		main_memory[163] = 32'h281;

		main_memory[192] = 32'h88888888;
		main_memory[193] = 32'h77777777;
		main_memory[194] = 32'h66666666;
		main_memory[195] = 32'h55555555;
		main_memory[196] = 32'h88888888;
		main_memory[197] = 32'h77777777;
		main_memory[198] = 32'h66666666;
		main_memory[199] = 32'h55555555;

		main_memory[228] = 32'h39d;
		main_memory[229] = 32'h395;
		main_memory[230] = 32'h0;
		main_memory[231] = 32'h391;
		main_memory[244] = 32'h3dd;
		main_memory[245] = 32'h3d9;
		main_memory[246] = 32'h3d5;
		main_memory[247] = 32'h3d1;

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
