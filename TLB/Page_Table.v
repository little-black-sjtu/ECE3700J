
module page_table (
    //1'b1 for read, 1'b0 for write
	input			    read_from_TLB,
	input       [1:0]	phy_page_num_in,
    input       [5:0]	vir_page_num_in,
                   
    output reg          done,
	output reg  [1:0]   phy_page_num,

    output reg          page_fault
);
	reg 	    [4:0] 	page_table [63:0];

	integer i;
	initial begin
		//wait for initial data
		for(i = 0; i < 64; i = i + 1) begin 
           // page_table[i][2] = 0;  // ref
           // page_table[i][3] = 0;  // dirty
           // page_table[i][4] = 0;  // valid
           page_table[i] = 0;
        end
        
        page_table[4][1:0] = 2;
        page_table[0][1:0] = 1;
        page_table[1][1:0] = 3;
        page_table[7][1:0] = 1;
        page_table[8][1:0] = 1;
        page_table[10][1:0] = 1;
        
        page_table[4][4] = 1;
        page_table[0][4] = 1;
        page_table[1][4] = 1;
        page_table[7][4] = 1;
        page_table[8][4] = 1;
        page_table[10][4] = 1;

		done = 1'b0; page_fault = 1'b0;

        //display page table
        // for (i = 0; i < 64; i = i + 1) begin
        //     $display("page_table[%d] = %b", i, page_table[i]);
        // end
	end
	
	always @(read_from_TLB or phy_page_num_in or vir_page_num_in) begin
        $display("enter page_table modification");
	//always @(*) begin
	    # 0.75
            if (read_from_TLB == 1'b1) begin
                page_table[vir_page_num_in][1:0] = phy_page_num_in;
                page_table[vir_page_num_in][2] = 1;  // ref
                page_table[vir_page_num_in][3] = 1;  // dirty
                $display("read from TLB page_table[%d] = %b", vir_page_num_in, page_table[vir_page_num_in]);
            end
	
            if (read_from_TLB == 1'b0) begin
                //phy_page_num = main_memory[vir_page_num_in][1:0];
                phy_page_num = page_table[vir_page_num_in][1:0];
                page_table[vir_page_num_in][2] = 1;  // ref
                page_fault=1-page_table[vir_page_num_in][4];//
                $display("write to TLB page_table[%d] = %b", vir_page_num_in, page_table[vir_page_num_in]);
            end
		    done = 1'b1;
		    #1 done = 1'b0;
	end

endmodule
