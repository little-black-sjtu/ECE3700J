// Forward_unit
module Forward_unit
(
    input [4:0] idex_r1,
    input [4:0] idex_r2,
    input [4:0] ifid_r1,
    input [4:0] ifid_r2,
    input [4:0] exmem_r2, //New_Bug_05
    
    input [4:0] memwb_reg_rd,
    input [4:0] exmem_reg_rd,
    input [4:0] idex_reg_rd,
    
    input memwb_reg_write, 
    input exmem_reg_write,
    input exmem_mem_write,

    input exmem_mem_read,
    input memwb_mem_read,
    
    input idex_mem_read,
    input idex_mem_write,
    input idex_reg_write,
    input ifid_branch,
    
    output reg [1:0] o_forward_a,
    output reg [1:0] o_forward_b,
    output reg o_forward_eq1,
    output reg o_forward_eq2,
    output reg o_mem_src
    );
    
    initial begin
        o_forward_a = 2'b00;
        o_forward_b = 2'b00;
        o_forward_eq1 = 1'b0;
        o_forward_eq2 = 1'b0;
        o_mem_src = 1'b0; //NewBug_08
    end
    
    always @(*) begin
    //ForwardA
        //EX
        if (exmem_reg_write && (exmem_reg_rd!=0) && exmem_reg_rd == idex_r1 
            && !exmem_mem_read && !idex_mem_read && !idex_mem_write) //NewBug_16
            o_forward_a = 2'b10;
        //MEM
        else if (memwb_reg_write && (memwb_reg_rd!=0) && memwb_reg_rd == idex_r1)
            o_forward_a = 2'b01;
        else
            o_forward_a = 2'b00;
    
    //ForwardB
         //EX
        if (exmem_reg_write && (exmem_reg_rd!=0) && exmem_reg_rd == idex_r2 
            && !exmem_mem_read && !idex_mem_read && !idex_mem_write) //NewBug_06
            o_forward_b = 2'b10;
         //MEM
        else if (memwb_reg_write && (memwb_reg_rd!=0) && memwb_reg_rd == idex_r2) 
            o_forward_b = 2'b01;
        else
            o_forward_b = 2'b00;
   
    //Memsrc
        if (memwb_reg_rd == exmem_r2 && exmem_mem_write) //NewBug_03
            o_mem_src = 1'b1;
        else
            o_mem_src = 1'b0;

    //ForwardEq
        if (exmem_reg_write && (exmem_reg_rd!=0) && exmem_reg_rd == ifid_r1 && ifid_branch)
            o_forward_eq1 = 1'b1;
        else
            o_forward_eq1 = 1'b0;
        
        if (exmem_reg_write && (exmem_reg_rd!=0) && exmem_reg_rd == ifid_r2 && ifid_branch)
            o_forward_eq2 = 1'b1; //NewBug_07
        else
            o_forward_eq2 = 1'b0; //NewBug_07

    end

endmodule
