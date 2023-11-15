module Hazard_detection(
    input [4:0]ifid_r1,
    input [4:0]ifid_r2,
    input [4:0]idex_rd,
    input [4:0]exmem_rd,
    input idex_mem_read,
    //input idex_mem_write,
    input exmem_mem_read,
    input whether_branch,
    input id_mem_write,
    input idex_regwrite,

    output reg ifid_write,
    // output reg whether_hazard,
    output reg PC_write,
    output reg whether_hazard
);

    initial begin
        PC_write=1'b1; ifid_write=1'b1; whether_hazard=1'b1;
    end

    always @(*) begin 
        if (idex_mem_read && !id_mem_write) begin
            if (idex_rd == ifid_r1 || idex_rd == ifid_r2) begin
                PC_write=1'b0; ifid_write=1'b0; whether_hazard=1'b1;
            end
            else begin
                PC_write=1'b1; ifid_write=1'b1; whether_hazard=1'b0;
            end  
        end
        else if (whether_branch && idex_regwrite) begin
            if ((idex_rd!=0) && (ifid_r1 == idex_rd || ifid_r2 == idex_rd)) begin
                PC_write=1'b0; ifid_write=1'b0; whether_hazard=1'b1;
            end
            else begin
                PC_write=1'b1; ifid_write=1'b1; whether_hazard=1'b0;
            end  
        end
        else if (whether_branch && exmem_mem_read) begin
            if ((exmem_rd!=0) && (ifid_r1 == exmem_rd || ifid_r2 == exmem_rd)) begin
                PC_write=1'b0; ifid_write=1'b0; whether_hazard=1'b1;
            end
            else begin
                PC_write=1'b1; ifid_write=1'b1; whether_hazard=1'b0;
            end  
        end
        else begin
            PC_write=1'b1; ifid_write=1'b1; whether_hazard=1'b0;
        end
    end
endmodule