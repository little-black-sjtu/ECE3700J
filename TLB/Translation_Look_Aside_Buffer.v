module translation_look_aside_buffer
// (
//     parameter   V_addr_width = 14,
//     parameter   P_addr_width = 10,
//     parameter   Page_offset  = 8,
//     parameter   TLB_row_num  = 4,
//     ////////////////////////////////////////////////////// You don't need to modify the following.~
//     parameter   VPN_size = V_addr_width - Page_offset,  // 6
//     parameter   PPN_size = P_addr_width - Page_offset,  // 2
//     parameter   V = VPN_size, P = PPN_size, VA = V_addr_width, PA = P_addr_width
// )
(
    input                           done,               // From PageTable, 1'b1 for done, 1'b0 for unfinished
    input       [13:0]      Virtual_addr,               // From Processor
    input       [1:0]       P_addr_PT_in,               // From PageTable

    output                       TLB_hit,               // To Processor, 1'b1 for hit, 1'b0 for miss 
    output reg            write_to_table,               // To PageTable, 1'b1 for write, 1'b0 for read
    output reg  [5:0]      V_addr_PT_out,               // To PageTable
    output reg  [1:0]      P_addr_PT_out,               // To PageTable
    output reg  [9:0]      Physical_addr,               // To Cache Mem
    output reg             Addr_prepared                // To Cache Mem
);
    reg         [11:0]      TLB     [3:0];              // 1bit_Valid = TLB[11], 1bit_Dirty = TLB[10], 2bit_Ref = TLB[9:8];
                                                        // Tag = TLB[7:2]; Physical Page Num = TLB[1:0];
    ////////////////////////////////////////////////////// 4X4-Byte TLB
    
    wire        [3:0]       Hit;                        // whether [Tag == VPN] && Valid in this rowS
    wire        [5:0]       VPN;                        // Data of Virtual Page Number, in Processor
    
    ////////////////////////////////////////////////////// Initialization
   
    integer i, j, k;
    initial begin
        V_addr_PT_out = 0; P_addr_PT_out = 0; 
        Physical_addr = 0; Addr_prepared = 0;
        write_to_table = 0;
        for (i = 0; i < 4; i = i + 1) begin///
            TLB[i] = 12'b0;
            TLB[i][9:8]=i;
        end
    end
    
    assign VPN = Virtual_addr[13: 8];
    assign Hit[0] = (VPN == TLB[0][7:2]) && TLB[0][11];
    assign Hit[1] = (VPN == TLB[1][7:2]) && TLB[1][11];
    assign Hit[2] = (VPN == TLB[2][7:2]) && TLB[2][11];
    assign Hit[3] = (VPN == TLB[3][7:2]) && TLB[3][11];

    ////////////////////////////////////////////////////// To PageTable
    or (TLB_hit, Hit[0], Hit[1], Hit[2], Hit[3]);
    
    always @(*) begin
        if (!TLB_hit) begin 
            Addr_prepared = 1'b0;
            for (i = 0; i < 4; i = i + 1) begin // LRU_sel
                if (TLB[i][9:8] == 2'b11) begin
                    // Write physical_addr from TLB to Page Table if dirty
                    if (TLB[i][10]) begin // If the certain block is dirty
                        V_addr_PT_out = TLB[i][7:2];
                        P_addr_PT_out = TLB[i][1:0];
                        write_to_table = 1'b1;
                    end 
                    // Read physical_addr from Page Table to TLB anyway
                    write_to_table = 1'b0;
                    V_addr_PT_out = VPN;
                    # 1
                    if (done) begin
                        TLB[i][1:0] = P_addr_PT_in;
                        TLB[i][7:2] = VPN; // Reset P_addr in TLB[i]
                    end
                    TLB[i][10] = 1'b0; // Mark as NOT dirty
                    TLB[i][11] = 1'b1; // Set Valid to True 
//                    for (j = 0; j < TLB_row_num; j = j + 1) begin // LRU_sel
//                        TLB[j][9:8] = TLB[j][9:8] + 2'b01;
//                    end
//                    TLB[i][9:8] = 2'b00; // Reset 2-bit LRU 
                end
            end
        end
        ////////////////////////////////////////////////////// To Cache Mem
        else begin
            Physical_addr[7:0] = Virtual_addr[7:0];
            case (Hit)
                4'b0001:  begin Physical_addr[9:8] = TLB[0][1:0];
                    k = 0;
                end
                4'b0010:  begin Physical_addr[9:8] = TLB[1][1:0];
                    k = 1;
                end
                4'b0100:  begin Physical_addr[9:8] = TLB[2][1:0];
                    k = 2;
                end
                4'b1000:  begin Physical_addr[9:8] = TLB[3][1:0];
                    k = 3;
                end
                default:  Physical_addr[9:8] = Physical_addr[9:8];
            endcase
            Addr_prepared = 1'b1;
            for (j = 0; j < 4; j = j + 1) begin
                if (TLB[j][9:8] < TLB[k][9:8]) begin
                    TLB[j][9:8] = TLB[j][9:8] + 2'b01;
                end
            end
            TLB[k][9:8] = 2'b00;
        end
    end

endmodule
