// Data Memory
module DataMemory(
    input [31:0] address,
    input [31:0] writedata,
    input [2:0] fun3,
    input memread,
    input memwrite,
    output reg [31:0] readdata
);
    integer i = 0;
    reg [7:0] data [127:0];
    initial 
        begin  
            readdata<=0;
            for(i=0;i<128;i=i+1)  
                data[i] <= 8'b00000000;  
        end  
        always @(*)
        begin  
           if (memwrite) //write 
            begin
                case(fun3)
                  3'b010:begin
                   data[address] <= writedata[7:0];  
                   data[address+1] <= writedata[15:8];  
                   data[address+2] <= writedata[23:16];  
                   data[address+3] <= writedata[31:24];  
                  end
                  3'b000:begin
                    data[address] <= writedata[7:0];
                  end
                  default:data[address] <=data [address];
                endcase
             end
           end
           
           always @ (*) begin
           if (memread) begin//read
            case (fun3)
                3'b010: begin //load word
                    readdata <= {data[address+3], data[address+2], data[address+1], data[address]};
                end
                
                3'b000: begin //load byte
                    readdata <= {{24{data[address][7]}}, data[address]};
                end
                3'b100: begin //load byte unsigned
                    readdata <= {{24{1'b0}}, data[address]};
                end
                default: readdata<= 0;
            endcase
         end
        end
endmodule//done
