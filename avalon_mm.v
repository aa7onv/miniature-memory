// exposes dist1_cm and dist2_cm signals as read-only registers,
// addressable over the HPS over the HPS-to-FPGA bridge 

module avalon_mm (
    input clk,
    input reset,

    // Avalon-MM slave interface for platform designer
    input [0:0] address,
    input read,
    output reg [31:0] readdata, //avalon always 32 bits

    //existing sensor wires
    input [9:0] dist1_cm,
    input [9:0] dist2_cm
);

always @(posedge clk or posedge reset) begin
    if (reset) begin
        readdata <= 32'd0;
    end
    else if (read) begin
        case (address)
            1'b0: readdata <= {22'd0, dist1_cm}; // zero padded, lower 10 bits dist1_cm
            1'b1: readdata <= {22'd0, dist2_cm}; // ""      ""                  dist2_cm
            default: readdata <= 32'd0;
        endcase
    end
end

endmodule