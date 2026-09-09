// reads two HC-SR04 sensors sequentially and displays
// each distance in cm (0-999) on the onboard 7-segment displays.

// pin mapping:
//   GPIO_0[0] -> TRIG1
//   GPIO_0[1] -> ECHO1 (through voltage divider, 5V -> ~3.3V)
//   GPIO_0[2] -> TRIG2
//   GPIO_0[3] -> ECHO2 (through voltage divider)
//   BTN[0] -> active-low reset
//   HEX0-HEX2 -> sensor 1 distance (ones, tens, hundreds)
//   HEX3-HEX5 -> sensor 2 distance (ones, tens, hundreds)

module top (
    input CLOCK_50,
    input [3:0] KEY,
    inout [35:0] GPIO_0,
    output [6:0] HEX0,
    output [6:0] HEX1,
    output [6:0] HEX2,
    output [6:0] HEX3,
    output [6:0] HEX4,
    output [6:0] HEX5
);

wire rst_n = KEY[0];

// ===== 1us tick generator from 50MHz ======
reg [5:0] div_cnt;
reg clk_1us;
always @(posedge CLOCK_50 or negedge rst_n) begin
    if (!rst_n) begin
        div_cnt <= 6'd0;
        clk_1us <= 1'b0;
    end else if (div_cnt == 6'd49) begin
        div_cnt <= 6'd0;
        clk_1us <= 1'b1;
    end else begin
        div_cnt <= div_cnt + 1'b1;
        clk_1us <= 1'b0;
    end
end

// ==== sensor IO ===
wire trig1, trig2;
wire echo1 = GPIO_0[1];
wire echo2 = GPIO_0[3];
assign GPIO_0[0] = trig1;
assign GPIO_0[2] = trig2;


// =========== 2X sensor drivers ===========
reg  start1, start2;
wire done1, done2;
wire [16:0] us1, us2;

hcsr04_driver sensor1 (
    .clk(CLOCK_50), .rst_n(rst_n), .clk_1us(clk_1us),
    .start(start1), .echo(echo1),

    .trig(trig1), .distance_us(us1), .done(done1)
);

hcsr04_driver sensor2 (
    .clk(CLOCK_50), .rst_n(rst_n), .clk_1us(clk_1us),
    .start(start2), .echo(echo2),

    .trig(trig2), .distance_us(us2), .done(done2)
);

//====== State scheduler =======
localparam S_START1 = 2'd0,
            S_WAIT1  = 2'd1,
            S_START2 = 2'd2,
            S_WAIT2  = 2'd3;

reg [1:0] sched_state;
reg [16:0] dist1_us_latched, dist2_us_latched;

always @(posedge CLOCK_50 or negedge rst_n) begin
    if (!rst_n) begin
        sched_state <= S_START1;
        start1 <= 1'b0;
        start2 <= 1'b0;
        dist1_us_latched <= 17'd0;
        dist2_us_latched <= 17'd0;
    end 
    else begin
        start1 <= 1'b0;
        start2 <= 1'b0;
        case (sched_state)
            S_START1: begin
                start1 <= 1'b1;
                sched_state <= S_WAIT1;
            end
            S_WAIT1: begin
                if (done1) begin
                    dist1_us_latched <= us1;
                    sched_state <= S_START2;
                end
            end
            S_START2: begin
                start2      <= 1'b1;
                sched_state <= S_WAIT2;
            end
            S_WAIT2: begin
                if (done2) begin
                    dist2_us_latched <= us2;
                    sched_state      <= S_START1;
                end
            end
            default: sched_state <= S_START1;
        endcase
    end
end

// ====== convert echo time (us) to distance (cm) ======
wire [9:0] dist1_cm = dist1_us_latched / 17'd58;
wire [9:0] dist2_cm = dist2_us_latched / 17'd58;

// --- split into hundreds, tens, ones for 7-seg display ---
wire [3:0] d1_hundreds = (dist1_cm / 100) % 10;
wire [3:0] d1_tens = (dist1_cm / 10) % 10;
wire [3:0] d1_ones =  dist1_cm % 10;

wire [3:0] d2_hundreds = (dist2_cm / 100) % 10;
wire [3:0] d2_tens = (dist2_cm / 10) % 10;
wire [3:0] d2_ones =  dist2_cm % 10;

// 7seg displ
seg7_decode u_h0 (.bcd(d1_ones), .seg(HEX0));
seg7_decode u_h1 (.bcd(d1_tens), .seg(HEX1));
seg7_decode u_h2 (.bcd(d1_hundreds), .seg(HEX2));
seg7_decode u_h3 (.bcd(d2_ones), .seg(HEX3));
seg7_decode u_h4 (.bcd(d2_tens), .seg(HEX4));
seg7_decode u_h5 (.bcd(d2_hundreds), .seg(HEX5));


endmodule

