// reads two HC-SR04 sensors sequentially
// displays each distance in cm on 7seg display
// exposes both distance values to Linux running on the HPS via the 
//  sensor_avalon_slave component inside soc_system (Platform Designer)
//
// pin mappings:
//   GPIO_0[0] -> TRIG1
//   GPIO_0[1] -> ECHO1 (through voltage divider, 5V -> ~3.3V)
//   GPIO_0[2] -> TRIG2
//   GPIO_0[3] -> ECHO2 (through voltage divider)
//   KEY[0]    -> active-low reset
//   HEX0-HEX2 -> sensor 1 distance (ones, tens, hundreds)
//   HEX3-HEX5 -> sensor 2 distance (ones, tens, hundreds)
//   LEDR      -> driven by soc_system's leds PIO (Linux can write here
//                directly over the lightweight bridge, independent of
//                the sensor logic -- this is the same LED sanity check
//                used in mmap_test.c)
//
// NOTE: memory_* and hps_io_* ports are the DE1-SoC
// dedicated HPS hard-block pins (DDR3 + Ethernet/SD/USB/UART), NOT
// regular FPGA I/O. 
// Their physical pin locations are fixed by the chip
// itself -- you do NOT manually assign them in Pin Planner. 
// Instead,
// after running Analysis & Synthesis, run the generated
// hps_sdram_p0_pin_assignments.tcl script (Tools -> Tcl Scripts) BEFORE
// Fitting, which assigns these automatically.


module top (
    input CLOCK_50,
    input [3:0] KEY,
    inout [35:0] GPIO_0,
    output [6:0] HEX0,
    output [6:0] HEX1,
    output [6:0] HEX2,
    output [6:0] HEX3,
    output [6:0] HEX4,
    output [6:0] HEX5,
    output [9:0] LEDR,

    // ---- HPS dedicated Ethernet (EMAC1) pins ----
    output HPS_ENET_TX_CLK,
    output [3:0] HPS_ENET_TXD,
    input [3:0] HPS_ENET_RXD,
    inout HPS_ENET_MDIO,
    output HPS_ENET_MDC,
    input HPS_ENET_RX_CTL,
    output HPS_ENET_TX_CTL,
    input HPS_ENET_RX_CLK,

    // ---- HPS dedicated SD card pins ----
    inout HPS_SD_CMD,
    inout HPS_SD_D0,
    inout HPS_SD_D1,
    output HPS_SD_CLK,
    inout HPS_SD_D2,
    inout HPS_SD_D3,

    // ---- HPS dedicated USB1 pins ----
    inout [7:0] HPS_USB_DATA,
    output HPS_USB_CLKOUT,
    output HPS_USB_STP,
    input HPS_USB_DIR,
    input HPS_USB_NXT,

    // ---- HPS dedicated UART0 pins ----
    input HPS_UART_RX,
    output HPS_UART_TX,

    // ---- HPS dedicated DDR3 pins ----
    output [14:0] HPS_DDR3_ADDR,
    output [2:0] HPS_DDR3_BA,
    output HPS_DDR3_CK_P,
    output HPS_DDR3_CK_N,
    output HPS_DDR3_CKE,
    output HPS_DDR3_CS_N,
    output HPS_DDR3_RAS_N,
    output HPS_DDR3_CAS_N,
    output HPS_DDR3_WE_N,
    output HPS_DDR3_RESET_N,
    inout [31:0] HPS_DDR3_DQ,
    inout [3:0] HPS_DDR3_DQS_P,
    inout [3:0] HPS_DDR3_DQS_N,
    output HPS_DDR3_ODT,
    output [3:0] HPS_DDR3_DM,
    input HPS_DDR3_OCT_RZQIN
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
wire busy1, busy2, done1, done2;
wire [16:0] us1, us2;

hcsr04_driver sensor1 (
    .clk(CLOCK_50), .rst_n(rst_n), .clk_1us(clk_1us),
    .start(start1), .echo(echo1),

    .trig(trig1), .busy(busy1), .distance_us(us1), .done(done1)
);

hcsr04_driver sensor2 (
    .clk(CLOCK_50), .rst_n(rst_n), .clk_1us(clk_1us),
    .start(start2), .echo(echo2),

    .trig(trig2), .busy(busy2), .distance_us(us2), .done(done2)
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
            //  pulse start when the sensor = !busy (i.e. IDLE) 
            // pulse start while busy leads to locked scheduler (state machine perp wiating)
            
            S_START1: begin
                if (!busy1) begin
                    start1 <= 1'b1;
                    sched_state <= S_WAIT1;
                end
            end
            S_WAIT1: begin
                if (done1) begin
                    dist1_us_latched <= us1;
                    sched_state <= S_START2;
                end
            end
            S_START2: begin
                if (!busy2) begin
                    start2      <= 1'b1;
                    sched_state <= S_WAIT2;
                end
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

// 7seg decode
seg7_decode u_h0 (.bcd(d1_ones), .seg(HEX0));
seg7_decode u_h1 (.bcd(d1_tens), .seg(HEX1));
seg7_decode u_h2 (.bcd(d1_hundreds), .seg(HEX2));
seg7_decode u_h3 (.bcd(d2_ones), .seg(HEX3));
seg7_decode u_h4 (.bcd(d2_tens), .seg(HEX4));
seg7_decode u_h5 (.bcd(d2_hundreds), .seg(HEX5));

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// =========== Platform Designer system (HPS + sensor registers + LED PIO) ===========
// dist1_cm/dist2_cm feed straight into the sensor_avalon_slave component's
// conduit inputs, exposing them to Linux at the addresses 0x100
// Set in Platform Designer/ Sensor Avalon Slave Address Map tab

soc_system u0 (
    .clk_clk (CLOCK_50),
    // tie high. HPS manages its own reset internally, and the HPS-to-FPGA bridge is always on
    .reset_reset_nx(1'b1),

    .dist1_export (dist1_cm),
    .dist2_export (dist2_cm),
    .leds_export (LEDR),

    // ---- HPS Ethernet (EMAC1) ----
    .hps_io_hps_io_emac1_inst_TX_CLK (HPS_ENET_TX_CLK),
    .hps_io_hps_io_emac1_inst_TXD0   (HPS_ENET_TXD[0]),
    .hps_io_hps_io_emac1_inst_TXD1   (HPS_ENET_TXD[1]),
    .hps_io_hps_io_emac1_inst_TXD2   (HPS_ENET_TXD[2]),
    .hps_io_hps_io_emac1_inst_TXD3   (HPS_ENET_TXD[3]),
    .hps_io_hps_io_emac1_inst_RXD0   (HPS_ENET_RXD[0]),
    .hps_io_hps_io_emac1_inst_MDIO   (HPS_ENET_MDIO),
    .hps_io_hps_io_emac1_inst_MDC    (HPS_ENET_MDC),
    .hps_io_hps_io_emac1_inst_RX_CTL (HPS_ENET_RX_CTL),
    .hps_io_hps_io_emac1_inst_TX_CTL (HPS_ENET_TX_CTL),
    .hps_io_hps_io_emac1_inst_RX_CLK (HPS_ENET_RX_CLK),
    .hps_io_hps_io_emac1_inst_RXD1   (HPS_ENET_RXD[1]),
    .hps_io_hps_io_emac1_inst_RXD2   (HPS_ENET_RXD[2]),
    .hps_io_hps_io_emac1_inst_RXD3   (HPS_ENET_RXD[3]),

    // ---- HPS SD card ----
    .hps_io_hps_io_sdio_inst_CMD     (HPS_SD_CMD),
    .hps_io_hps_io_sdio_inst_D0      (HPS_SD_D0),
    .hps_io_hps_io_sdio_inst_D1      (HPS_SD_D1),
    .hps_io_hps_io_sdio_inst_CLK     (HPS_SD_CLK),
    .hps_io_hps_io_sdio_inst_D2      (HPS_SD_D2),
    .hps_io_hps_io_sdio_inst_D3      (HPS_SD_D3),

    // ---- HPS USB1 ----
    .hps_io_hps_io_usb1_inst_D0      (HPS_USB_DATA[0]),
    .hps_io_hps_io_usb1_inst_D1      (HPS_USB_DATA[1]),
    .hps_io_hps_io_usb1_inst_D2      (HPS_USB_DATA[2]),
    .hps_io_hps_io_usb1_inst_D3      (HPS_USB_DATA[3]),
    .hps_io_hps_io_usb1_inst_D4      (HPS_USB_DATA[4]),
    .hps_io_hps_io_usb1_inst_D5      (HPS_USB_DATA[5]),
    .hps_io_hps_io_usb1_inst_D6      (HPS_USB_DATA[6]),
    .hps_io_hps_io_usb1_inst_D7      (HPS_USB_DATA[7]),
    .hps_io_hps_io_usb1_inst_CLK     (HPS_USB_CLKOUT),
    .hps_io_hps_io_usb1_inst_STP     (HPS_USB_STP),
    .hps_io_hps_io_usb1_inst_DIR     (HPS_USB_DIR),
    .hps_io_hps_io_usb1_inst_NXT     (HPS_USB_NXT),

    // ---- HPS UART0 ----
    .hps_io_hps_io_uart0_inst_RX     (HPS_UART_RX),
    .hps_io_hps_io_uart0_inst_TX     (HPS_UART_TX),

    // ---- HPS DDR3 ----
    .memory_mem_a                    (HPS_DDR3_ADDR),
    .memory_mem_ba                   (HPS_DDR3_BA),
    .memory_mem_ck                   (HPS_DDR3_CK_P),
    .memory_mem_ck_n                 (HPS_DDR3_CK_N),
    .memory_mem_cke                  (HPS_DDR3_CKE),
    .memory_mem_cs_n                 (HPS_DDR3_CS_N),
    .memory_mem_ras_n                (HPS_DDR3_RAS_N),
    .memory_mem_cas_n                (HPS_DDR3_CAS_N),
    .memory_mem_we_n                 (HPS_DDR3_WE_N),
    .memory_mem_reset_n              (HPS_DDR3_RESET_N),
    .memory_mem_dq                   (HPS_DDR3_DQ),
    .memory_mem_dqs                  (HPS_DDR3_DQS_P),
    .memory_mem_dqs_n                (HPS_DDR3_DQS_N),
    .memory_mem_odt                  (HPS_DDR3_ODT),
    .memory_mem_dm                   (HPS_DDR3_DM),
    .memory_oct_rzqin                (HPS_DDR3_OCT_RZQIN)
);

endmodule