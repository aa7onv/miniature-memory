//drives HC-SR04 sensor to fire a 10us trigger pulse, and measures the echo pulse width to calculate distance.
// Trig high for 10 µs -> wait for Echo to rise → count µs while Echo is high ->
// when Echo falls, hold the count -> wait ~60ms recovery period (prevent trig signal to the echo sig)-> repeat

// pins
// Vcc, trig(in), echo(out), Gnd

module hcsr04_driver #(
    parameter [16:0] RECOVERY_US = 60000, // ~60ms recommended idle time between pings
    parameter [16:0] TIMEOUT_US  = 30000  // ~30ms timeout if echo never returns
)(
    input clk,
    input rst_n,
    input clk_1us, //tick enaeble driven by top.v
    input start,
    input echo,

    output reg trig,
    output reg [16:0] distance_us,
    output reg done
);

//STATES
localparam IDLE = 3'd0,
           TRIG = 3'd1,
           WAIT_RISE = 3'd2,
           MEASURE = 3'd3,
           LATCH = 3'd4;
           RECOVER = 3'd5;

reg [2:0] state;
reg [16:0] us_counter;  
reg [16:0] echo_counter; // high time of echo

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state <= IDLE;
        trig  <= 1'b0;
        distance_us <= 17'd0;
        done <= 1'b0;
        us_counter <= 17'd0;
        echo_counter <= 17'd0;
    end
    else begin
        done <= 1'b0;

        case (state)
            IDLE: begin
                trig <= 1'b0;
                if (start) begin
                    us_counter <= 17'd0;
                    state <= TRIG;
                end
            end

            TRIG: begin //hold trig high for 10us
                trig <= 1'b1;
                if (clk_1us) begin
                    us_counter <= us_counter + 1'b1;
                    if (us_counter >= 4'b1001) begin
                        trig <= 1'b0;
                        us_counter <= 17'd0;
                        state <= WAIT_RISE;
                    end
                end
            end

           
            WAIT_RISE: begin
                if (echo) begin           // if ECHO
                    us_counter <= 17'd0;
                    echo_counter <= 17'd0;
                    state <= MEASURE;
                end
                else if (clk_1us) begin   // if TIMEOUT
                    us_counter <= us_counter + 1'b1;
                    if (us_counter >= TIMEOUT_US) begin
                        distance_us <= 17'd0; // report 0 = no echo / out of range
                        state <= LATCH;
                    end 
                end
            end

            MEASURE: begin
                if (clk_1us) begin
                    if (echo) begin
                        echo_counter <= echo_counter + 1'b1;
                    end else begin
                        distance_us <= echo_counter;
                        state <= LATCH;
                    end
                end
            end

            LATCH: begin
                done <= 1'b1;
                us_counter <= 17'd0;
                state <= RECOVER;
            end

            // Idle gap before this sensor can be triggered again
            RECOVER: begin
                if (clk_1us) begin
                    us_counter <= us_counter + 1'b1;
                    if (us_counter >= RECOVERY_US) begin
                        state <= IDLE;
                    end
                end
            end

        endcase
    end // else

end
endmodule