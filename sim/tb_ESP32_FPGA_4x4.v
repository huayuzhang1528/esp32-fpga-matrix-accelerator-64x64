`timescale 1ns/1ps

module tb_ESP32_FPGA_4x4;
    reg clk  = 1'b0;
    reg sclk = 1'b0;
    reg cs   = 1'b1;
    reg mosi = 1'b0;
    wire miso;
    wire done;

    reg [7:0] rx_bytes [0:63];
    reg [7:0] rx_scratch;
    reg [31:0] actual;
    integer i;
    integer errors;

    top dut(
        .clk(clk),
        .sclk(sclk),
        .cs(cs),
        .mosi(mosi),
        .miso(miso),
        .done(done)
    );

    always #10 clk = ~clk;  // 50 MHz FPGA clock

    task spi_begin;
        begin
            cs = 1'b0;
            #100;
        end
    endtask

    task spi_end;
        begin
            #25;
            cs = 1'b1;
            sclk = 1'b0;
            mosi = 1'b0;
            #200;
        end
    endtask

    // SPI mode 0, 20 MHz. The master presents MOSI before the rising edge and
    // samples MISO on the rising edge.
    task spi_byte;
        input [7:0] tx;
        output [7:0] rx;
        integer bit_index;
        begin
            rx = 8'd0;
            for (bit_index = 7; bit_index >= 0; bit_index = bit_index - 1) begin
                mosi = tx[bit_index];
                #25;
                sclk = 1'b1;
                rx = {rx[6:0], miso};
                #25;
                sclk = 1'b0;
            end
        end
    endtask

    task write_byte;
        input [7:0] value;
        begin
            spi_byte(value, rx_scratch);
        end
    endtask

    function signed [31:0] expected_value;
        input integer index;
        begin
            case (index)
                 0: expected_value =  32'sd3;
                 1: expected_value = 32'sd13;
                 2: expected_value =  32'sd4;
                 3: expected_value =  32'sd6;
                 4: expected_value = 32'sd11;
                 5: expected_value = 32'sd29;
                 6: expected_value = 32'sd12;
                 7: expected_value = 32'sd14;
                 8: expected_value = -32'sd11;
                 9: expected_value =  32'sd7;
                10: expected_value =  32'sd0;
                11: expected_value =  32'sd2;
                12: expected_value =  32'sd9;
                13: expected_value = -32'sd3;
                14: expected_value = 32'sd10;
                15: expected_value = -32'sd8;
                default: expected_value = 32'sd0;
            endcase
        end
    endfunction

    initial begin
        errors = 0;
        #500;

        // SET_N = 4
        spi_begin();
        write_byte(8'hA0);
        write_byte(8'd4);
        spi_end();

        // B =
        // [ 1  0  2 -1 ]
        // [ 0  1 -1  2 ]
        // [ 2  1  0  1 ]
        // [-1  2  1  0 ]
        spi_begin();
        write_byte(8'hB0);
        write_byte( 8'sd1); write_byte( 8'sd0);
        write_byte( 8'sd2); write_byte(-8'sd1);
        write_byte( 8'sd0); write_byte( 8'sd1);
        write_byte(-8'sd1); write_byte( 8'sd2);
        write_byte( 8'sd2); write_byte( 8'sd1);
        write_byte( 8'sd0); write_byte( 8'sd1);
        write_byte(-8'sd1); write_byte( 8'sd2);
        write_byte( 8'sd1); write_byte( 8'sd0);
        spi_end();

        // A =
        // [ 1  2  3  4 ]
        // [ 5  6  7  8 ]
        // [-1  2 -3  4 ]
        // [ 4 -3  2 -1 ]
        spi_begin();
        write_byte(8'hA1);
        write_byte( 8'sd1); write_byte( 8'sd2);
        write_byte( 8'sd3); write_byte( 8'sd4);
        write_byte( 8'sd5); write_byte( 8'sd6);
        write_byte( 8'sd7); write_byte( 8'sd8);
        write_byte(-8'sd1); write_byte( 8'sd2);
        write_byte(-8'sd3); write_byte( 8'sd4);
        write_byte( 8'sd4); write_byte(-8'sd3);
        write_byte( 8'sd2); write_byte(-8'sd1);
        spi_end();

        // COMPUTE_TILE column 0
        spi_begin();
        write_byte(8'hC0);
        write_byte(8'd0);
        spi_end();

        i = 0;
        while (!done && i < 1000) begin
            #20;
            i = i + 1;
        end
        if (!done) begin
            $display("FAIL: timeout waiting for DONE");
            $finish;
        end

        // Arm a read, then use a separate transaction for the 64 data bytes.
        spi_begin();
        write_byte(8'hD0);
        spi_end();

        spi_begin();
        for (i = 0; i < 64; i = i + 1)
            spi_byte(8'h00, rx_bytes[i]);
        spi_end();

        for (i = 0; i < 16; i = i + 1) begin
            actual = {
                rx_bytes[i * 4 + 3],
                rx_bytes[i * 4 + 2],
                rx_bytes[i * 4 + 1],
                rx_bytes[i * 4 + 0]
            };
            if ($signed(actual) !== expected_value(i)) begin
                $display(
                    "Mismatch %0d: expected %0d, actual %0d",
                    i,
                    expected_value(i),
                    $signed(actual)
                );
                errors = errors + 1;
            end
        end

        if (errors == 0)
            $display("PASS: signed 4x4 tile and SPI protocol");
        else
            $display("FAIL: %0d mismatches", errors);

        #500;
        $finish;
    end
endmodule
