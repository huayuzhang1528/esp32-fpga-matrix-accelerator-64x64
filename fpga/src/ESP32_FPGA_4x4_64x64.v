// ESP32 <-> Gowin GW1NSR-4C matrix accelerator
//
// Data path:
//   * Signed INT8 inputs, signed INT32 accumulation/results.
//   * B is retained on the FPGA (up to 64x64).
//   * ESP32 streams two rows of A at a time.
//   * The FPGA returns one 2x4 C tile per COMPUTE_TILE command.
//   * Eight Gowin MULTADD blocks evaluate sixteen products per cycle by
//     accumulating two K terms for all eight outputs of a 2x4 tile.
//
// SPI mode 0 protocol (all commands use their own CS-low transaction):
//   A0 N                 SET_N; N = 4, 8, 12, ... 64
//   B0 <N*N bytes>       LOAD_B, signed INT8, row-major
//   A1 <2*N bytes>       LOAD_A_BLOCK, two signed INT8 rows, row-major
//   C0 tile_column       COMPUTE_TILE, tile_column = 0 .. N/4-1
//   <next transaction>   clock 32 dummy bytes; receive 8 little-endian INT32
//
// High-speed variant: SPI bytes are captured by the native SCLK on FPGA
// package pin 42 (GCLKC_1), then transferred into the board-clock domain.

module top(
    input  wire clk,
    input  wire sclk,
    input  wire cs,
    input  wire mosi,
    output wire miso,
    output wire done
);

    // Pin 42 is GCLKC_1. Explicitly use the dedicated global clock network;
    // otherwise Gowin routes SCLK as ordinary fabric and introduces large
    // edge skew even though the package pin itself is clock-capable.
    wire sclk_global;
    BUFG sclk_global_buffer (.I(sclk), .O(sclk_global));

    localparam [7:0] CMD_SET_N        = 8'hA0;
    localparam [7:0] CMD_LOAD_A_BLOCK = 8'hA1;
    localparam [7:0] CMD_LOAD_B       = 8'hB0;
    localparam [7:0] CMD_COMPUTE_TILE = 8'hC0;

    // ---------------------------------------------------------------------
    // Result registers and serialized readback state.
    // ---------------------------------------------------------------------
    reg [2:0] spi_bit_count;
    reg [7:0] spi_rx_shift;
    reg [7:0] spi_rx_byte_native;
    reg       spi_rx_toggle;
    reg [2:0] spi_tx_bit_count;
    reg [4:0] spi_tx_byte_index;
    reg [7:0] spi_tx_shift;

    // These result registers are written only by clk and remain stable while
    // a tile is read. Byte 0 is c00[7:0].
    reg signed [31:0] c00, c01, c02, c03;
    reg signed [31:0] c10, c11, c12, c13;

    function [7:0] result_byte;
        input [4:0] index;
        begin
            case (index)
                5'd0:  result_byte = c00[7:0];
                5'd1:  result_byte = c00[15:8];
                5'd2:  result_byte = c00[23:16];
                5'd3:  result_byte = c00[31:24];
                5'd4:  result_byte = c01[7:0];
                5'd5:  result_byte = c01[15:8];
                5'd6:  result_byte = c01[23:16];
                5'd7:  result_byte = c01[31:24];
                5'd8:  result_byte = c02[7:0];
                5'd9:  result_byte = c02[15:8];
                5'd10: result_byte = c02[23:16];
                5'd11: result_byte = c02[31:24];
                5'd12: result_byte = c03[7:0];
                5'd13: result_byte = c03[15:8];
                5'd14: result_byte = c03[23:16];
                5'd15: result_byte = c03[31:24];
                5'd16: result_byte = c10[7:0];
                5'd17: result_byte = c10[15:8];
                5'd18: result_byte = c10[23:16];
                5'd19: result_byte = c10[31:24];
                5'd20: result_byte = c11[7:0];
                5'd21: result_byte = c11[15:8];
                5'd22: result_byte = c11[23:16];
                5'd23: result_byte = c11[31:24];
                5'd24: result_byte = c12[7:0];
                5'd25: result_byte = c12[15:8];
                5'd26: result_byte = c12[23:16];
                5'd27: result_byte = c12[31:24];
                5'd28: result_byte = c13[7:0];
                5'd29: result_byte = c13[15:8];
                5'd30: result_byte = c13[23:16];
                default: result_byte = c13[31:24];
            endcase
        end
    endfunction

    reg done_reg;

    // Byte 0 is driven directly so it is valid before the first rising edge.
    // Later bytes are selected during the preceding byte and shifted from an
    // 8-bit register, avoiding the slow 256-to-1 bit mux at 20 MHz.
    assign miso = (!cs && done_reg)
                ? ((spi_tx_byte_index == 5'd0)
                    ? c00[3'd7 - spi_tx_bit_count]
                    : spi_tx_shift[7])
                : 1'b0;

    // SPI mode 0: sample MOSI on rising SCLK. The byte register stays stable
    // until the next complete byte, giving the toggle synchronizer ample time.
    always @(posedge sclk_global or posedge cs) begin
        if (cs) begin
            spi_bit_count <= 3'd0;
            spi_rx_shift  <= 8'd0;
        end else begin
            spi_rx_shift <= {spi_rx_shift[6:0], mosi};
            if (spi_bit_count == 3'd7) begin
                spi_bit_count      <= 3'd0;
                spi_rx_byte_native <= {spi_rx_shift[6:0], mosi};
                spi_rx_toggle      <= ~spi_rx_toggle;
            end else begin
                spi_bit_count <= spi_bit_count + 1'b1;
            end
        end
    end

    // ESP32 mode 0 samples on rising SCLK. Prepare the following bit on the
    // intervening falling edge. The first bit is available before clocking.
    always @(negedge sclk_global or posedge cs) begin
        if (cs) begin
            spi_tx_bit_count  <= 3'd0;
            spi_tx_byte_index <= 5'd0;
            spi_tx_shift      <= 8'd0;
        end else if (done_reg) begin
            if (spi_tx_bit_count == 3'd7) begin
                spi_tx_bit_count <= 3'd0;
                if (spi_tx_byte_index < 5'd31) begin
                    spi_tx_byte_index <= spi_tx_byte_index + 1'b1;
                    spi_tx_shift <= result_byte(spi_tx_byte_index + 1'b1);
                end
            end else begin
                spi_tx_bit_count <= spi_tx_bit_count + 1'b1;
                if (spi_tx_byte_index != 5'd0)
                    spi_tx_shift <= {spi_tx_shift[6:0], 1'b0};
            end
        end
    end

    // ---------------------------------------------------------------------
    // clk-domain synchronization and command parser.
    // ---------------------------------------------------------------------
    reg [2:0] cs_sync;
    reg [2:0] spi_rx_toggle_sync;

    wire cs_falling = (cs_sync[2:1] == 2'b10);
    wire cs_rising  = (cs_sync[2:1] == 2'b01);
    wire rx_event = spi_rx_toggle_sync[2] ^ spi_rx_toggle_sync[1];
    wire [7:0] spi_rx_byte = spi_rx_byte_native;

    always @(posedge clk) begin
        cs_sync            <= {cs_sync[1:0], cs};
        spi_rx_toggle_sync <= {spi_rx_toggle_sync[1:0], spi_rx_toggle};
    end

    reg [7:0]  opcode;
    reg [12:0] command_byte_position;
    reg        reading_session;

    reg [6:0] matrix_n;
    reg       configuration_valid;

    reg [5:0] b_load_row;
    reg [5:0] b_load_col;
    reg [2:0] a_load_row;
    reg [5:0] a_load_col;

    reg       b_write_enable;
    reg [2:0] b_write_bank;
    reg [8:0] b_write_address;
    reg [7:0] b_write_data;

    reg       a_write_enable;
    reg [1:0] a_write_bank;
    reg [4:0] a_write_address;
    reg [7:0] a_write_data;

    reg [3:0] requested_tile_column;
    reg       compute_request_toggle;
    reg       read_complete_toggle;

    assign done = done_reg;

    always @(posedge clk) begin
        b_write_enable <= 1'b0;
        a_write_enable <= 1'b0;

        if (cs_falling) begin
            command_byte_position <= 13'd0;
            reading_session       <= done_reg;
        end

        if (cs_rising) begin
            if (reading_session) begin
                read_complete_toggle <= ~read_complete_toggle;
            end
            reading_session <= 1'b0;
        end

        if (rx_event && !reading_session) begin
            if (command_byte_position == 13'd0) begin
                opcode                <= spi_rx_byte;
                command_byte_position <= 13'd1;

                if (spi_rx_byte == CMD_LOAD_B) begin
                    b_load_row <= 6'd0;
                    b_load_col <= 6'd0;
                end
                if (spi_rx_byte == CMD_LOAD_A_BLOCK) begin
                    a_load_row <= 2'd0;
                    a_load_col <= 6'd0;
                end
            end else begin
                command_byte_position <= command_byte_position + 1'b1;

                case (opcode)
                    CMD_SET_N: begin
                        if ((command_byte_position == 13'd1) &&
                            (spi_rx_byte >= 8'd4) &&
                            (spi_rx_byte <= 8'd64) &&
                            (spi_rx_byte[1:0] == 2'b00)) begin
                            matrix_n            <= spi_rx_byte[6:0];
                            configuration_valid <= 1'b1;
                        end
                    end

                    CMD_LOAD_B: begin
                        if (configuration_valid && (b_load_row < matrix_n)) begin
                            b_write_enable  <= 1'b1;
                            // bank = {K parity, output-column modulo 4}
                            b_write_bank    <= {b_load_row[0], b_load_col[1:0]};
                            // Sixteen column groups are reserved for MAX_N=64.
                            b_write_address <= {b_load_row[5:1], 4'b0000}
                                             + b_load_col[5:2];
                            b_write_data    <= spi_rx_byte;

                            if (b_load_col == matrix_n - 1'b1) begin
                                b_load_col <= 6'd0;
                                b_load_row <= b_load_row + 1'b1;
                            end else begin
                                b_load_col <= b_load_col + 1'b1;
                            end
                        end
                    end

                    CMD_LOAD_A_BLOCK: begin
                        if (configuration_valid && (a_load_row < 3'd2)) begin
                            a_write_enable  <= 1'b1;
                            // bank = {row in the block, K parity}
                            a_write_bank    <= {a_load_row[0], a_load_col[0]};
                            a_write_address <= a_load_col[5:1];
                            a_write_data    <= spi_rx_byte;

                            if (a_load_col == matrix_n - 1'b1) begin
                                a_load_col <= 6'd0;
                                a_load_row <= a_load_row + 1'b1;
                            end else begin
                                a_load_col <= a_load_col + 1'b1;
                            end
                        end
                    end

                    CMD_COMPUTE_TILE: begin
                        if ((command_byte_position == 13'd1) &&
                            configuration_valid &&
                            (spi_rx_byte < (matrix_n >> 2))) begin
                            requested_tile_column <= spi_rx_byte[3:0];
                            compute_request_toggle <= ~compute_request_toggle;
                        end
                    end

                    default: begin
                    end
                endcase
            end
        end
    end

    // ---------------------------------------------------------------------
    // Banked operand memories.
    //
    // A: two rows, split into even/odd K banks (4 x 32 bytes).
    // B: four column banks x even/odd K (8 x 512 bytes = 4096 bytes).
    // This banking supplies the sixteen INT8 operands needed per cycle.
    // ---------------------------------------------------------------------
    reg signed [7:0] a0_even [0:31];
    reg signed [7:0] a0_odd  [0:31];
    reg signed [7:0] a1_even [0:31];
    reg signed [7:0] a1_odd  [0:31];

    reg signed [7:0] b0_even [0:511];
    reg signed [7:0] b1_even [0:511];
    reg signed [7:0] b2_even [0:511];
    reg signed [7:0] b3_even [0:511];
    reg signed [7:0] b0_odd  [0:511];
    reg signed [7:0] b1_odd  [0:511];
    reg signed [7:0] b2_odd  [0:511];
    reg signed [7:0] b3_odd  [0:511];

    reg signed [7:0] a0e_q, a0o_q, a1e_q, a1o_q;
    reg signed [7:0] b0e_q, b0o_q, b1e_q, b1o_q;
    reg signed [7:0] b2e_q, b2o_q, b3e_q, b3o_q;

    localparam [1:0] CORE_IDLE = 2'd0;
    localparam [1:0] CORE_RUN  = 2'd1;
    localparam [1:0] CORE_DONE = 2'd2;

    reg [1:0] core_state;
    reg [4:0] read_k_pair;
    reg [4:0] mac_k_pair;
    reg       reads_complete;
    reg       mac_valid;
    reg [3:0] active_tile_column;

    wire [8:0] b_read_address = {read_k_pair, 4'b0000}
                                      + active_tile_column;

    always @(posedge clk) begin
        if (a_write_enable) begin
            case (a_write_bank)
                2'd0: a0_even[a_write_address] <= a_write_data;
                2'd1: a0_odd [a_write_address] <= a_write_data;
                2'd2: a1_even[a_write_address] <= a_write_data;
                2'd3: a1_odd [a_write_address] <= a_write_data;
            endcase
        end

        if (b_write_enable) begin
            case (b_write_bank)
                3'd0: b0_even[b_write_address] <= b_write_data;
                3'd1: b1_even[b_write_address] <= b_write_data;
                3'd2: b2_even[b_write_address] <= b_write_data;
                3'd3: b3_even[b_write_address] <= b_write_data;
                3'd4: b0_odd [b_write_address] <= b_write_data;
                3'd5: b1_odd [b_write_address] <= b_write_data;
                3'd6: b2_odd [b_write_address] <= b_write_data;
                3'd7: b3_odd [b_write_address] <= b_write_data;
            endcase
        end

        if ((core_state == CORE_RUN) && !reads_complete) begin
            a0e_q <= a0_even[read_k_pair];
            a0o_q <= a0_odd [read_k_pair];
            a1e_q <= a1_even[read_k_pair];
            a1o_q <= a1_odd [read_k_pair];

            b0e_q <= b0_even[b_read_address];
            b0o_q <= b0_odd [b_read_address];
            b1e_q <= b1_even[b_read_address];
            b1o_q <= b1_odd [b_read_address];
            b2e_q <= b2_even[b_read_address];
            b2o_q <= b2_odd [b_read_address];
            b3e_q <= b3_even[b_read_address];
            b3o_q <= b3_odd [b_read_address];
        end
    end

    wire signed [7:0] ar0e = a0e_q;
    wire signed [7:0] ar0o = a0o_q;
    wire signed [7:0] ar1e = a1e_q;
    wire signed [7:0] ar1o = a1o_q;

    wire signed [15:0] m00e = ar0e * b0e_q;
    wire signed [15:0] m00o = ar0o * b0o_q;
    wire signed [15:0] m01e = ar0e * b1e_q;
    wire signed [15:0] m01o = ar0o * b1o_q;
    wire signed [15:0] m02e = ar0e * b2e_q;
    wire signed [15:0] m02o = ar0o * b2o_q;
    wire signed [15:0] m03e = ar0e * b3e_q;
    wire signed [15:0] m03o = ar0o * b3o_q;

    wire signed [15:0] m10e = ar1e * b0e_q;
    wire signed [15:0] m10o = ar1o * b0o_q;
    wire signed [15:0] m11e = ar1e * b1e_q;
    wire signed [15:0] m11o = ar1o * b1o_q;
    wire signed [15:0] m12e = ar1e * b2e_q;
    wire signed [15:0] m12o = ar1o * b2o_q;
    wire signed [15:0] m13e = ar1e * b3e_q;
    wire signed [15:0] m13o = ar1o * b3o_q;

    wire signed [16:0] pair00 = {m00e[15], m00e} + {m00o[15], m00o};
    wire signed [16:0] pair01 = {m01e[15], m01e} + {m01o[15], m01o};
    wire signed [16:0] pair02 = {m02e[15], m02e} + {m02o[15], m02o};
    wire signed [16:0] pair03 = {m03e[15], m03e} + {m03o[15], m03o};
    wire signed [16:0] pair10 = {m10e[15], m10e} + {m10o[15], m10o};
    wire signed [16:0] pair11 = {m11e[15], m11e} + {m11o[15], m11o};
    wire signed [16:0] pair12 = {m12e[15], m12e} + {m12o[15], m12o};
    wire signed [16:0] pair13 = {m13e[15], m13e} + {m13o[15], m13o};

    reg compute_request_seen;
    reg read_complete_seen;

    always @(posedge clk) begin
        // Reading a tile acknowledges it and returns DONE low before the next
        // command, avoiding a short low pulse that the ESP32 could miss.
        if (read_complete_seen != read_complete_toggle) begin
            read_complete_seen <= read_complete_toggle;
            done_reg           <= 1'b0;
        end

        if ((compute_request_seen != compute_request_toggle) &&
            (core_state != CORE_RUN)) begin
            compute_request_seen <= compute_request_toggle;
            active_tile_column   <= requested_tile_column;
            core_state           <= CORE_RUN;
            read_k_pair          <= 5'd0;
            mac_k_pair           <= 5'd0;
            reads_complete       <= 1'b0;
            mac_valid            <= 1'b0;
            done_reg             <= 1'b0;

            c00 <= 32'sd0; c01 <= 32'sd0; c02 <= 32'sd0; c03 <= 32'sd0;
            c10 <= 32'sd0; c11 <= 32'sd0; c12 <= 32'sd0; c13 <= 32'sd0;
        end else if (core_state == CORE_RUN) begin
            if (!reads_complete) begin
                mac_k_pair <= read_k_pair;
                mac_valid  <= 1'b1;

                if (read_k_pair == ((matrix_n >> 1) - 1'b1)) begin
                    reads_complete <= 1'b1;
                end else begin
                    read_k_pair <= read_k_pair + 1'b1;
                end
            end else begin
                mac_valid <= 1'b0;
            end

            if (mac_valid) begin
                c00 <= c00 + pair00; c01 <= c01 + pair01;
                c02 <= c02 + pair02; c03 <= c03 + pair03;
                c10 <= c10 + pair10; c11 <= c11 + pair11;
                c12 <= c12 + pair12; c13 <= c13 + pair13;

                if (mac_k_pair == ((matrix_n >> 1) - 1'b1)) begin
                    core_state <= CORE_DONE;
                    done_reg   <= 1'b1;
                    mac_valid  <= 1'b0;
                end
            end
        end
    end

    initial begin
        spi_bit_count          = 3'd0;
        spi_rx_shift           = 8'd0;
        spi_rx_byte_native     = 8'd0;
        spi_rx_toggle          = 1'b0;
        spi_tx_bit_count       = 3'd0;
        spi_tx_byte_index      = 5'd0;
        spi_tx_shift           = 8'd0;
        cs_sync                = 3'b111;
        spi_rx_toggle_sync     = 3'b000;
        opcode                 = 8'd0;
        command_byte_position  = 13'd0;
        reading_session        = 1'b0;
        matrix_n               = 7'd4;
        configuration_valid    = 1'b0;
        b_load_row             = 6'd0;
        b_load_col             = 6'd0;
        a_load_row             = 3'd0;
        a_load_col             = 6'd0;
        b_write_enable         = 1'b0;
        a_write_enable         = 1'b0;
        requested_tile_column  = 4'd0;
        compute_request_toggle = 1'b0;
        read_complete_toggle   = 1'b0;
        done_reg               = 1'b0;
        core_state             = CORE_IDLE;
        read_k_pair            = 5'd0;
        mac_k_pair             = 5'd0;
        reads_complete         = 1'b0;
        mac_valid              = 1'b0;
        active_tile_column     = 4'd0;
        compute_request_seen   = 1'b0;
        read_complete_seen     = 1'b0;
        c00 = 0; c01 = 0; c02 = 0; c03 = 0;
        c10 = 0; c11 = 0; c12 = 0; c13 = 0;
    end

endmodule
