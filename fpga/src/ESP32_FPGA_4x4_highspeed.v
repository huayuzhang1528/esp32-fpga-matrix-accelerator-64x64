// ESP32 <-> Gowin GW1NSR-4C matrix accelerator
//
// Data path:
//   * Signed INT8 inputs, signed INT32 accumulation/results.
//   * B is retained on the FPGA (up to 32x32).
//   * ESP32 streams four rows of A at a time.
//   * The FPGA returns one 4x4 C tile per COMPUTE_TILE command.
//   * Eight Gowin MULTADD blocks evaluate sixteen products per cycle by
//     accumulating two K terms for eight outputs at a time. Two row phases
//     complete the sixteen outputs of a 4x4 tile.
//
// SPI mode 0 protocol (all commands use their own CS-low transaction):
//   A0 N                 SET_N; N = 4, 8, 12, ... 32
//   B0 <N*N bytes>       LOAD_B, signed INT8, row-major
//   A1 <4*N bytes>       LOAD_A_BLOCK, four signed INT8 rows, row-major
//   C0 tile_column       COMPUTE_TILE, tile_column = 0 .. N/4-1
//   <next transaction>   clock 64 dummy bytes; receive 16 little-endian INT32
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
    reg [8:0] spi_tx_position;
    reg [511:0] spi_tx_shift;
    reg       miso_reg;

    // These result registers are written only by clk and remain stable while
    // a tile is read. Byte 0 is c00[7:0].
    reg signed [31:0] c00, c01, c02, c03;
    reg signed [31:0] c10, c11, c12, c13;
    reg signed [31:0] c20, c21, c22, c23;
    reg signed [31:0] c30, c31, c32, c33;

    wire [511:0] result_bus = {
        c33, c32, c31, c30,
        c23, c22, c21, c20,
        c13, c12, c11, c10,
        c03, c02, c01, c00
    };

    function [31:0] stream_word;
        input [31:0] value;
        begin
            // SPI sends the least-significant result byte first, MSB-first
            // within each byte.
            stream_word = {value[7:0], value[15:8], value[23:16], value[31:24]};
        end
    endfunction

    wire [511:0] result_stream = {
        stream_word(c00), stream_word(c01), stream_word(c02), stream_word(c03),
        stream_word(c10), stream_word(c11), stream_word(c12), stream_word(c13),
        stream_word(c20), stream_word(c21), stream_word(c22), stream_word(c23),
        stream_word(c30), stream_word(c31), stream_word(c32), stream_word(c33)
    };
    reg [511:0] result_stream_snapshot;

    reg done_reg;

    assign miso = (!cs && done_reg)
                ? ((spi_tx_position < 9'd8)
                    ? result_stream_snapshot[9'd511 - spi_tx_position]
                    : miso_reg)
                : 1'b0;

    // Capture the fully updated result one board-clock after DONE is raised.
    // The ESP32 software and CS setup delay leave much more than one cycle
    // before the first read clock.
    always @(posedge clk) begin
        if (done_reg)
            result_stream_snapshot <= result_stream;
    end

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
            spi_tx_position <= 9'd0;
            spi_tx_shift    <= 512'd0;
            miso_reg        <= 1'b0;
        end else if (done_reg) begin
            if (spi_tx_position < 9'd511) begin
                spi_tx_position <= spi_tx_position + 1'b1;
                if (spi_tx_position == 9'd0) begin
                    spi_tx_shift <= {result_stream_snapshot[510:0], 1'b0};
                    miso_reg     <= result_stream_snapshot[510];
                end else begin
                    spi_tx_shift <= {spi_tx_shift[510:0], 1'b0};
                    miso_reg     <= spi_tx_shift[510];
                end
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
    reg [6:0] b_write_address;
    reg [7:0] b_write_data;

    reg       a_write_enable;
    reg [2:0] a_write_bank;
    reg [3:0] a_write_address;
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
                            (spi_rx_byte <= 8'd32) &&
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
                            // Eight column groups are reserved for MAX_N=32.
                            b_write_address <= {b_load_row[4:1], 3'b000}
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
                        if (configuration_valid && (a_load_row < 3'd4)) begin
                            a_write_enable  <= 1'b1;
                            // bank = {row in the block, K parity}
                            a_write_bank    <= {a_load_row[1:0], a_load_col[0]};
                            a_write_address <= a_load_col[4:1];
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
    // A: four rows, split into even/odd K banks (8 x 16 bytes).
    // B: four column banks x even/odd K (8 x 128 bytes = 1024 bytes).
    // This banking supplies the sixteen INT8 operands needed per cycle.
    // ---------------------------------------------------------------------
    reg signed [7:0] a0_even [0:15];
    reg signed [7:0] a0_odd  [0:15];
    reg signed [7:0] a1_even [0:15];
    reg signed [7:0] a1_odd  [0:15];
    reg signed [7:0] a2_even [0:15];
    reg signed [7:0] a2_odd  [0:15];
    reg signed [7:0] a3_even [0:15];
    reg signed [7:0] a3_odd  [0:15];

    reg signed [7:0] b0_even [0:127];
    reg signed [7:0] b1_even [0:127];
    reg signed [7:0] b2_even [0:127];
    reg signed [7:0] b3_even [0:127];
    reg signed [7:0] b0_odd  [0:127];
    reg signed [7:0] b1_odd  [0:127];
    reg signed [7:0] b2_odd  [0:127];
    reg signed [7:0] b3_odd  [0:127];

    reg signed [7:0] a0e_q, a0o_q, a1e_q, a1o_q;
    reg signed [7:0] a2e_q, a2o_q, a3e_q, a3o_q;
    reg signed [7:0] b0e_q, b0o_q, b1e_q, b1o_q;
    reg signed [7:0] b2e_q, b2o_q, b3e_q, b3o_q;

    localparam [1:0] CORE_IDLE = 2'd0;
    localparam [1:0] CORE_RUN  = 2'd1;
    localparam [1:0] CORE_DONE = 2'd2;

    reg [1:0] core_state;
    reg       row_phase;
    reg [3:0] read_k_pair;
    reg [3:0] mac_k_pair;
    reg       reads_complete;
    reg       mac_valid;
    reg [3:0] active_tile_column;

    wire [6:0] b_read_address = {read_k_pair, 3'b000}
                                      + active_tile_column;

    always @(posedge clk) begin
        if (a_write_enable) begin
            case (a_write_bank)
                3'd0: a0_even[a_write_address] <= a_write_data;
                3'd1: a0_odd [a_write_address] <= a_write_data;
                3'd2: a1_even[a_write_address] <= a_write_data;
                3'd3: a1_odd [a_write_address] <= a_write_data;
                3'd4: a2_even[a_write_address] <= a_write_data;
                3'd5: a2_odd [a_write_address] <= a_write_data;
                3'd6: a3_even[a_write_address] <= a_write_data;
                3'd7: a3_odd [a_write_address] <= a_write_data;
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
            a2e_q <= a2_even[read_k_pair];
            a2o_q <= a2_odd [read_k_pair];
            a3e_q <= a3_even[read_k_pair];
            a3o_q <= a3_odd [read_k_pair];

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

    // Select two rows at a time so eight dual-multiply DSP blocks are shared
    // across the top and bottom halves of the 4x4 output tile.
    wire signed [7:0] ar0e = row_phase ? a2e_q : a0e_q;
    wire signed [7:0] ar0o = row_phase ? a2o_q : a0o_q;
    wire signed [7:0] ar1e = row_phase ? a3e_q : a1e_q;
    wire signed [7:0] ar1o = row_phase ? a3o_q : a1o_q;

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
            row_phase            <= 1'b0;
            read_k_pair          <= 4'd0;
            mac_k_pair           <= 4'd0;
            reads_complete       <= 1'b0;
            mac_valid            <= 1'b0;
            done_reg             <= 1'b0;

            c00 <= 32'sd0; c01 <= 32'sd0; c02 <= 32'sd0; c03 <= 32'sd0;
            c10 <= 32'sd0; c11 <= 32'sd0; c12 <= 32'sd0; c13 <= 32'sd0;
            c20 <= 32'sd0; c21 <= 32'sd0; c22 <= 32'sd0; c23 <= 32'sd0;
            c30 <= 32'sd0; c31 <= 32'sd0; c32 <= 32'sd0; c33 <= 32'sd0;
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
                if (!row_phase) begin
                    c00 <= c00 + pair00; c01 <= c01 + pair01;
                    c02 <= c02 + pair02; c03 <= c03 + pair03;
                    c10 <= c10 + pair10; c11 <= c11 + pair11;
                    c12 <= c12 + pair12; c13 <= c13 + pair13;
                end else begin
                    c20 <= c20 + pair00; c21 <= c21 + pair01;
                    c22 <= c22 + pair02; c23 <= c23 + pair03;
                    c30 <= c30 + pair10; c31 <= c31 + pair11;
                    c32 <= c32 + pair12; c33 <= c33 + pair13;
                end

                if (mac_k_pair == ((matrix_n >> 1) - 1'b1)) begin
                    if (!row_phase) begin
                        row_phase      <= 1'b1;
                        read_k_pair    <= 4'd0;
                        mac_k_pair     <= 4'd0;
                        reads_complete <= 1'b0;
                        mac_valid      <= 1'b0;
                    end else begin
                        // Include the final pair because nonblocking cXX
                        // assignments above are not visible until this edge.
                        c20 <= c20 + pair00; c21 <= c21 + pair01;
                        c22 <= c22 + pair02; c23 <= c23 + pair03;
                        c30 <= c30 + pair10; c31 <= c31 + pair11;
                        c32 <= c32 + pair12; c33 <= c33 + pair13;
                        core_state <= CORE_DONE;
                        done_reg   <= 1'b1;
                        mac_valid  <= 1'b0;
                    end
                end
            end
        end
    end

    initial begin
        spi_bit_count          = 3'd0;
        spi_rx_shift           = 8'd0;
        spi_rx_byte_native     = 8'd0;
        spi_rx_toggle          = 1'b0;
        spi_tx_position        = 9'd0;
        spi_tx_shift           = 512'd0;
        miso_reg               = 1'b0;
        result_stream_snapshot = 512'd0;
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
        row_phase              = 1'b0;
        read_k_pair            = 4'd0;
        mac_k_pair             = 4'd0;
        reads_complete         = 1'b0;
        mac_valid              = 1'b0;
        active_tile_column     = 4'd0;
        compute_request_seen   = 1'b0;
        read_complete_seen     = 1'b0;
        c00 = 0; c01 = 0; c02 = 0; c03 = 0;
        c10 = 0; c11 = 0; c12 = 0; c13 = 0;
        c20 = 0; c21 = 0; c22 = 0; c23 = 0;
        c30 = 0; c31 = 0; c32 = 0; c33 = 0;
    end

endmodule
