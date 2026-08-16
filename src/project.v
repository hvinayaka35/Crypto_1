/*
 * Copyright (c) 2026 H Vinayaka
 * SPDX-License-Identifier: Apache-2.0
 *
 * Crypto-agile NTT butterfly unit for the three NIST PQC standards.
 *
 *   ML-KEM  (FIPS 203)   q =    3329   k = 12
 *   FN-DSA  (FIPS 206)   q =   12289   k = 14
 *   ML-DSA  (FIPS 204)   q = 8380417   k = 23
 *
 * A single radix-2 bit-serial Montgomery modular multiplier is time-shared
 * with the butterfly modular add/sub through one 25-bit three-operand adder.
 * Because Montgomery reduction is valid for any odd modulus, agility costs
 * only a constant mux on q and a different terminal count -- there is no
 * per-scheme reduction hardware.
 *
 *   t = b * w * 2^-k mod q      (w supplied in the Montgomery domain)
 *   v = (a - t) mod q
 *   u = (a + t) mod q
 */

`default_nettype none

module tt_um_vinayaka_ntt_bfly (
    input  wire [7:0] ui_in,    // Dedicated inputs  - operand byte stream
    output wire [7:0] uo_out,   // Dedicated outputs - result byte stream
    input  wire [7:0] uio_in,   // IOs: Input path   - control
    output wire [7:0] uio_out,  // IOs: Output path  - status
    output wire [7:0] uio_oe,   // IOs: Enable path
    input  wire       ena,      // always 1 when the design is powered
    input  wire       clk,      // clock
    input  wire       rst_n     // reset_n - low to reset
);

  localparam W = 24;

  // ---------------------------------------------------------------- control
  wire       sh       = uio_in[0];      // shift a byte in (idle) / out (done)
  wire       start    = uio_in[1];      // launch a butterfly
  wire [1:0] scheme_i = uio_in[4:3];    // 00=ML-KEM 01=FN-DSA 1x=ML-DSA

  localparam S_IDLE = 3'd0, S_MUL  = 3'd1, S_RED  = 3'd2, S_SUB1 = 3'd3,
             S_SUB2 = 3'd4, S_ADD1 = 3'd5, S_ADD2 = 3'd6, S_DONE = 3'd7;

  reg [2:0]   state;
  reg [1:0]   scheme;
  reg [W-1:0] areg;   // a  -> u
  reg [W-1:0] breg;   // b  -> v
  reg [W-1:0] wreg;   // w, consumed LSB-first by the Montgomery loop
  reg [W-1:0] sreg;   // Montgomery accumulator, then t
  reg [4:0]   cnt;
  reg         borrow;

  // ------------------------------------------------------- scheme constants
  reg [W-1:0] q;
  always @(*) begin
    case (scheme)
      2'd0:    q = 24'd3329;
      2'd1:    q = 24'd12289;
      default: q = 24'd8380417;
    endcase
  end

  reg [4:0] klast;                      // k-1
  always @(*) begin
    case (scheme)
      2'd0:    klast = 5'd11;
      2'd1:    klast = 5'd13;
      default: klast = 5'd22;
    endcase
  end

  // ------------------------------------------------- shared 3-operand adder
  wire is_mul = (state == S_MUL);
  wire abit   = wreg[0];
  wire mbit   = sreg[0] ^ (abit & breg[0]);   // q is odd, so q[0] == 1

  reg [W-1:0] opx;
  always @(*) begin
    case (state)
      S_SUB1, S_ADD1, S_ADD2: opx = areg;
      S_SUB2:                 opx = breg;
      default:                opx = sreg;
    endcase
  end

  reg [1:0] ysel;                       // 00:zero 01:breg 10:q 11:sreg
  reg       yinv;
  always @(*) begin
    case (state)
      S_MUL  : begin ysel = {1'b0, abit};   yinv = 1'b0; end
      S_RED  : begin ysel = 2'b10;          yinv = 1'b1; end
      S_SUB1 : begin ysel = 2'b11;          yinv = 1'b1; end
      S_SUB2 : begin ysel = {borrow, 1'b0}; yinv = 1'b0; end
      S_ADD1 : begin ysel = 2'b11;          yinv = 1'b0; end
      default: begin ysel = 2'b10;          yinv = 1'b1; end  // S_ADD2
    endcase
  end

  reg [W-1:0] yraw;
  always @(*) begin
    case (ysel)
      2'b01:   yraw = breg;
      2'b10:   yraw = q;
      2'b11:   yraw = sreg;
      default: yraw = {W{1'b0}};
    endcase
  end
  wire [W-1:0] opy = yraw ^ {W{yinv}};

  // opz is q during the Montgomery step, otherwise the +1 of a two's
  // complement subtraction.
  wire subop = (state == S_RED) | (state == S_SUB1) | (state == S_ADD2);
  wire [W-1:0] opz = (is_mul & mbit) ? q
                                     : {{(W-1){1'b0}}, (subop & ~is_mul)};

  wire [W:0] sum = {1'b0, opx} + {1'b0, opy} + {1'b0, opz};

  // ------------------------------------------------------------- sequencing
  always @(posedge clk) begin
    if (!rst_n) begin
      state  <= S_IDLE;
      scheme <= 2'd0;
      areg   <= {W{1'b0}};
      breg   <= {W{1'b0}};
      wreg   <= {W{1'b0}};
      sreg   <= {W{1'b0}};
      cnt    <= 5'd0;
      borrow <= 1'b0;
    end else begin
      case (state)

        // Byte shift chain. Loading (idle) and unloading (done) drive the
        // identical network, so they cost one set of muxes between them:
        //   in -> wreg -> breg -> areg -> out
        S_IDLE, S_DONE: begin
          if (sh) begin
            areg  <= {areg[15:0], breg[W-1:16]};
            breg  <= {breg[15:0], wreg[W-1:16]};
            wreg  <= {wreg[15:0], ui_in};
            state <= S_IDLE;
          end else if (start) begin
            scheme <= scheme_i;
            sreg   <= {W{1'b0}};
            cnt    <= 5'd0;
            state  <= S_MUL;
          end
        end

        // S <- (S + a_i*B + m*q) >> 1
        S_MUL: begin
          sreg <= sum[W:1];
          wreg <= {1'b0, wreg[W-1:1]};
          cnt  <= cnt + 5'd1;
          if (cnt == klast) state <= S_RED;
        end

        // S < 2q  ->  t = S - q if that does not borrow
        S_RED: begin
          if (sum[W]) sreg <= sum[W-1:0];
          state <= S_SUB1;
        end

        S_SUB1: begin                   // breg <- a - t  (mod 2^24)
          breg   <= sum[W-1:0];
          borrow <= ~sum[W];
          state  <= S_SUB2;
        end

        S_SUB2: begin                   // v = breg + (borrow ? q : 0)
          breg  <= sum[W-1:0];
          state <= S_ADD1;
        end

        S_ADD1: begin                   // areg <- a + t  (< 2q)
          areg  <= sum[W-1:0];
          state <= S_ADD2;
        end

        S_ADD2: begin                   // u = areg - q if that does not borrow
          if (sum[W]) areg <= sum[W-1:0];
          state <= S_DONE;
        end

        default: state <= S_IDLE;

      endcase
    end
  end

  // ----------------------------------------------------------------- output
  wire busy = (state != S_IDLE) & (state != S_DONE);

  assign uo_out  = areg[W-1:16];
  assign uio_out = {1'b0, busy, (state == S_DONE), 5'b00000};
  assign uio_oe  = 8'b1110_0000;

  wire _unused = &{ena, uio_in[7:5], uio_in[2], 1'b0};

endmodule
