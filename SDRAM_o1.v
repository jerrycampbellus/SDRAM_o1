`timescale 1ns / 1ps

module sdram_controller_with_power_modes (
    //------------------------------------------------------
    //                SYSTEM INTERFACE
    //------------------------------------------------------
    input  wire                 clk,       // ~143 MHz
    input  wire                 reset,     // async reset (active high)

    // Example command interface
    input  wire                 i_valid,   
    input  wire [4:0]           i_cmd,     
    // Possible i_cmd definitions (example):
    //   0 = NOP
    //   1 = READ
    //   2 = WRITE
    //   3 = REFRESH_REQ
    //   4 = ENTER_PWRDOWN
    //   5 = ENTER_SELFREF
    //   6 = EXIT_LOWPOWER
    //   etc.

    input  wire [12:0]          i_row_addr,
    input  wire [8:0]           i_col_addr,
    input  wire [1:0]           i_bank_addr,
    input  wire [15:0]          i_wdata,
    output reg  [15:0]          o_rdata,
    output reg                  o_ready,

    //------------------------------------------------------
    //                SDRAM INTERFACE (x16)
    //------------------------------------------------------
    output reg  cs_n,
    output reg  ras_n,
    output reg  cas_n,
    output reg  we_n,
    output reg  [1:0]   sdram_ba,
    output reg  [12:0]  sdram_addr,
    inout  wire [15:0]  sdram_dq,
    output reg          cke,
    output reg  [1:0]   dqm
);

    //------------------------------------------------------
    //  PARAMETERIZED TIMING (for ~143MHz, -7 device)
    //------------------------------------------------------
    parameter tRCD_CYCLES   =  3;
    parameter tRP_CYCLES    =  3;
    parameter tRC_CYCLES    =  9;
    parameter tMRD_CYCLES   =  2;
    parameter tDPL_CYCLES   =  2;
    parameter CAS_LAT       =  3;

    // Refresh interval in clock cycles (~7.8us / 7ns = ~1114).
    parameter tREF_CYCLES   = 1100;

    // Self-refresh exit time (tXS). Example ~75ns => ~11 cycles @7ns.
    parameter tXS_CYCLES    = 11;

    // Power-up wait (~100us => ~14285 cycles @143MHz). Use margin:
    parameter PWRUP_WAIT    = 16000;

    //------------------------------------------------------
    //  STATE MACHINE
    //------------------------------------------------------
    localparam ST_RESET          = 5'd0,
               ST_POWERUP        = 5'd1,
               ST_PRECHARGE      = 5'd2,
               ST_AR1            = 5'd3,
               ST_AR2            = 5'd4,
               ST_LOAD_MODE      = 5'd5,
               ST_READY          = 5'd6,
               ST_ACTIVE         = 5'd7,
               ST_READ           = 5'd8,
               ST_WRITE          = 5'd9,
               ST_REF_REQ        = 5'd10,
               ST_REF_EXEC       = 5'd11,
               ST_PWRDOWN_ENTRY  = 5'd12,
               ST_PWRDOWN        = 5'd13,
               ST_SELFREF_ENTRY  = 5'd14,
               ST_SELFREF        = 5'd15,
               ST_SELFREF_EXIT   = 5'd16;

    reg [4:0] current_state, next_state;

    // Timers/counters
    reg [15:0] pwrup_cnt;
    reg        powerup_done;
    reg [15:0] refresh_timer;
    wire       refresh_due;
    reg [7:0]  cmd_timer;
    reg        wait_done;
    reg [15:0] dq_out_reg;
    reg        dq_oe; 

    // Driving DQ bus
    assign sdram_dq = dq_oe ? dq_out_reg : 16'hzzzz;

    // Capture READ data (simplified)
    always @(posedge clk) begin
        if ((current_state == ST_READ) && wait_done) begin
            o_rdata <= sdram_dq;
        end
    end

    //------------------------------------------------------
    //  SEQUENTIAL STATE REGISTER
    //------------------------------------------------------
    always @(posedge clk or posedge reset) begin
        if (reset)
            current_state <= ST_RESET;
        else
            current_state <= next_state;
    end

    //------------------------------------------------------
    //  COMBINATIONAL NEXT-STATE LOGIC
    //------------------------------------------------------
    always @(*) begin
        next_state = current_state;
        case (current_state)
        ST_RESET: begin
            next_state = ST_POWERUP;
        end

        ST_POWERUP: begin
            if (powerup_done)
                next_state = ST_PRECHARGE;
        end

        ST_PRECHARGE: begin
            if (wait_done)
                next_state = ST_AR1;
        end

        ST_AR1: begin
            if (wait_done)
                next_state = ST_AR2;
        end

        ST_AR2: begin
            if (wait_done)
                next_state = ST_LOAD_MODE;
        end

        ST_LOAD_MODE: begin
            if (wait_done)
                next_state = ST_READY;
        end

        ST_READY: begin
            // Idle. Check refresh or user cmds.
            if (refresh_due)
                next_state = ST_REF_REQ;
            else if (i_valid) begin
                case (i_cmd)
                    5'h1: next_state = ST_ACTIVE; // READ
                    5'h2: next_state = ST_ACTIVE; // WRITE
                    5'h3: next_state = ST_REF_REQ; // manual refresh
                    5'h4: next_state = ST_PWRDOWN_ENTRY; // enter pwr-down
                    5'h5: next_state = ST_SELFREF_ENTRY; // self-refresh
                    default: next_state = ST_READY;
                endcase
            end
        end

        ST_REF_REQ: begin
            next_state = ST_REF_EXEC;
        end

        ST_REF_EXEC: begin
            if (wait_done)
                next_state = ST_READY;
        end

        ST_ACTIVE: begin
            if (wait_done) begin
                if (i_cmd == 5'h1) // READ
                    next_state = ST_READ;
                else if (i_cmd == 5'h2) // WRITE
                    next_state = ST_WRITE;
                else
                    next_state = ST_READY;
            end
        end

        ST_READ: begin
            if (wait_done)
                next_state = ST_READY;
        end

        ST_WRITE: begin
            if (wait_done)
                next_state = ST_READY;
        end

        // Power-Down
        ST_PWRDOWN_ENTRY: begin
            next_state = ST_PWRDOWN;
        end
        ST_PWRDOWN: begin
            // Remain in PWRDOWN until user issues EXIT_LOWPOWER
            if (i_valid && (i_cmd == 5'h6))
                next_state = ST_READY;
        end

        // Self-Refresh
        ST_SELFREF_ENTRY: begin
            next_state = ST_SELFREF;
        end
        ST_SELFREF: begin
            // Wait for EXIT_LOWPOWER
            if (i_valid && (i_cmd == 5'h6))
                next_state = ST_SELFREF_EXIT;
        end
        ST_SELFREF_EXIT: begin
            if (wait_done)
                next_state = ST_READY;
        end

        endcase
    end

    //------------------------------------------------------
    //  COMMAND TIMER / WAIT_DONE
    //------------------------------------------------------
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            cmd_timer <= 0;
            wait_done <= 1'b0;
        end else begin
            wait_done <= 1'b0;

            case (current_state)
            ST_PRECHARGE: begin
                if (cmd_timer < tRP_CYCLES)
                    cmd_timer <= cmd_timer + 1;
                else begin
                    wait_done <= 1'b1;
                    cmd_timer <= 0;
                end
            end
            ST_AR1, ST_AR2, ST_REF_EXEC: begin
                if (cmd_timer < tRC_CYCLES)
                    cmd_timer <= cmd_timer + 1;
                else begin
                    wait_done <= 1'b1;
                    cmd_timer <= 0;
                end
            end
            ST_LOAD_MODE: begin
                if (cmd_timer < tMRD_CYCLES)
                    cmd_timer <= cmd_timer + 1;
                else begin
                    wait_done <= 1'b1;
                    cmd_timer <= 0;
                end
            end
            ST_ACTIVE: begin
                if (cmd_timer < tRCD_CYCLES)
                    cmd_timer <= cmd_timer + 1;
                else begin
                    wait_done <= 1'b1;
                    cmd_timer <= 0;
                end
            end
            ST_READ: begin
                if (cmd_timer < CAS_LAT)
                    cmd_timer <= cmd_timer + 1;
                else begin
                    wait_done <= 1'b1;
                    cmd_timer <= 0;
                end
            end
            ST_WRITE: begin
                if (cmd_timer < tDPL_CYCLES)
                    cmd_timer <= cmd_timer + 1;
                else begin
                    wait_done <= 1'b1;
                    cmd_timer <= 0;
                end
            end
            ST_SELFREF_EXIT: begin
                // Wait tXS
                if (cmd_timer < tXS_CYCLES)
                    cmd_timer <= cmd_timer + 1;
                else begin
                    wait_done <= 1'b1;
                    cmd_timer <= 0;
                end
            end
            default: cmd_timer <= 0;
            endcase
        end
    end

    //------------------------------------------------------
    //  POWER-UP WAIT
    //------------------------------------------------------
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            pwrup_cnt    <= 0;
            powerup_done <= 1'b0;
        end else begin
            if (!powerup_done) begin
                if (pwrup_cnt < PWRUP_WAIT)
                    pwrup_cnt <= pwrup_cnt + 1;
                else
                    powerup_done <= 1'b1;
            end
        end
    end

    //------------------------------------------------------
    //  REFRESH TIMER
    //------------------------------------------------------
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            refresh_timer <= 0;
        end else begin
            // For simplicity, keep counting even in self-refresh/power-down
            if (current_state == ST_REF_EXEC)
                refresh_timer <= 0;
            else
                refresh_timer <= refresh_timer + 1;
        end
    end
    assign refresh_due = (refresh_timer >= tREF_CYCLES);

    //------------------------------------------------------
    //  OUTPUT / COMMAND SIGNALS
    //------------------------------------------------------
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            cs_n       <= 1'b1;
            ras_n      <= 1'b1;
            cas_n      <= 1'b1;
            we_n       <= 1'b1;
            sdram_ba   <= 2'b00;
            sdram_addr <= 13'h0000;
            cke        <= 1'b1;
            dqm        <= 2'b00;
            dq_oe      <= 1'b0;
            dq_out_reg <= 16'h0000;
            o_ready    <= 1'b0;
        end else begin
            // Defaults = NOP
            cs_n       <= 1'b0;
            ras_n      <= 1'b1;
            cas_n      <= 1'b1;
            we_n       <= 1'b1;
            sdram_ba   <= 2'b00;
            sdram_addr <= 13'h0000;
            cke        <= 1'b1;
            dqm        <= 2'b00;
            dq_oe      <= 1'b0;
            dq_out_reg <= i_wdata;
            o_ready    <= (current_state == ST_READY);

            case (current_state)
            ST_POWERUP: begin
                // NOP while powering up
            end
            ST_PRECHARGE: begin
                // PRECHARGE ALL => RAS=0, WE=0, A10=1
                ras_n         <= 1'b0;
                we_n          <= 1'b0;
                sdram_addr[10]<= 1'b1;
            end
            ST_AR1, ST_AR2, ST_REF_EXEC: begin
                // AUTO REFRESH => RAS=0, CAS=0, WE=1
                ras_n <= 1'b0;
                cas_n <= 1'b0;
                we_n  <= 1'b1;
            end
            ST_LOAD_MODE: begin
                // LOAD MODE => RAS=0, CAS=0, WE=0
                ras_n <= 1'b0;
                cas_n <= 1'b0;
                we_n  <= 1'b0;
                // Example: CAS=3, BL=4, sequential, normal writes
                sdram_addr <= 13'b000_0110_1010; 
            end
            ST_ACTIVE: begin
                // ACTIVE => RAS=0
                ras_n      <= 1'b0;
                sdram_ba   <= i_bank_addr;
                sdram_addr <= i_row_addr;
            end
            ST_READ: begin
                // READ => CAS=0
                cas_n      <= 1'b0;
                sdram_ba   <= i_bank_addr;
                sdram_addr <= i_col_addr;
            end
            ST_WRITE: begin
                // WRITE => CAS=0, WE=0
                cas_n     <= 1'b0;
                we_n      <= 1'b0;
                sdram_ba  <= i_bank_addr;
                sdram_addr<= i_col_addr;
                dq_oe     <= 1'b1;
                dq_out_reg<= i_wdata;
            end
            ST_PWRDOWN_ENTRY: begin
                // Next cycle => ST_PWRDOWN with cke=0
            end
            ST_PWRDOWN: begin
                cke <= 1'b0; // internal clock suspended
            end
            ST_SELFREF_ENTRY: begin
                // Self-refresh => RAS=0, CAS=0, WE=? (datasheet-specific), cke=0
                ras_n <= 1'b0;
                cas_n <= 1'b0;
                // Some datasheets show WE=1 or 0 for self-refresh
                we_n  <= 1'b1; 
                cke   <= 1'b0;
            end
            ST_SELFREF: begin
                cke <= 1'b0; // remain in self-refresh
            end
            ST_SELFREF_EXIT: begin
                // Raise cke=1, wait tXS
                cke <= 1'b1;
            end
            endcase

            // If transitioning to power-down explicitly
            if (current_state == ST_PWRDOWN_ENTRY) begin
                cke <= 1'b0;
            end
        end
    end
endmodule
