`timescale 1ns / 1ps

module sdram_controller_tb;

// --------------------------------------------------
// Parameters / local definitions for the test bench
// --------------------------------------------------
localparam CLK_PERIOD = 7;  // ~143 MHz => 7ns period

// Commands (matching i_cmd bits in the controller)
localparam CMD_NOP          = 5'd0;
localparam CMD_READ         = 5'd1;
localparam CMD_WRITE        = 5'd2;
localparam CMD_REFRESH_REQ  = 5'd3;
localparam CMD_ENTER_PWRDN  = 5'd4;
localparam CMD_ENTER_SELFREF= 5'd5;
localparam CMD_EXIT_LOWPOWER= 5'd6; 

// --------------------------------------------------
// DUT (Device Under Test) I/O signals
// --------------------------------------------------
reg  clk;
reg  reset;
reg  i_valid;
reg  [4:0]  i_cmd;
reg  [12:0] i_row_addr;
reg  [8:0]  i_col_addr;
reg  [1:0]  i_bank_addr;
reg  [15:0] i_wdata;
wire [15:0] o_rdata;
wire        o_ready;

// SDRAM pins from the controller
wire  cs_n;
wire  ras_n;
wire  cas_n;
wire  we_n;
wire  [1:0]  sdram_ba;
wire  [12:0] sdram_addr;
wire  [15:0] sdram_dq;
wire         cke;
wire  [1:0]  dqm;

// --------------------------------------------------
// Instantiate the SDRAM Controller
// --------------------------------------------------
sdram_controller_with_power_modes dut (
    .clk         (clk),
    .reset       (reset),
    .i_valid     (i_valid),
    .i_cmd       (i_cmd),
    .i_row_addr  (i_row_addr),
    .i_col_addr  (i_col_addr),
    .i_bank_addr (i_bank_addr),
    .i_wdata     (i_wdata),
    .o_rdata     (o_rdata),
    .o_ready     (o_ready),

    .cs_n        (cs_n),
    .ras_n       (ras_n),
    .cas_n       (cas_n),
    .we_n        (we_n),
    .sdram_ba    (sdram_ba),
    .sdram_addr  (sdram_addr),
    .sdram_dq    (sdram_dq),
    .cke         (cke),
    .dqm         (dqm)
);

// --------------------------------------------------
// SIMPLE STUB for sdram_dq (no real DRAM model)
// Just tie sdram_dq to 'Z' or some pattern
// In a real test, you'd instantiate an SDRAM model
// --------------------------------------------------
assign sdram_dq = 16'hzzzz; // Placeholder
// For a more realistic test, integrate a simulation
// model of the SDRAM that responds to commands.

// --------------------------------------------------
// Clock Generation
// --------------------------------------------------
initial begin
    clk = 1'b0;
    forever #(CLK_PERIOD/2) clk = ~clk;
end

// --------------------------------------------------
// Reset Logic
// --------------------------------------------------
initial begin
    reset = 1'b1;
    #100;        // hold reset for 100ns
    reset = 1'b0;
end

// --------------------------------------------------
// Test Stimulus
// --------------------------------------------------
initial begin
    // Initialize
    $dumpfile("SDRAM_o1.vcd");
	$dumpvars(0, SDRAM_o1_tb);
	i_valid     = 1'b0;
    i_cmd       = CMD_NOP;
    i_row_addr  = 13'h0000;
    i_col_addr  = 9'h000;
    i_bank_addr = 2'b00;
    i_wdata     = 16'h1234;

    // Wait some time after reset
    #(500);

    // The controller does power-up for ~100us (~16000 cycles).
    // We'll wait ~18000 cycles or so to let it initialize
    // (We won't literally wait 18000 * 7ns here, just a smaller wait in simulation
    //  or you can reduce the PWRUP_WAIT parameter for demo)
    #(200000);  // 200,000 ns = 200 us (enough for demonstration)

    $display("Time=%t : Controller should now be in ST_READY.\n",$time);

    // Example 1: Issue a READ command
    @(posedge clk);
    i_valid    <= 1'b1;
    i_cmd      <= CMD_READ;
    i_row_addr <= 13'h00A5;
    i_col_addr <= 9'h03F;
    i_bank_addr<= 2'b01;
    #(CLK_PERIOD);
    i_valid    <= 1'b0;
    i_cmd      <= CMD_NOP;

    // Wait for READ to finish
    #(100);

    // Example 2: Issue a WRITE command
    $display("Time=%t : Now doing a WRITE command.\n", $time);
    i_wdata    <= 16'hDEAD;
    @(posedge clk);
    i_valid    <= 1'b1;
    i_cmd      <= CMD_WRITE;
    i_row_addr <= 13'h0123;
    i_col_addr <= 9'h055;
    i_bank_addr<= 2'b10;
    #(CLK_PERIOD);
    i_valid    <= 1'b0;
    i_cmd      <= CMD_NOP;

    // Wait a bit
    #(200);

    // Example 3: Enter Power-Down mode
    $display("Time=%t : Requesting Power-Down.\n", $time);
    @(posedge clk);
    i_valid <= 1'b1;
    i_cmd   <= CMD_ENTER_PWRDN;
    #(CLK_PERIOD);
    i_valid <= 1'b0;
    i_cmd   <= CMD_NOP;

    // Remain in power-down for ~500ns
    #(500);

    // Exit low-power
    $display("Time=%t : Exiting Power-Down.\n", $time);
    @(posedge clk);
    i_valid <= 1'b1;
    i_cmd   <= CMD_EXIT_LOWPOWER;
    #(CLK_PERIOD);
    i_valid <= 1'b0;
    i_cmd   <= CMD_NOP;
    #(1000);

    // Example 4: Enter Self-Refresh
    $display("Time=%t : Requesting Self-Refresh.\n", $time);
    @(posedge clk);
    i_valid <= 1'b1;
    i_cmd   <= CMD_ENTER_SELFREF;
    #(CLK_PERIOD);
    i_valid <= 1'b0;
    i_cmd   <= CMD_NOP;

    // Wait inside self-refresh
    #(2000);

    // Exit Self-Refresh
    $display("Time=%t : Exiting Self-Refresh.\n", $time);
    @(posedge clk);
    i_valid <= 1'b1;
    i_cmd   <= CMD_EXIT_LOWPOWER;
    #(CLK_PERIOD);
    i_valid <= 1'b0;
    i_cmd   <= CMD_NOP;

    // Wait some cycles
    #(2000);

    // Finish Simulation
    $display("Time=%t : Testbench simulation complete.\n",$time);
    $stop;
end

endmodule
