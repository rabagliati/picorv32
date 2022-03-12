`timescale 1 ns / 1 ps

module system (
	input            CLK100MHZ,
        input   wire [15:0] sw,       	// Slide switches.
        input   wire        uart_rx, 	// UART Receive pin.    HOST -> FT2232 -> A9
        output  wire        uart_tx, 	// UART transmit pin.   HOST <- FT2232 <- D10

        // output  wire        spi1_cs0n,
        // output  wire        spi1_mosi,
        // input   wire        spi1_miso,
        output  logic [15:0] led,
        output  reg   [2:0]  rgb0, rgb1,
        output  wire  seg7A, seg7B, seg7C, seg7D, seg7E, seg7F, seg7G, dp,
        output  wire  [7:0]  anodes,
        output  wire  [15:0] monitor,        // ja, jb, pods for digital logging
        input   wire  btnC, btnU, btnD, btnL, btnR,
        input   wire  nstep,			// single step active low XXX HIGH
	input            resetn
);
	parameter [0:0] BARREL_SHIFTER = 1;
	parameter [0:0] ENABLE_MUL = 1;
	parameter [0:0] ENABLE_DIV = 1;
	parameter [0:0] ENABLE_FAST_MUL = 0;
	parameter [0:0] ENABLE_COMPRESSED = 1;
	parameter [0:0] ENABLE_COUNTERS = 1;
	parameter [0:0] ENABLE_IRQ_QREGS = 0;

	parameter integer MEM_WORDS = 256;
	parameter [31:0] STACKADDR = (4*MEM_WORDS);       // end of memory
	parameter [31:0] PROGADDR_RESET = 32'h 0010_0000; // 1 MB into flash
	parameter [31:0] PROGADDR_IRQ = 32'h 0000_0000;
	// set this to 0 for better timing but less performance/MHz
	parameter FAST_MEMORY = 1;

	// 4096 32bit words = 16kB memory
	parameter MEM_SIZE = 4096;

        parameter LOG_CLK_DIVIDE = 20;	// bigger than all other LOG_*
        parameter LOG_DEBOUNCE = 18;        // User interface, readable on 7seg digit enables
        parameter LOG_SPI_CLK = 2;

        reg [LOG_CLK_DIVIDE:0]   divide;
        always_ff @(posedge CLK100MHZ) begin divide <= divide + 1; end
        wire            clk = divide[1];

        assign          led[15:0] = sw[15:0];
	wire            mem_valid;
	wire            mem_instr;
        reg             ram_ready;
	wire [31:0]     mem_addr;
	wire [31:0]     mem_wdata;
	wire [3:0]      mem_wstrb;
	reg [31:0]      ram_rdata;
	logic [31:0]    mem_rdata;
        logic           mem_ready;

	wire            mem_la_read;
	wire            mem_la_write;
	wire [31:0]     mem_la_addr;
	wire [31:0]     mem_la_wdata;
	wire [3:0]      mem_la_wstrb;

	wire            simpleuart_reg_div_sel = mem_valid && (mem_addr == 32'h 0200_0004);
	wire [31:0]     simpleuart_reg_div_do;

	wire            simpleuart_reg_dat_sel = mem_valid && (mem_addr == 32'h 0200_0008);
	wire [31:0]     simpleuart_reg_dat_do;

        wire            simpleuart_reg_dat_wait;


        enum { RV_STOP, RV_RUN } j1_st;
        reg [7:0]   longstep;           // ref DEBOUNCE, size of reg is what counts
                                        // (longstep == 7'b1111111) - resting at top
                                        // (longstep == 7'b1111110) - blip to signal
        reg [3:0]   step;                               // single step
        reg [4:0]   buttons[1:0];   // C, L, R, U, D
        wire [4:0]  button_up = ~buttons[0] & buttons[1];

        wire        step_up = (longstep != 7'b1111111) & step[1] & !step[0];

        reg         rv_state;
        wire        breaking = (sw == mem_addr[15:0]) & mem_instr;

        always_ff @(posedge clk) begin
            if (!resetn | button_up[0])             // *C, L, R, U, D
                rv_state <= RV_STOP;
            else if (button_up[2])                  // C, L, *R, U, D
                rv_state <= RV_RUN;
            else if (longstep == 7'b1111110)
                rv_state <= RV_RUN;
            else if (breaking)
                rv_state <= RV_STOP;
        end
        reg         display_data;
        always_ff @(posedge clk) begin
            if (!resetn)
                display_data <= 0;
            else if (button_up[3])                  // C, L, R, *U, D
                display_data <= !display_data;
        end

//         assign mem_ready = simpleuart_reg_div_sel ? 1 :
//                            simpleuart_reg_dat_sel ? !simpleuart_reg_dat_wait :
//                                     !mem_addr[14] ? ram_ready && ((rv_state == RV_RUN) | step_up | button_up[0]) : 0;
// 
//         assign mem_rdata = simpleuart_reg_div_sel ? simpleuart_reg_div_do :
//                            simpleuart_reg_dat_sel ? simpleuart_reg_dat_do :
//                                         ram_ready ? ram_rdata : 32'h 0000_0000;

        always_comb begin
             case (1)
                 simpleuart_reg_div_sel: begin
                     mem_rdata = simpleuart_reg_div_do;
                     mem_ready = 1;
                 end
                 simpleuart_reg_dat_sel: begin
                     mem_rdata = simpleuart_reg_dat_do;
                     mem_ready = !simpleuart_reg_dat_wait;
                 end
                 !mem_addr[14]: begin
                     mem_rdata = ram_rdata;
                     mem_ready = ((rv_state == RV_RUN) | step_up | button_up[0]) & ram_ready;  // *C, L, *R or longstep, U, D
                 end
                 default: begin
                     mem_rdata = 32'h 0000_0000;
                     mem_ready = 0;
                 end
            endcase
        end

        always_ff @(posedge clk) begin
            if (!resetn) begin
                rgb0[0] <= 0;
                rgb0[1] <= 0;
                rgb0[2] <= 0;
            end else begin

                // if (divide[2:0] == 3'b 000)
                step[0]         <= step[1];                 // set step_up and down
                buttons[1]      <= buttons[0];   // C, L, R, U, D

                if (divide[LOG_DEBOUNCE:3] == 0) begin      // slow speed
                    step[3:1] <= {nstep, step[3:2]};
                    if (step[1] & !(&longstep))        // any bits clear
                        longstep <= longstep +1;
                    else if (!step[1])                  // button not pressed
                        longstep <= 0;                  // otherwise stick at 111111
                    buttons[0] <= { btnD, btnU, btnR, btnL, btnC};
                end

                if (divide[LOG_DEBOUNCE:LOG_DEBOUNCE-2] == 3'b000) begin    // 1/8 duty cycle

                    rgb0[0] <= (rv_state == RV_STOP);
                    rgb0[1] <= (rv_state == RV_RUN);

	            rgb1[0] <= mem_instr;
                    rgb1[1] <= mem_ready;
	            rgb1[2] <= breaking;

                end else begin
                    rgb0 <= 3'b000;       // PWM
                    rgb1 <= 3'b000;       // PWM
                end
            end
        end

        wire trap;
	picorv32 #(
		.STACKADDR(STACKADDR),
		.PROGADDR_RESET(PROGADDR_RESET),
		.PROGADDR_IRQ(PROGADDR_IRQ),
		.BARREL_SHIFTER(BARREL_SHIFTER),
		.COMPRESSED_ISA(ENABLE_COMPRESSED),
		.ENABLE_COUNTERS(ENABLE_COUNTERS),
		.ENABLE_MUL(ENABLE_MUL),
		.ENABLE_DIV(ENABLE_DIV),
		.ENABLE_FAST_MUL(ENABLE_FAST_MUL),
		.ENABLE_IRQ(1),
		.ENABLE_IRQ_QREGS(ENABLE_IRQ_QREGS)
            ) _cpu (
		.clk         (divide[2]   ),
		.resetn      (resetn      ),
		.trap        (trap        ),
		.mem_valid   (mem_valid   ),
		.mem_instr   (mem_instr   ),
		.mem_ready   (mem_ready   ),
		.mem_addr    (mem_addr    ),
		.mem_wdata   (mem_wdata   ),
		.mem_wstrb   (mem_wstrb   ),
		.mem_rdata   (mem_rdata   ),
		.mem_la_read (mem_la_read ),
		.mem_la_write(mem_la_write),
		.mem_la_addr (mem_la_addr ),
		.mem_la_wdata(mem_la_wdata),
		.mem_la_wstrb(mem_la_wstrb)
	);

	simpleuart _simpleuart (
		.clk         (clk         ),
		.resetn      (resetn      ),

		.ser_tx      (uart_tx      ),
		.ser_rx      (uart_rx      ),

		.reg_div_we  (simpleuart_reg_div_sel ? mem_wstrb : 4'b 0000),
		.reg_div_di  (mem_wdata),
		.reg_div_do  (simpleuart_reg_div_do),

		.reg_dat_we  (simpleuart_reg_dat_sel ? mem_wstrb[0] : 1'b 0),
		.reg_dat_re  (simpleuart_reg_dat_sel && !mem_wstrb),
		.reg_dat_di  (mem_wdata),
		.reg_dat_do  (simpleuart_reg_dat_do),
		.reg_dat_wait(simpleuart_reg_dat_wait)
	);

        Binary_To_7Segment _b7s(
            .i_Clk(clk),
            .i_LeftHex(display_data ? mem_rdata[31:16] : mem_addr[31:16]),
            .i_RightHex(display_data ? mem_rdata[15:0] : mem_addr[15:0]),
            .i_Mux(divide[LOG_DEBOUNCE:LOG_DEBOUNCE-2]),        // digit selectors
            .o_Segment_A(seg7A),
            .o_Segment_B(seg7B),
            .o_Segment_C(seg7C),
            .o_Segment_D(seg7D),
            .o_Segment_E(seg7E),
            .o_Segment_F(seg7F),
            .o_Segment_G(seg7G),
            .o_Segment_dp(dp),
            .o_Digits(anodes)
        );

	reg [31:0] memory [0:MEM_SIZE-1];
	initial $readmemh("firmware.hex", memory);

	reg [31:0] m_read_data;
	reg m_read_en;

	generate if (FAST_MEMORY) begin
		always @(posedge clk) begin
                        ram_ready <= 1;
			ram_rdata <= memory[mem_la_addr >> 2];
			if (mem_la_write && (mem_la_addr >> 2) < MEM_SIZE) begin
				if (mem_la_wstrb[0]) memory[mem_la_addr >> 2][ 7: 0] <= mem_la_wdata[ 7: 0];
				if (mem_la_wstrb[1]) memory[mem_la_addr >> 2][15: 8] <= mem_la_wdata[15: 8];
				if (mem_la_wstrb[2]) memory[mem_la_addr >> 2][23:16] <= mem_la_wdata[23:16];
				if (mem_la_wstrb[3]) memory[mem_la_addr >> 2][31:24] <= mem_la_wdata[31:24];
			end
		end
	end else begin
		always @(posedge clk) begin
			m_read_en <= 0;
			ram_ready <= mem_valid && !ram_ready && m_read_en;

			m_read_data <= memory[mem_addr >> 2];
			ram_rdata <= m_read_data;

			(* parallel_case *)
			case (1)
				mem_valid && !ram_ready && !mem_wstrb && (mem_addr >> 2) < MEM_SIZE: begin
					m_read_en <= 1;
				end
				mem_valid && !ram_ready && |mem_wstrb && (mem_addr >> 2) < MEM_SIZE: begin
					if (mem_wstrb[0]) memory[mem_addr >> 2][ 7: 0] <= mem_wdata[ 7: 0];
					if (mem_wstrb[1]) memory[mem_addr >> 2][15: 8] <= mem_wdata[15: 8];
					if (mem_wstrb[2]) memory[mem_addr >> 2][23:16] <= mem_wdata[23:16];
					if (mem_wstrb[3]) memory[mem_addr >> 2][31:24] <= mem_wdata[31:24];
					ram_ready <= 1;
				end
			endcase
		end
	end endgenerate
        wire spi1_clk;
        assign spi1_clk    = divide[LOG_SPI_CLK];  // 54 Mhz max for 03 read
        wire cfgclk, cfgmclk, eos, preq;

        STARTUPE2 #(
            .PROG_USR("FALSE"),     // Activate program event security feature. Requires encrypted bitstreams.
            .SIM_CCLK_FREQ(0.0)     // Set the Configuration Clock Frequency(ns) for simulation.
        ) STARTUPE2_inst (
            .CFGCLK(cfgclk),        // 1-bit output: Configuration main clock output
            .CFGMCLK(cfgmclk),      // 1-bit output: Configuration internal oscillator clock output
            .EOS(eos),              // 1-bit output: Active high output signal indicating the End Of Startup.
            .PREQ(preq),            // 1-bit output: PROGRAM request to fabric output
            .CLK(0),                // 1-bit input: User start-up clock input
            .GSR(0),                // 1-bit input: Global Set/Reset input (GSR cannot be used for the port name)
            .GTS(0),                // 1-bit input: Global 3-state input (GTS cannot be used for the port name)
            .KEYCLEARB(0),          // 1-bit input: Clear AES Decrypter Key input from Battery-Backed RAM (BBRAM)
            .PACK(0),               // 1-bit input: PROGRAM acknowledge input
            .USRCCLKO(spi1_clk),    // 1-bit input: User CCLK input
            .USRCCLKTS(0),          // 1-bit input: User CCLK 3-state enable input
            .USRDONEO(1),           // 1-bit input: User DONE pin output control
            .USRDONETS(0)           // 1-bit input: User DONE 3-state enable output
        );
// End of STARTUPE2_inst instantiation

        assign monitor[0] = clk;
        assign monitor[1] = mem_instr;
        assign monitor[2] = mem_ready;
        assign monitor[3] = mem_wstrb;
	assign monitor[4] = simpleuart_reg_dat_sel;
	assign monitor[5] = simpleuart_reg_dat_wait;
        assign monitor[6] = uart_rx;
        assign monitor[7] = uart_tx;
        assign monitor[15:8] = mem_la_addr[7:0];
        // assign monitor[15:12] = mem_rdata[3:0];
endmodule
