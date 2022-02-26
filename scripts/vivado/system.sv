`timescale 1 ns / 1 ps

module system (
	input            clk,
        output  logic [15:0] led,
        output  reg   [2:0]  rgb0, rgb1,
        output  wire  seg7A, seg7B, seg7C, seg7D, seg7E, seg7F, seg7G, dp,
        output  wire  [7:0]  anodes,
        // output  wire  [15:0] monitor,        // ja, jb, pods for digital logging
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

	wire mem_valid;
	wire mem_instr;
        reg ram_ready;
	wire [31:0] mem_addr;
	wire [31:0] mem_wdata;
	wire [3:0] mem_wstrb;
	reg [31:0] ram_rdata;
	wire  [31:0] mem_rdata = ram_rdata;

	wire mem_la_read;
	wire mem_la_write;
	wire [31:0] mem_la_addr;
	wire [31:0] mem_la_wdata;
	wire [3:0] mem_la_wstrb;

	wire        simpleuart_reg_div_sel = mem_valid && (mem_addr == 32'h 0200_0004);
	wire [31:0] simpleuart_reg_div_do;

	wire        simpleuart_reg_dat_sel = mem_valid && (mem_addr == 32'h 0200_0008);
	wire [31:0] simpleuart_reg_dat_do;

        wire   simpleuart_reg_dat_wait;


        enum { RV_STOP, RV_RUN } j1_st;
        reg [5:0]   longstep;           // ref DEBOUNCE, size of reg is what counts
        reg [3:0]   step;                               // single step
        wire        step_up = step[1] & !step[0];
        wire        step_btn = (nstep ^ btnC);               // active high

        reg         rv_state;
	assign mem_ready = ((rv_state == RV_RUN) | step_up) & (
                                ram_ready ||
			        simpleuart_reg_div_sel ||
                     (simpleuart_reg_dat_sel && !simpleuart_reg_dat_wait));

	assign mem_rdata = ram_ready ? ram_rdata :
                           simpleuart_reg_div_sel ? simpleuart_reg_div_do :
                           simpleuart_reg_dat_sel ? simpleuart_reg_dat_do : 32'h 0000_0000;

        reg [LOG_CLK_DIVIDE:0]   divide;
        always_ff @(posedge clk) begin divide <= divide + 1; end

        always_ff @(posedge clk) begin
            if (!resetn) begin
                rv_state <= RV_STOP;
            end else begin
                step[0]         <= step[1];                 // set step_up and down

                if (divide[LOG_DEBOUNCE:0] == 0) begin      // slow speed
                    step[3:1] <= {step_btn, step[3:2]};
                    if (step[1] & !(&longstep))        // any bits clear
                        longstep <= longstep +1;
                    else if (!step[1])                  // button not pressed
                        longstep <= 0;                  // otherwise stick at 11111
                end

                if (divide[LOG_DEBOUNCE:LOG_DEBOUNCE-2] == 3'b000) begin    // 1/8 duty cycle

                    rgb0[0] <= (rv_state == RV_STOP);
                    rgb0[1] <= (rv_state == RV_RUN);

	            rgb1[0] <=  mem_instr;
                    rgb1[1] <= mem_ready;
	            rgb1[2] <=  mem_valid;

                end else begin
                    rgb0 <= 3'b000;       // PWM
                    rgb1 <= 3'b000;       // PWM
                end
            end
        end

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
            ) picorv32_core (
		.clk         (clk         ),
		.resetn      (resetn      ),
		.trap        (led[9]        ),
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

	simpleuart simpleuart (
		.clk         (clk         ),
		.resetn      (resetn      ),

		.ser_tx      (ser_tx      ),
		.ser_rx      (ser_rx      ),

		.reg_div_we  (simpleuart_reg_div_sel ? mem_wstrb : 4'b 0000),
		.reg_div_di  (mem_wdata),
		.reg_div_do  (simpleuart_reg_div_do),

		.reg_dat_we  (simpleuart_reg_dat_sel ? mem_wstrb[0] : 1'b 0),
		.reg_dat_re  (simpleuart_reg_dat_sel && !mem_wstrb),
		.reg_dat_di  (mem_wdata),
		.reg_dat_do  (simpleuart_reg_dat_do),
		.reg_dat_wait(simpleuart_reg_dat_wait)
	);

	reg [31:0] memory [0:MEM_SIZE-1];
	initial $readmemh("firmware.hex", memory);

	reg [31:0] m_read_data;
	reg m_read_en;

	generate if (FAST_MEMORY) begin
		always @(posedge clk) begin
			ram_ready <= 1;
			led[8] <= 1;    // out_byte_en <= 0;
			ram_rdata <= memory[mem_la_addr >> 2];
			if (mem_la_write && (mem_la_addr >> 2) < MEM_SIZE) begin
				if (mem_la_wstrb[0]) memory[mem_la_addr >> 2][ 7: 0] <= mem_la_wdata[ 7: 0];
				if (mem_la_wstrb[1]) memory[mem_la_addr >> 2][15: 8] <= mem_la_wdata[15: 8];
				if (mem_la_wstrb[2]) memory[mem_la_addr >> 2][23:16] <= mem_la_wdata[23:16];
				if (mem_la_wstrb[3]) memory[mem_la_addr >> 2][31:24] <= mem_la_wdata[31:24];
			end
			else
			if (mem_la_write && mem_la_addr == 32'h1000_0000) begin
				led[8] <= 1;    // out_byte_en <= 1;
				led[7:0] <= mem_la_wdata;    // out_byte <= mem_la_wdata;
			end
		end
	end else begin
		always @(posedge clk) begin
			m_read_en <= 0;
			ram_ready <= mem_valid && !ram_ready && m_read_en;

			m_read_data <= memory[mem_addr >> 2];
			ram_rdata <= m_read_data;

			led[8] <= 0;    // out_byte_en <= 0;

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
				mem_valid && !ram_ready && |mem_wstrb && mem_addr == 32'h1000_0000: begin
					led[8] <= 1;            // out_byte_en <= 1;
					led[7:0] <= mem_wdata; // out_byte
					ram_ready <= 1;
				end
			endcase
		end
	end endgenerate
endmodule
