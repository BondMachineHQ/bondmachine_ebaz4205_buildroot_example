
`timescale 1ns/1ps
module a0(clock_signal, reset_signal, i0, i0_valid , i0_received, o0, o0_valid, o0_received);

	input clock_signal;
	input reset_signal;

	input [7:0] i0;
	input i0_valid;
	output i0_received;
	output [7:0] o0;
	output o0_valid;
	input o0_received;

	wire [7:0] rom_bus;
	wire [9:0] rom_value;

	wire [7:0] a0din;
	wire [7:0] a0dout;
	wire [7:0] a0addr;
	wire a0wren;
	wire a0en;

	p0 p0_instance(clock_signal, reset_signal, rom_bus, rom_value, a0din, a0dout, a0addr, a0wren, a0en, i0, i0_valid , i0_received, o0, o0_valid, o0_received);
	p0rom p0rom_instance(rom_bus, rom_value);
	p0ram p0ram_instance(clock_signal, reset_signal, a0din, a0dout, a0addr, a0wren, a0en);

endmodule

module bondmachine(clk, reset, i0, o0);

	//--------------Input Ports-----------------------
	input clk, reset;
	input [7:0] i0;
	//--------------Output Ports-----------------------
	output [7:0] o0;



	wire [7:0] p0o0;
	wire p0o0_valid;
	wire p0o0_received;
	wire [7:0] i0;
	wire i0_valid;
	wire i0_received;


	//Instantiation of the Processors and Shared Objects
	a0 a0_inst(clk, reset, i0, i0_valid, i0_received, p0o0, p0o0_valid, p0o0_received);

	assign o0 = p0o0;

endmodule
`timescale 1ns/1ps
module p0ram(clk, rst, din, dout, addr, wren, en);

	//--------------Input Ports-----------------------
	input clk;
	input rst;
	input [7:0] addr;
	input [7:0] din;
	input wren;
	input en;

	//--------------Inout Ports-----------------------
	output [7:0] dout;

	//--------------Reg-------------------------------
	reg [7:0] mem [0:255];

	reg [7:0] dout_i;

	// Memory Write Block  
	// Write Operation we = 1 
	always @ (posedge clk) 
	begin : MEM_WRITE 
		integer k; 
		if (rst)
		begin 
			for(k=0;k<256;k=k+1) 
				mem[k] <= #1 8'b0; 
		end 
		else if (wren)
			mem[addr] <= #1 din;
	end 

	// Memory Read Block
	// Read Operation when we = 0 and oe = 1 
	always @ (posedge clk) 
	begin : MEM_READ 
		if (!wren)
			dout_i <= #1 mem[addr];
	end

	assign dout = dout_i;

endmodule 
`timescale 1ns/1ps
module p0rom(input [7:0] rom_bus, output [9:0] rom_value);
	reg [9:0] _rom [0:255];
	initial
	begin
	_rom[0] = 10'b0000000000;
	_rom[1] = 10'b1000000000;
	_rom[2] = 10'b0100000000;
	_rom[3] = 10'b1100000000;
	end
	assign rom_value = _rom[rom_bus];
endmodule
`timescale 1ns/1ps
module p0(clock_signal, reset_signal, rom_bus, rom_value,  ram_din, ram_dout, ram_addr, ram_wren, ram_en, i0, i0_valid, i0_received, o0, o0_valid, o0_received);

	input clock_signal;
	input reset_signal;
	output  [7:0] rom_bus;
	input  [9:0] rom_value;
	input  [7:0] ram_dout;
	output [7:0] ram_din;
	output  [7:0] ram_addr;
	output ram_wren, ram_en;

	input [7:0] i0;
	input i0_valid;
	output i0_received;
	output [7:0] o0;
	output o0_valid;
	input o0_received;

			// Opcodes in the istructions, lenght accourding the number of the selected.
	localparam	I2R=2'b00,          // Input to register
			R2O=2'b01,          // Register to output
			INC=2'b10,          // Increment a register by 1
			J=2'b11;          // Jump to a program location

	localparam	R0=2'b00,		// Registers in the intructions
			R1=2'b01,
			R2=2'b10,
			R3=2'b11;
	localparam			I0=1'b0;
	localparam			O0=1'b0;
	reg [7:0] _auxo0;

	reg [7:0] _ram [0:255];		// Internal processor RAM

	(* KEEP = "TRUE" *) reg [7:0] _pc;		// Program counter

	// The number of registers are 2^R, two letters and an unserscore as identifier , maximum R=8 and 265 rigisters
	(* KEEP = "TRUE" *) reg [7:0] _r0;
	(* KEEP = "TRUE" *) reg [7:0] _r1;
	(* KEEP = "TRUE" *) reg [7:0] _r2;
	(* KEEP = "TRUE" *) reg [7:0] _r3;

	reg i0_recv;

	always @(posedge clock_signal, posedge reset_signal)
	begin
		if (reset_signal)
		begin
			i0_recv <= #1 1'b0;
		end
		else
		begin
			case(rom_value[9:8])
				I2R: begin
					case (rom_value[5])
					I0 : begin
						i0_recv <= #1 1'b1;
					end
					default: begin
						if (!i0_valid)
						begin
							i0_recv <= #1 1'b0;
						end
					end
					endcase
				end
				default: begin
					if (!i0_valid)
					begin
						i0_recv <= #1 1'b0;
					end
				end
			endcase
		end
	end

	reg o0_val;

	always @(posedge clock_signal, posedge reset_signal)
	begin
		if (reset_signal)
		begin
			o0_val <= #1 1'b0;
		end
		else
		begin
			case(rom_value[9:8])
				R2O: begin
					case (rom_value[5])
					O0 : begin
						o0_val <= 1'b1;
					end
					default: begin
						if (o0_received)
						begin
							o0_val <= #1 1'b0;
						end
					end
					endcase
				end
				default: begin
					if (o0_received)
					begin
						o0_val <= #1 1'b0;
					end
				end
			endcase
		end
	end

	always @(posedge clock_signal, posedge reset_signal)
	begin
		if(reset_signal)
		begin
			_pc <= #1 8'h0;
			_r0 <= #1 8'h0;
			_r1 <= #1 8'h0;
			_r2 <= #1 8'h0;
			_r3 <= #1 8'h0;
		end
		else begin
			$display("Program Counter:%d", _pc);
			$display("Instruction:%b", rom_value);
			$display("Registers r0:%b r1:%b r2:%b r3:%b ", _r0, _r1, _r2, _r3);
				case(rom_value[9:8])
					I2R: begin
						case (rom_value[7:6])
						R0 : begin
							case (rom_value[5])
							I0 : begin
								_r0 <= #1 i0;
								$display("I2R R0 I0");
							end
							endcase
						end
						R1 : begin
							case (rom_value[5])
							I0 : begin
								_r1 <= #1 i0;
								$display("I2R R1 I0");
							end
							endcase
						end
						R2 : begin
							case (rom_value[5])
							I0 : begin
								_r2 <= #1 i0;
								$display("I2R R2 I0");
							end
							endcase
						end
						R3 : begin
							case (rom_value[5])
							I0 : begin
								_r3 <= #1 i0;
								$display("I2R R3 I0");
							end
							endcase
						end
						endcase
						_pc <= #1 _pc + 1'b1 ;
					end
					R2O: begin
						case (rom_value[7:6])
						R0 : begin
							case (rom_value[5])
							O0 : begin
								_auxo0 <= #1 _r0;
								$display("R2O R0 O0");
							end
							endcase
						end
						R1 : begin
							case (rom_value[5])
							O0 : begin
								_auxo0 <= #1 _r1;
								$display("R2O R1 O0");
							end
							endcase
						end
						R2 : begin
							case (rom_value[5])
							O0 : begin
								_auxo0 <= #1 _r2;
								$display("R2O R2 O0");
							end
							endcase
						end
						R3 : begin
							case (rom_value[5])
							O0 : begin
								_auxo0 <= #1 _r3;
								$display("R2O R3 O0");
							end
							endcase
						end
						endcase
						_pc <= #1 _pc + 1'b1 ;
					end
					INC: begin
						case (rom_value[7:6])
						R0 : begin
							_r0 <= #1 _r0 + 1'b1;
							$display("INC R0");
						end
						R1 : begin
							_r1 <= #1 _r1 + 1'b1;
							$display("INC R1");
						end
						R2 : begin
							_r2 <= #1 _r2 + 1'b1;
							$display("INC R2");
						end
						R3 : begin
							_r3 <= #1 _r3 + 1'b1;
							$display("INC R3");
						end
						endcase
						_pc <= #1 _pc + 1'b1 ;
					end
					J: begin
						_pc <= #1 rom_value[7:0];
						$display("J ", rom_value[7:0]);
					end
					default : begin
						$display("Unknown Opcode");
						_pc <= #1 _pc + 1'b1;
					end
				endcase
		end
	end
	assign rom_bus = _pc;
	assign i0_received = i0_recv;
	assign o0 = _auxo0;
	assign o0_valid = o0_val;
endmodule
