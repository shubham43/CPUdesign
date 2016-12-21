`timescale 1ns / 1ps

////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer:
//
// Design Name:   CPU_main
// Module Name:   D:/CPU_ASJ/CPU_2110/CPU_TB.v
// Project Name:  CPU_2110
// Target Device:  
// Tool versions:  
// Description: 
//
// Verilog Test Fixture created by ISE for module: CPU_main
//
// Dependencies:
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
////////////////////////////////////////////////////////////////////////////////

module CPU_TB;

	// Inputs
	reg clk;
	reg reset;
	reg [15:0] rom_mem0;
	reg [15:0] rom_mem1;
	reg [15:0] rom_mem2;
	reg [15:0] rom_mem3;
	reg [15:0] rom_mem4;
	reg [15:0] rom_mem5;
	reg [15:0] rom_mem6;
	reg [15:0] rom_mem7;
	reg [15:0] rom_mem8;
	reg [15:0] rom_mem9;
	reg [15:0] rom_mem10;
	reg [15:0] rom_mem11;
	reg [15:0] rom_mem12;
	reg [15:0] rom_mem13;
	reg [15:0] rom_mem14;
	reg [15:0] rom_mem15;
	reg [15:0] rom_mem16;
	reg [15:0] rom_mem17;
	reg [15:0] rom_mem18;
	reg [15:0] rom_mem19;

	// Outputs
	wire [15:0] r0;
	wire [15:0] r1;
	wire [15:0] r2;
	wire [15:0] r3;
	wire [15:0] r4;
	wire [15:0] r5;
	wire [15:0] r6;
	wire [15:0] r7;

	// Instantiate the Unit Under Test (UUT)
	CPU_main uut (
		.clk(clk), 
		.reset(reset), 
		.rom_mem0(rom_mem0), 
		.rom_mem1(rom_mem1), 
		.rom_mem2(rom_mem2), 
		.rom_mem3(rom_mem3), 
		.rom_mem4(rom_mem4), 
		.rom_mem5(rom_mem5), 
		.rom_mem6(rom_mem6), 
		.rom_mem7(rom_mem7), 
		.rom_mem8(rom_mem8), 
		.rom_mem9(rom_mem9), 
		.rom_mem10(rom_mem10), 
		.rom_mem11(rom_mem11), 
		.rom_mem12(rom_mem12), 
		.rom_mem13(rom_mem13), 
		.rom_mem14(rom_mem14), 
		.rom_mem15(rom_mem15), 
		.rom_mem16(rom_mem16), 
		.rom_mem17(rom_mem17), 
		.rom_mem18(rom_mem18), 
		.rom_mem19(rom_mem19), 
		.r0(r0), 
		.r1(r1), 
		.r2(r2), 
		.r3(r3), 
		.r4(r4), 
		.r5(r5), 
		.r6(r6), 
		.r7(r7)
	);


	initial begin
		// Initialize Inputs
		clk = 0;
		reset = 1;
		rom_mem0 = 16'b0110000000000000;
		rom_mem1 = 16'b0000000000101100;
		rom_mem2 = 16'b0110000001000000;
		rom_mem3 = 16'b0000000011010101;
		rom_mem4 = 16'b0111010000001000;
		rom_mem5 = 16'b0000000000001010;
		rom_mem6 = 16'b0110000011000000;
		rom_mem7 = 16'b0000000000001010;
		rom_mem8 = 16'b0000000011000000;
		rom_mem9 = 16'b0000000000010100;
		rom_mem10 = 16'b0001001011001000;
		rom_mem11 = 16'b0110000100000000;
		rom_mem12 = 16'b0000000000000000;
		rom_mem13 = 16'b0010010011000100;
		rom_mem14 = 16'b0000000000001010;
		rom_mem15 = 16'b0110000101000000;
		rom_mem16 = 16'b0000000000001010;
		rom_mem17 = 16'b0010011011101000;
		rom_mem18 = 16'b1111111111111111;
		rom_mem19 = 16'b1111111111111111;
		
		// Wait 100 ns for global reset to finish
		#100;
		reset=0;
		
		// Add stimulus here

	end


	always begin
		#1;
		clk=~clk;
	end
      
endmodule

