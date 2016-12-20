`timescale 1ns / 1ps

////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer:
//
// Create Date:   12:08:34 10/23/2016
// Design Name:   CPU_controller
// Module Name:   D:/CPU_ASJ/CPU_2110/CPU_controller_TB.v
// Project Name:  CPU_2110
// Target Device:  
// Tool versions:  
// Description: 
//
// Verilog Test Fixture created by ISE for module: CPU_controller
//
// Dependencies:
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
////////////////////////////////////////////////////////////////////////////////

module CPU_controller_TB;

	// Inputs
	reg C;
	reg V;
	reg S;
	reg Z_det;
	reg clk;
	reg reset;
	reg [6:0] opc;
	reg [2:0] opd1;
	reg [2:0] opd2;
	reg [2:0] opd3;
	reg [4:0] state;

	// Outputs
	wire ldPC;
	wire ldIR;
	wire ldMAR;
	wire rd_mem;
	wire wr_mem;
	wire ldtmp;
	wire ldMDRZ;
	wire ldMDRdata;
	wire wr_reg;
	wire rd_reg;
	wire ldALU;
	wire ldXPC;
	wire ldYPC;
	wire ldXtmp;
	wire ldYtmp;
	wire ldXreg;
	wire ldYreg;
	wire ldXmem;
	wire ldYmem;
	wire ldXtmp2;
	wire ldYtmp2;
	wire [2:0] wr_regA;
	wire [2:0] rd_regA;
	wire [2:0] fsel;
	wire [4:0] next_state;

	// Instantiate the Unit Under Test (UUT)
	CPU_controller uut (
		.C(C), 
		.V(V), 
		.S(S), 
		.Z_det(Z_det), 
		.clk(clk), 
		.reset(reset), 
		.opc(opc), 
		.opd1(opd1), 
		.opd2(opd2), 
		.opd3(opd3), 
		.ldPC(ldPC), 
		.ldIR(ldIR), 
		.ldMAR(ldMAR), 
		.rd_mem(rd_mem), 
		.wr_mem(wr_mem), 
		.ldtmp(ldtmp), 
		.ldMDRZ(ldMDRZ), 
		.ldMDRdata(ldMDRdata), 
		.wr_reg(wr_reg), 
		.rd_reg(rd_reg), 
		.ldALU(ldALU), 
		.ldXPC(ldXPC), 
		.ldYPC(ldYPC), 
		.ldXtmp(ldXtmp), 
		.ldYtmp(ldYtmp), 
		.ldXreg(ldXreg), 
		.ldYreg(ldYreg), 
		.ldXmem(ldXmem), 
		.ldYmem(ldYmem), 
		.ldXtmp2(ldXtmp2), 
		.ldYtmp2(ldYtmp2), 
		.wr_regA(wr_regA), 
		.rd_regA(rd_regA), 
		.fsel(fsel), 
		.state(state), 
		.next_state(next_state)
	);

	initial begin
		// Initialize Inputs
		C = 0;
		V = 0;
		S = 0;
		Z_det = 0;
		clk = 0;
		reset = 1;
		opc = 0;
		opd1 = 0;
		opd2 = 0;
		opd3 = 0;
		state = 0;

		// Wait 100 ns for global reset to finish
		#100;
      reset=0;
		// Add stimulus here
		
		


/*		rom_mem1 = 16'b0000000000011001;
		rom_mem2 = 16'b0110000001000000;
		rom_mem3 = 16'b0000000011010101;
		rom_mem4 = 16'b0111010000001000;
		rom_mem5 = 16'b1111111111111111;
		rom_mem6 = 16'b1111111111111111;
		rom_mem7 = 16'b1111111111111111;
		rom_mem8 = 16'b1111111111111111;
		rom_mem9 = 16'b1111111111111111;
		rom_mem10 = 16'b1111111111111111;
		rom_mem11 = 16'b1111111111111111;
		rom_mem12 = 16'b1111111111111111;
		rom_mem13 = 16'b1111111111111111;
		rom_mem14 = 16'b1111111111111111;
		rom_mem15 = 16'b1111111111111111;
		rom_mem16 = 16'b1111111111111111;
		rom_mem17 = 16'b1111111111111111;
		rom_mem18 = 16'b1111111111111111;
		rom_mem19 = 16'b1111111111111111;
		
*/

	end
	
	always #1 clk=~clk;
	always@(posedge clk)state=next_state;
	always@state begin
		if(state==4)begin
			#2;
			opc= 7'b0110000;
			opd1=3'b000;
			opd2=3'b000;
			opd3=3'b000;
		end
	end

endmodule

