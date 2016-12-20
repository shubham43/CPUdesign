`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Design Name: 
// Module Name:    CPU_main 
// Project Name: 
// Target Devices: 
// Tool versions: 
// Description: 
//
// Dependencies: 
//
// Revision: 
// Revision 0.01 - File Created
// Additional Comments: 
//
//////////////////////////////////////////////////////////////////////////////////
module CPU_main(
	clk,reset,
	rom_mem0,rom_mem1,rom_mem2,rom_mem3,rom_mem4,rom_mem5,rom_mem6,rom_mem7,rom_mem8,rom_mem9,
	rom_mem10,rom_mem11,rom_mem12,rom_mem13,rom_mem14,rom_mem15,rom_mem16,rom_mem17,rom_mem18,rom_mem19,
	r0,r1,r2,r3,r4,r5,r6,r7);
	 
	input clk,reset;
	input[15:0] rom_mem0,rom_mem1,rom_mem2,rom_mem3,rom_mem4,rom_mem5,rom_mem6,rom_mem7,rom_mem8,rom_mem9;
	input[15:0] rom_mem10,rom_mem11,rom_mem12,rom_mem13,rom_mem14,rom_mem15,rom_mem16,rom_mem17,rom_mem18,rom_mem19;
		
	wire ldPC,ldIR,ldMAR,rd_mem,wr_mem,ldtmp,ldMDRZ,ldMDRdata,wr_reg,rd_reg,ldALU;
	wire ldXPC,ldYPC,ldXtmp,ldYtmp,ldXreg,ldYreg,ldXmem,ldYmem,ldXtmp2,ldYtmp2;
	wire[2:0] wr_regA,rd_regA,fsel,opd1,opd2,opd3;
	wire C,V,S,Z_det;
	wire[4:0] state,next_state;
	wire[6:0] opc;
	
	output[15:0] r0,r1,r2,r3,r4,r5,r6,r7;

	CPU_datapath dp(clk,ldPC,ldIR,ldMAR,rd_mem,wr_mem,ldtmp,ldMDRZ,ldMDRdata,wr_reg,rd_reg,ldALU,
		ldXPC,ldYPC,ldXtmp,ldYtmp,ldXreg,ldYreg,ldXmem,ldYmem,ldXtmp2,ldYtmp2,
		wr_regA,rd_regA,fsel,opd1,opd2,opd3,C,V,S,Z_det,state,next_state,opc,reset,
		rom_mem0,rom_mem1,rom_mem2,rom_mem3,rom_mem4,rom_mem5,rom_mem6,rom_mem7,rom_mem8,rom_mem9,
		rom_mem10,rom_mem11,rom_mem12,rom_mem13,rom_mem14,rom_mem15,rom_mem16,rom_mem17,rom_mem18,rom_mem19,
		r0,r1,r2,r3,r4,r5,r6,r7);

	
	CPU_controller ctrl(C,V,S,Z_det,clk,reset,
		opc,opd1,opd2,opd3,
		ldPC,ldIR,ldMAR,rd_mem,wr_mem,ldtmp,ldMDRZ,ldMDRdata,wr_reg,rd_reg,ldALU,
		ldXPC,ldYPC,ldXtmp,ldYtmp,ldXreg,ldYreg,ldXmem,ldYmem,ldXtmp2,ldYtmp2,
		wr_regA,rd_regA,fsel,
		state,next_state);

endmodule

