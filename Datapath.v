`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Design Name: 
// Module Name:    CPU_datapath 
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

module CPU_datapath(clk,ldPC,ldIR,ldMAR,rd_mem,wr_mem,ldtmp,ldMDRZ,ldMDRdata,wr_reg,rd_reg,ldALU,
	ldXPC,ldYPC,ldXtmp,ldYtmp,ldXreg,ldYreg,ldXmem,ldYmem,ldXtmp2,ldYtmp2,
	wr_regA,rd_regA,fsel,opd1,opd2,opd3,C,V,S,Z_det,state,next_state,opc,reset,
	rom_mem0,rom_mem1,rom_mem2,rom_mem3,rom_mem4,rom_mem5,rom_mem6,rom_mem7,rom_mem8,rom_mem9,
	rom_mem10,rom_mem11,rom_mem12,rom_mem13,rom_mem14,rom_mem15,rom_mem16,rom_mem17,rom_mem18,rom_mem19,
	r0,r1,r2,r3,r4,r5,r6,r7);	// Main datapath module to connect different datapath elements
	
	
	wire[15:0] Zbus,XYbus,PC,IRin,MAR,out_data,MDRin,MDR,tmp,reg_rdOut,Y;
	wire[2:0] XYbusInSel;
	wire ldXYPC,ldXYtmp,ldXYreg,ldXYmem,ldXYtmp2,ldMDR,ldwY0,ldwY1,ldwY2,ldwXY0,ldwXY1,ldwXY2,ldXYbus,ldY;
	input clk,reset,ldPC,ldIR,ldMAR,rd_mem,wr_mem,ldtmp,ldMDRZ,ldMDRdata,wr_reg,rd_reg,ldALU;
	input ldXPC,ldYPC,ldXtmp,ldYtmp,ldXreg,ldYreg,ldXmem,ldYmem,ldXtmp2,ldYtmp2;
	input[2:0] wr_regA,rd_regA,fsel;
	input[4:0] next_state;
	input[15:0] rom_mem0,rom_mem1,rom_mem2,rom_mem3,rom_mem4,rom_mem5,rom_mem6,rom_mem7,rom_mem8,rom_mem9;
	input[15:0] rom_mem10,rom_mem11,rom_mem12,rom_mem13,rom_mem14,rom_mem15,rom_mem16,rom_mem17,rom_mem18,rom_mem19;
	output[4:0] state;
	output[6:0] opc;
	output[2:0] opd1,opd2,opd3;
	output C,V,S,Z_det;
	output[15:0] r0,r1,r2,r3,r4,r5,r6,r7;
	Register ProgCntr(Zbus,clk,PC,ldPC,reset);
	Register InstReg(out_data,clk,{opc,opd1,opd2,opd3},ldIR,reset);
	Register MAddReg(Zbus,clk,MAR,ldMAR,reset);
	Register Temp(Zbus,clk,tmp,ldtmp,reset);
	Register MDataReg(MDRin,clk,MDR,ldMDR,reset);
	Memory mem(MAR,rd_mem,wr_mem,clk,MDR,out_data,reset,
		rom_mem0,rom_mem1,rom_mem2,rom_mem3,rom_mem4,rom_mem5,rom_mem6,rom_mem7,rom_mem8,rom_mem9,
		rom_mem10,rom_mem11,rom_mem12,rom_mem13,rom_mem14,rom_mem15,rom_mem16,rom_mem17,rom_mem18,rom_mem19);
	mux2to1 MemDataRegInMux(MDRin,ldMDRZ,ldMDR,out_data,Zbus);
	or omdr0(ldMDR,ldMDRZ,ldMDRdata);
	
	or oPC(ldXYPC,ldXPC,ldYPC);
	or otmp(ldXYtmp,ldXtmp,ldYtmp);
	or oreg(ldXYreg,ldXreg,ldYreg);
	or omem(ldXYmem,ldXmem,ldYmem);
	or otmp2(ldXYtmp2,ldXtmp2,ldYtmp2);
	or oY0(ldwY0,ldYPC,ldYtmp);
	or oY1(ldwY1,ldwY0,ldYreg);
	or oY2(ldwY2,ldwY1,ldYmem);
	or oY3(ldY,ldwY2,ldYtmp2);
	or oXY0(ldwXY0,ldXYPC,ldXYtmp);
	or oXY1(ldwXY1,ldwXY0,ldXYreg);
	or oXY2(ldwXY2,ldwXY1,ldXYmem);
	or oXY3(ldXYbus,ldwXY2,ldXYtmp2);
   encoder8to3 XSelectLine(XYbusInSel,{1'b0,1'b0,1'b0,ldXYtmp2,ldXYmem,ldXYreg,ldXYtmp,ldXYPC});

	mux5to1 XYbusInMux(XYbus,XYbusInSel,ldXYbus,PC,tmp,reg_rdOut,MDR,16'b0000000000000010);
	reg_bank regs(Zbus,wr_regA,rd_regA,wr_reg,rd_reg,reg_rdOut,clk,reset,r0,r1,r2,r3,r4,r5,r6,r7);
	ALU alu_op(XYbus,Y,Zbus,fsel,C,V,S,Z_det,ldALU);
	Register buffer0(XYbus,clk,Y,ldY,reset);
	
	Register_5bit next_move(next_state,clk,state,reset);
	
endmodule


module Memory(address,read,write,clk,in_data,out_data,reset,
	rom_mem0,rom_mem1,rom_mem2,rom_mem3,rom_mem4,rom_mem5,rom_mem6,rom_mem7,rom_mem8,rom_mem9,
	rom_mem10,rom_mem11,rom_mem12,rom_mem13,rom_mem14,rom_mem15,rom_mem16,rom_mem17,rom_mem18,rom_mem19);
					//address : 16-bit address line 
					//read : read from memory : flag
					//write : write into memory : flag
					//clk : clock (unusual memory module which reads or write @ positive edge of the clock)
					//in_data : data to write into memory
					//out_data : data to read from memory
					//rom : initial instruction set to initialize memory (used for testing)

	input[15:0]address;
	input read,write,clk,reset;
	input[15:0] in_data;
	input[15:0] rom_mem0,rom_mem1,rom_mem2,rom_mem3,rom_mem4,rom_mem5,rom_mem6,rom_mem7,rom_mem8,rom_mem9;
	input[15:0] rom_mem10,rom_mem11,rom_mem12,rom_mem13,rom_mem14,rom_mem15,rom_mem16,rom_mem17,rom_mem18,rom_mem19;
	output[15:0] out_data;
	reg[7:0] memory[65535:0];
	reg[15:0] out_data;
	always@(posedge clk or reset)
	begin
		if(reset)begin
			memory[0]=rom_mem0[15:8];
			memory[1]=rom_mem0[7:0];
			memory[2]=rom_mem1[15:8];
			memory[3]=rom_mem1[7:0];
			memory[4]=rom_mem2[15:8];
			memory[5]=rom_mem2[7:0];
			memory[6]=rom_mem3[15:8];
			memory[7]=rom_mem3[7:0];
			memory[8]=rom_mem4[15:8];
			memory[9]=rom_mem4[7:0];
			memory[10]=rom_mem5[15:8];
			memory[11]=rom_mem5[7:0];
			memory[12]=rom_mem6[15:8];
			memory[13]=rom_mem6[7:0];
			memory[14]=rom_mem7[15:8];
			memory[15]=rom_mem7[7:0];
			memory[16]=rom_mem8[15:8];
			memory[17]=rom_mem8[7:0];
			memory[18]=rom_mem9[15:8];
			memory[19]=rom_mem9[7:0];
			memory[20]=rom_mem10[15:8];
			memory[21]=rom_mem10[7:0];
			memory[22]=rom_mem11[15:8];
			memory[23]=rom_mem11[7:0];
			memory[24]=rom_mem12[15:8];
			memory[25]=rom_mem12[7:0];
			memory[26]=rom_mem13[15:8];
			memory[27]=rom_mem13[7:0];
			memory[28]=rom_mem14[15:8];
			memory[29]=rom_mem14[7:0];
			memory[30]=rom_mem15[15:8];
			memory[31]=rom_mem15[7:0];
			memory[32]=rom_mem16[15:8];
			memory[33]=rom_mem16[7:0];
			memory[34]=rom_mem17[15:8];
			memory[35]=rom_mem17[7:0];
			memory[36]=rom_mem18[15:8];
			memory[37]=rom_mem18[7:0];
			memory[38]=rom_mem19[15:8];
			memory[39]=rom_mem19[7:0];
		end
		if(read)out_data={memory[address],memory[{address[15:1],1'b1}]};
		if(write)begin
			memory[address]=in_data[15:8];
			memory[{address[15:1],1'b1}]=in_data[7:0];
		end
	end
endmodule

module encoder8to3(out,in);	//in[0]=1 -> out = 0
	input[7:0] in;
	output[2:0] out;
	reg[2:0] out;
	always@(in)begin
		casex(in)
			8'b00000001:out=3'b000;
			8'b00000010:out=3'b001;
			8'b00000100:out=3'b010;
			8'b00001000:out=3'b011;
			8'b00010000:out=3'b100;
			8'b00100000:out=3'b101;
			8'b01000000:out=3'b110;
			8'b10000000:out=3'b111;
			default:out=out;
		endcase
	end
endmodule

module mux5to1(out,sel,ld,r0,r1,r2,r3,r4);	//sel = 0 -> out = r0
	output[15:0] out;
	input[2:0] sel;
	input ld;
	input[15:0] r0,r1,r2,r3,r4;
	reg[15:0] out;
	always@(sel or ld or r0 or r1 or r2 or r3 or r4)
	begin
		if(ld)begin
			casex(sel)
				3'b000:out=r0;
				3'b001:out=r1;
				3'b010:out=r2;
				3'b011:out=r3;
				3'b100:out=r4;
				3'b101:out=out;
				3'b110:out=out;
				3'b111:out=out;
			endcase
		end
		else out=out;
	end
endmodule


module mux2to1(out,sel,ld,r0,r1);	//sel = 0 -> out = r0
	output[15:0] out;
	input sel;
	input ld;
	input[15:0] r0,r1;
	reg[15:0] out;
	always@(sel or ld or r0 or r1)
	begin
		if(ld)begin
			if(sel)out=r1;
			else out=r0;
		end
		else out=out;
	end
endmodule

module Register(in,clk,out,ld,rst);	//16-bit Register @ positive edge of clock
	input[15:0] in;
	input clk,ld,rst;
	output[15:0] out;
	reg[15:0] out;
	always@(posedge clk or posedge rst)
	begin
		if(rst)out=16'b0;
		else begin
			if(ld)out=in;
			else out=out;
		end
	end
endmodule

module Register_5bit(in,clk,out,rst);	//5-bit Register @ negative edge of clock (for state transition)
	input[4:0] in;
	input clk,rst;
	output[4:0] out;
	reg[4:0] out;
	always@(negedge clk or posedge rst)
	begin
		if(rst)out=5'b0;
		else out=in;
	end
endmodule

module decoder3to8(in,out,ld);	// in = 0 -> out[0] = 1;
	input[2:0] in;
	input ld;
	output[7:0] out;
	reg[7:0] out;
	always@(in or ld)
	begin
		if(ld)
		begin
			casex(in)
				3'b000:out=8'b00000001;
				3'b001:out=8'b00000010;
				3'b010:out=8'b00000100;
				3'b011:out=8'b00001000;
				3'b100:out=8'b00010000;
				3'b101:out=8'b00100000;
				3'b110:out=8'b01000000;
				3'b111:out=8'b10000000;
			endcase
		end
		else out=8'b00000000;
	end
endmodule


module mux8to1(out,sel,ld,r0,r1,r2,r3,r4,r5,r6,r7);	//sel = 0 -> out = r0
	output[15:0] out;
	input[2:0] sel;
	input ld;
	input[15:0] r0,r1,r2,r3,r4,r5,r6,r7;
	reg[15:0] out;
	always@(sel or ld or r0 or r1 or r2 or r3 or r4 or r5 or r6 or r7)
	begin
		if(ld)begin
			casex(sel)
				3'b000:out=r0;
				3'b001:out=r1;
				3'b010:out=r2;
				3'b011:out=r3;
				3'b100:out=r4;
				3'b101:out=r5;
				3'b110:out=r6;
				3'b111:out=r7;
			endcase
		end
		else out=out;
	end
endmodule


module mux8to1_1bit(out,sel,ld,r0,r1,r2,r3,r4,r5,r6,r7);	// sel = 0 -> out = r0
	output out;
	input[2:0] sel;
	input ld;
	input r0,r1,r2,r3,r4,r5,r6,r7;
	reg out;
	always@(sel or ld or r0 or r1 or r2 or r3 or r4 or r5 or r6 or r7)
	begin
		if(ld)begin
			casex(sel)
				3'b000:out=r0;
				3'b001:out=r1;
				3'b010:out=r2;
				3'b011:out=r3;
				3'b100:out=r4;
				3'b101:out=r5;
				3'b110:out=r6;
				3'b111:out=r7;
			endcase
		end
		else out=out;
	end
endmodule


module reg_bank(data,wp,pa,wrr,rdr,p,clk,rst,r0,r1,r2,r3,r4,r5,r6,r7);	//Register Bank
																								// wp = write port address
																								// pa = read port address
																								// wrr = write in register flag
																								// rdr = read from register flag
																								// p = output data
																								// data = input data
																								// clk = clock
																								// rst = reset flag
																								// ri = Register i
	input[15:0] data;
	input[2:0] wp,pa;
	input wrr,rdr;
	input clk,rst;
	output[15:0] p;
	
	wire[7:0] wr;
	output[15:0] r0,r1,r2,r3,r4,r5,r6,r7;
	
	decoder3to8 write(wp,wr,wrr);
	
	Register load0(data,clk,r0,wr[0],rst);
	Register load1(data,clk,r1,wr[1],rst);
	Register load2(data,clk,r2,wr[2],rst);
	Register load3(data,clk,r3,wr[3],rst);
	Register load4(data,clk,r4,wr[4],rst);
	Register load5(data,clk,r5,wr[5],rst);
	Register load6(data,clk,r6,wr[6],rst);
	Register load7(data,clk,r7,wr[7],rst);
	
	mux8to1 read(p,pa,rdr,r0,r1,r2,r3,r4,r5,r6,r7);

endmodule


module ALU(X,Y,Z,fsel,C,V,S,Z_det,alu_ld);	//ALU : X = input1, Y = input2, fsel = function select, Z = fsel(X,Y), alu_ld = load ALU
															//fsel :-
															// 0 -> Z = X + Y (ADD)
															// 1 -> Z = X - Y (SUB)
															// 2 -> Z = X & Y (AND)
															// 3 -> Z = X | Y (OR)
															// 4 -> Z = -X (2's complement of X)
															// 5 -> Z = X - Y (MINUS same as SUB)
															// 6 -> Z = X (TRANSFER X)
															// 7 -> Z = Y (TRANSFER Y)
															//flags :-
															// C = carry
															// V = overflow
															// S = sign
															// Z_det = zero
	input[15:0] X,Y;
	input[2:0]fsel;
	input alu_ld;
	output[15:0] Z;
	output C,V,S,Z_det;
	wire[15:0] w0,w1,w2,w3,w4;
	wire[14:0] w5;
	wire wca,wcs,wca0,wcs0,wva,wvs;

	add add_0(X,Y,w0,wca,wca0);
	sub sub_0(X,Y,w1,wcs,wcs0);
	and_op and_0(X,Y,w2);
	or_op or_0(X,Y,w3);
	cmp complement0(X,w4);
	
	genvar i;
	or o0(w5[0],Z[0],Z[1]); 
	generate
		for(i=2;i<16;i=i+1)begin:asj0
			or o1(w5[i-1],Z[i],w5[i-2]);
		end
	endgenerate
	not n0(Z_det,w5[14]);
	xor x1(wva,wca,wca0); 
	xor x2(wvs,wcs,wcs0); 
	and a0(S,Z[15],1'b1);
	
	mux8to1_1bit carrysel(C,fsel,alu_ld,wca0,wcs0,1'b0,1'b0,1'b0,wcs0,1'b0,1'b0);
	mux8to1_1bit ovflosel(C,fsel,alu_ld,wva,wvs,1'b0,1'b0,1'b0,wvs,1'b0,1'b0);
	mux8to1 feedZ(Z,fsel,alu_ld,w0,w1,w2,w3,w4,w1,X,Y);
	

endmodule


module add( a ,b ,sum ,carryls ,carryl );	//carryls = last second carry, carryl = last carry, sum = a + b

	input [15:0] a ;
	input [15:0] b ; 

	output [15:0] sum;
	output carryl,carryls ;

	wire [13:0]s;

	full_adder u100 (a[0],b[0],1'b0,sum[0],s[0]);
	full_adder u101 (a[1],b[1],s[0],sum[1],s[1]);
	full_adder u102 (a[2],b[2],s[1],sum[2],s[2]);
	full_adder u103 (a[3],b[3],s[2],sum[3],s[3]);
	full_adder u104 (a[4],b[4],s[3],sum[4],s[4]);
	full_adder u105 (a[5],b[5],s[4],sum[5],s[5]);
	full_adder u106 (a[6],b[6],s[5],sum[6],s[6]);
	full_adder u107 (a[7],b[7],s[6],sum[7],s[7]);
	full_adder u108 (a[8],b[8],s[7],sum[8],s[8]);
	full_adder u109 (a[9],b[9],s[8],sum[9],s[9]);
	full_adder u110 (a[10],b[10],s[9],sum[10],s[10]);
	full_adder u111 (a[11],b[11],s[10],sum[11],s[11]);
	full_adder u112 (a[12],b[12],s[11],sum[12],s[12]);
	full_adder u113 (a[13],b[13],s[12],sum[13],s[13]);
	full_adder u114 (a[14],b[14],s[13],sum[14],carryls);
	full_adder u115 (a[15],b[15],carryls,sum[15],carryl);

endmodule


module sub( a ,b ,diff ,borrowls,borrowl);	//borrowls = last second borrow, borrowl = last borrow, diff = a - b

	input [15:0] a ;
	input [15:0] b ; 

	output [15:0] diff ;
	output borrowl,borrowls ;

	wire [15:0] l;
	wire [13:0] s;
	
	assign l =( 16'b1111111111111111 ^ b);

	full_adder u00 (a[0],l[0],1'b1,diff[0],s[0]);
	full_adder u01 (a[1],l[1],s[0],diff[1],s[1]);
	full_adder u02 (a[2],l[2],s[1],diff[2],s[2]);
	full_adder u03 (a[3],l[3],s[2],diff[3],s[3]);
	full_adder u04 (a[4],l[4],s[3],diff[4],s[4]);
	full_adder u05 (a[5],l[5],s[4],diff[5],s[5]);
	full_adder u06 (a[6],l[6],s[5],diff[6],s[6]);
	full_adder u07 (a[7],l[7],s[6],diff[7],s[7]);
	full_adder u08 (a[8],l[8],s[7],diff[8],s[8]);
	full_adder u09 (a[9],l[9],s[8],diff[9],s[9]);
	full_adder u10 (a[10],l[10],s[9],diff[10],s[10]);
	full_adder u11 (a[11],l[11],s[10],diff[11],s[11]);
	full_adder u12 (a[12],l[12],s[11],diff[12],s[12]);
	full_adder u13 (a[13],l[13],s[12],diff[13],s[13]);
	full_adder u14 (a[14],l[14],s[13],diff[14],borrowls);
	full_adder u15 (a[15],l[15],borrowls,diff[15],borrowl);

endmodule


module full_adder ( a ,b ,c ,sum ,carry );	//(carry,sum) = a+b+c

	input a ; 
	input b ;
	input c ;
   
	output sum ;
	output carry ;

	assign sum = ((a ^ b) ^ c);  
	assign carry = (((a&b) | (b&c)) | (c&a));

endmodule  


module and_op(X,Y,out);
	input[15:0] X,Y;
	output[15:0] out;
	and and_fn0(out[0],X[0],Y[0]);
	and and_fn1(out[1],X[1],Y[1]);
	and and_fn2(out[2],X[2],Y[2]);
	and and_fn3(out[3],X[3],Y[3]);
	and and_fn4(out[4],X[4],Y[4]);
	and and_fn5(out[5],X[5],Y[5]);
	and and_fn6(out[6],X[6],Y[6]);
	and and_fn7(out[7],X[7],Y[7]);
	and and_fn8(out[8],X[8],Y[8]);
	and and_fn9(out[9],X[9],Y[9]);
	and and_fn10(out[10],X[10],Y[10]);
	and and_fn11(out[11],X[11],Y[11]);
	and and_fn12(out[12],X[12],Y[12]);
	and and_fn13(out[13],X[13],Y[13]);
	and and_fn14(out[14],X[14],Y[14]);
	and and_fn15(out[15],X[15],Y[15]);
endmodule

module or_op(X,Y,out);	//out = X | Y
	input[15:0] X,Y;
	output[15:0] out;
	or or_fn0(out[0],X[0],Y[0]);
	or or_fn1(out[1],X[1],Y[1]);
	or or_fn2(out[2],X[2],Y[2]);
	or or_fn3(out[3],X[3],Y[3]);
	or or_fn4(out[4],X[4],Y[4]);
	or or_fn5(out[5],X[5],Y[5]);
	or or_fn6(out[6],X[6],Y[6]);
	or or_fn7(out[7],X[7],Y[7]);
	or or_fn8(out[8],X[8],Y[8]);
	or or_fn9(out[9],X[9],Y[9]);
	or or_fn10(out[10],X[10],Y[10]);
	or or_fn11(out[11],X[11],Y[11]);
	or or_fn12(out[12],X[12],Y[12]);
	or or_fn13(out[13],X[13],Y[13]);
	or or_fn14(out[14],X[14],Y[14]);
	or or_fn15(out[15],X[15],Y[15]);
endmodule

module cmp(x,y);	//y = 2's complement of x
	input[15:0] x;
	output[15:0] y;
	wire w0,w1;
	sub twoscmp(16'b0000000000000000,x,y,w0,w1);
endmodule

