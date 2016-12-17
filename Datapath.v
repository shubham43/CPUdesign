`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    15:52:51 09/29/2016 
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
module CPU_datapath();


endmodule



module Register(in,clk,out,ld);
	input[15:0] in;
	input clk,ld;
	output[15:0] out;
	reg[15:0] out;
	always@(posedge clk)
	begin
		if(ld)out=in;
		else out=out;
	end

endmodule

/* structural
module decoder3to8(in,out);
	input[2:0] in;
	output[7:0] out;
	wire[7:0] w1,w2;
	
	or(w1[0],in[0],in[1]);
	or(w2[0],w1[0],in[2]);
	not(out[0],w2[0]);
	
	nor(w1[1],in[1],in[2]);
	and(out[1],w1[1],in[0]);
	
	nor(w1[2],in[2],in[0]);
	and(out[2],w1[2],in[1]);
	
	and(w1[3],in[0],in[1]);
	not(w2[3],in[2]);
	and(out[3],w1[3],w2[3]);

	nor(w1[4],in[1],in[0]);
	and(out[4],w1[4],in[2]);
	
	and(w1[5],in[2],in[0]);
	not(w2[5],in[1]);
	and(out[5],w2[5],w1[5]);
	
	and(w1[6],in[2],in[1]);
	not(w2[6],in[0]);
	and(out[6],w2[6],w1[6]);
	
	and(w1[7],in[0],in[1]);
	and(out[7],w1[7],in[2]);

endmodule
*/

module decoder3to8(in,out,ld);
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


module mux8to1(out,sel,ld,r0,r1,r2,r3,r4,r5,r6,r7);
	output[15:0] out;
	input[2:0] sel;
	input ld;
	input[15:0] r0,r1,r2,r3,r4,r5,r6,r7;
	reg[15:0] out;
	always@(sel or ld)
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


module reg_bank(data,wp,pa,wrr,rdr,p,clk);
	input[15:0] data;
	input[2:0] wp,pa;
	input wrr,rdr;
	input clk;
	output[15:0] p;
	
	wire[7:0] wr;
	wire[15:0] r0,r1,r2,r3,r4,r5,r6,r7;
	
	decoder3to8 write(wp,wr,wrr);
	
	Register load0(data,clk,r0,wr[0]);
	Register load1(data,clk,r1,wr[1]);
	Register load2(data,clk,r2,wr[2]);
	Register load3(data,clk,r3,wr[3]);
	Register load4(data,clk,r4,wr[4]);
	Register load5(data,clk,r5,wr[5]);
	Register load6(data,clk,r6,wr[6]);
	Register load7(data,clk,r7,wr[7]);
	
	mux8to1 read(p,pa,rdr,r0,r1,r2,r3,r4,r5,r6,r7);

endmodule


module ALU(X,Y,Z,fsel,C,V,S,Z_det);
	input[15:0] X,Y;
	input[2:0]fsel;
	output[15:0] Z;
	output C,V,S,Z_det;
	wire C0;
	wire[7:0] FS;
	wire[15:0] w0,w1,w2,w3,w4,w5;

	add_sub as0(X,Y,w0,fsel[0],C,C0);
	and_or ao0(X,Y,fsel[0],w1);
	cmp complement0(X,w3);
	trans transfer0(X,w4);
	
	genvar i;
	or o0(w5[0],X[0],X[1]); 
	generate
		for(i=2;i<16;i=i+1)begin:asj0
			or o1(w5[i-1],X[i],w5[i-2]);
		end
	endgenerate
	not n0(Z_det,w5[14]);
	xor x1(V,C,C0); 
	and a0(S,Z[15],1'b1);
	
	mux8to1 feedZ(Z,fsel,1'b1,w0,w0,w1,w1,w3,Z,w4,w4);
	

endmodule


module add_sub(X,Y,Z,sel,C1,C0);
	input[15:0] X,Y;
	input sel;
	output[15:0]Z;
	output C1,C0;
	wire[15:0] cmp;
	wire[13:0] c;

	genvar i;
	generate
		for(i=0;i<16;i=i+1)begin:asj1
			xor x0(cmp[i],Y[i],sel);
		end
	endgenerate
	
	full_adder fa0(Z[0],X[0],cmp[0],sel,c[0]);
	
	generate
		for(i=1;i<14;i=i+1)begin:asj2
			full_adder fa1(Z[i],X[i],cmp[i],c[i-1],c[i]);
		end
	endgenerate

	full_adder fa2(Z[14],X[14],cmp[14],c[13],C0);
	full_adder fa3(Z[15],X[15],cmp[15],C0,C1);
	
endmodule


module full_adder(out,in1,in2,cin,cout);
	input in1,in2,cin;
	output out,cout;
	wire w0,w1,w2,w3,w4;
	xor x1(w0,in1,in2);
	xor x2(out,w0,cin);
	and a0(w1,in1,in2);
	and a1(w2,in2,cin);
	and a2(w3,cin,in1); 
	or o0(w4,w1,w2);
	or o1(cout,w4,w3);
endmodule


module and_or(X,Y,sel,out);
	input[15:0] X,Y;
	input sel; 
	output[15:0] out;
	wire[15:0] wa,wo;
	not n1(w0,sel);
	genvar i;
	generate
		for(i=0;i<16;i=i+1)begin:asj3
			 or o0(wa[i],X[i],Y[i]);
			 or o1(wa[i],X[i],Y[i]);
			 and a1(wo[i],X[i],Y[i]);
			 and a2(w1,wa[i],w0);
			 and a3(w2,wo[i],sel);
			 or o2(out[i],w1,w2);
		end
	endgenerate
endmodule


module cmp(x,y);
	input[15:0] x;
	output[15:0] y;
	wire w0,w1;
	add_sub twoscmp(16'b0000000000000000,x,y,1'b1,w0,w1);
endmodule


module trans(x,y);
	input[15:0] x;
	output[15:0] y;
	wire w0,w1;
	add_sub addZero(16'b0000000000000000,x,y,1'b0,w0,w1);
endmodule
