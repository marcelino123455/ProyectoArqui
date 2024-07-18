`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 06/27/2024 08:41:03 AM
// Design Name: 
// Module Name: testbench
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

module testbench();
	reg clk;
	reg reset;
	wire [31:0] WriteData;
	wire [31:0] ALUEResult;
	wire MemWrite;
	wire [31:0] PC;
	wire [31:0] Instr;
	top dut(
		.clk(clk),
		.reset(reset),
		.WriteData(WriteData),
		.ALUResult(ALUEResult),
		.MemWrite(MemWrite),
		.PC(PC),
		.Instr(Instr)
	);
	initial begin
		reset <= 1;
		#(22)
			;
		reset <= 0;
	end
	always begin
		clk <= 1;
		#(5)
			;
		clk <= 0;
		#(5)
			;
	end
	always @(negedge clk)
		if (MemWrite)
			if ((ALUEResult === 100) & (ALUEResult === 7)) begin
				$display("Simulation succeeded");
				$stop;
			end
			else if (ALUEResult !== 96) begin
				$display("Simulation failed");
				$stop;
			end
endmodule

