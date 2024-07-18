`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 15.07.2024 04:28:36
// Design Name: 
// Module Name: armVector
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

//---
module top (
	clk,
	reset,
	PC, //+
	Instr, //+
	WriteData,
	ALUResult, //DataAdr
	MemWrite,
	VectorResult
);
	input wire clk;
	input wire reset;
	output wire [31:0] WriteData;
	output wire [31:0] ALUResult; //mod
	output wire MemWrite;
	output wire [31:0] PC; //+
	output wire [31:0] Instr; //+
    output wire [150:0] VectorResult;
	wire [31:0] ReadData;
	arm arm(
		.clk(clk),
		.reset(reset),
		.PC(PC),
		.Instr(Instr),
		.MemWrite(MemWrite),
		.ALUResult(ALUResult),
		.WriteData(WriteData),
		.ReadData(ReadData),
		.VectorResult(VectorResult)
	);
	imem imem(
		.a(PC),
		.rd(Instr)
	);
	dmem dmem(
		.clk(clk),
		.we(MemWrite),
		.a(ALUResult),
		.wd(WriteData),
		.rd(ReadData)
	);
endmodule
module dmem (
	clk,
	we,
	a,
	wd,
	rd
);
	input wire clk;
	input wire we;
	input wire [31:0] a;
	input wire [31:0] wd;
	output wire [31:0] rd;
	reg [31:0] RAM [63:0];
	assign rd = RAM[a[31:2]];
	always @(posedge clk)
		if (we)
			RAM[a[31:2]] <= wd;
endmodule
module imem (
	a,
	rd
);
	input wire [31:0] a;
	output wire [31:0] rd;
	reg [31:0] RAM [63:0]; //wire
	initial $readmemh("memfile5.dat", RAM);
	assign rd = RAM[a[31:2]];
endmodule

module arm (
	clk,
	reset,
	PC, //out
	Instr,
	MemWrite,
	ALUResult, //out
	WriteData, //out
	ReadData,
	VectorResult
	);
	input wire clk;
	input wire reset;
	output wire [31:0] PC;
	input wire [31:0] Instr;
	output wire MemWrite;
	output wire [31:0] ALUResult;
	output wire [31:0] WriteData;
	input wire [31:0] ReadData;
    output wire [150:0] VectorResult;
    wire PCSrc;
    wire [1:0] RegESrc;
    wire RegVSrc;
    wire MemtoReg;
    wire [4:0] ExecuteResult;
    wire [1:0] OpVResult;
    wire MOVControl;
    wire [2:0] ALUControl;
    wire [1:0] FPControl;
    wire RegVWrite;
    wire TypeSourceB;
    wire [1:0] ImmSrc;
    wire Floating;
    wire RegEWrite;
    wire [19:0] VFlags;
	

    //datapath
    datapath datapath(
        .clk(clk),
        .reset(reset),
        .PCSrc(PCSrc),
        .RegESrc(RegESrc),
        .RegVSrc(RegVSrc),
        .MemtoReg(MemtoReg),
        .ExecuteResult(ExecuteResult),
        .MemWrite(MemWrite),
        .OpVResult(OpVResult),
        .MOVControl(MOVControl),
        .ALUControl(ALUControl),
        .FPControl(FPControl),
        .RegVWrite(RegVWrite),
        .TypeSourceB(TypeSourceB),
        .ImmSrc(ImmSrc),
        .Floating(Floating),
        .RegEWrite(RegEWrite),
        .Instr(Instr),
        .ReadData(ReadData),
        .PC(PC),
        .WriteData(WriteData),
        .ALUEResult(ALUResult),
        .VFlags(VFlags),
		.VectorResult(VectorResult)
    );

    //controller
    controller controller(
        .clk(clk),
        .reset(reset),
        .Instr(Instr[31:12]),
        .VFlags(VFlags),
        .PCSrc(PCSrc),
        .RegESrc(RegESrc),
        .RegVSrc(RegVSrc),
        .MemtoReg(MemtoReg),
        .ExecuteResult(ExecuteResult),
        .MemWrite(MemWrite),
        .OpVResult(OpVResult),
        .MOVControl(MOVControl),
        .ALUControl(ALUControl),
        .FPControl(FPControl),
        .RegVWrite(RegVWrite),
        .TypeSourceB(TypeSourceB),
        .ImmSrc(ImmSrc),
        .Floating(Floating),
        .RegEWrite(RegEWrite)
    );

endmodule


//---

module controller (
    clk,
    reset,
    Instr,
    VFlags,
    PCSrc,
    RegESrc,
    RegVSrc,
    MemtoReg,
    ExecuteResult,
    MemWrite,
    OpVResult,
    MOVControl,
    ALUControl,
    FPControl,
    RegVWrite,
    TypeSourceB,
    ImmSrc,
    Floating,
    RegEWrite
	);

    input wire clk;
    input wire reset;
    input wire [31:12] Instr;
    input wire [19:0] VFlags;
    output wire PCSrc;
    output wire [1:0] RegESrc;
    output wire [1:0] RegVSrc;
    output wire MemtoReg;
    output wire [4:0] ExecuteResult;
    output wire MemWrite;
    output wire [1:0] OpVResult;
    output wire MOVControl;
    output wire [2:0] ALUControl;
    output wire [1:0] FPControl;
    output wire RegVWrite;
    output wire TypeSourceB;
    output wire [1:0] ImmSrc;
    output wire Floating;
    output wire RegEWrite;
    wire PCS;
    wire MemW;
    wire RegEW;
    wire RegVW;
    wire [1:0] FlagW;

    condlogic cond(
		.clk(clk),
		.reset(reset),
        .PCS(PCS),
        .MemW(MemW),
        .RegEW(RegEW),
        .RegVW(RegVW),
        .FlagW(FlagW),
        .VFlags(VFlags),
        .Cond(Instr[31:28]),
        .PCSrc(PCSrc),
        .MemWrite(MemWrite),
        .RegEWrite(RegEWrite),
        .RegVWrite(RegVWrite),
        .ExecuteResult(ExecuteResult)
    );

    decode decode(
        .Op(Instr[27:25]),
        .Funct(Instr[24:20]),
        .Rd(Instr[15:12]),
        .FlagW(FlagW),
        .PCS(PCS),
        .MemW(MemW),
        .MemtoReg(MemtoReg),
        .ImmSrc(ImmSrc),
        .ALUControl(ALUControl),
        .TypeSrcB(TypeSourceB),
        .RegESrc(RegESrc),
        .RegVSrc(RegVSrc),
        .OpVResult(OpVResult),
        .FPControl(FPControl),
        .MOVcontrol(MOVControl),
        .RegEW(RegEW),
        .Floating(Floating),
        .RegvVW(RegVW)
    );
endmodule

module decode(
	input wire [2:0] Op,
	input wire [4:0] Funct,
	input wire [3:0] Rd,
	//Outputs
	output wire [1:0] FlagW,
	output wire PCS,//Ya no PCS 
	output wire MemW,
	output wire MemtoReg,
	//output wire ALUSrc, -->No hay ALUsrc
	output wire [1:0] ImmSrc,
	output wire [2:0] ALUControl,
	output wire TypeSrcB,
	output wire [1:0] RegESrc,
	output wire  RegVSrc,
	output wire [1:0] OpVResult,
	output wire [1:0] FPControl,
	output wire MOVcontrol,
	output wire RegEW,
	output wire Floating,
	output wire RegvVW
	);
	wire Branch;
	//Aumentado señales
	wire ControllerOp;
	
	wire Fpsize;
	wire [1:0] FlagWalu;
	wire [1:0] FlagWfp;
	
	//MAINDECODER
	maindecoder maindecoder(
    .OP(Op),
    .Funct4(Funct[4]),
    .Funct0(Funct[0]),
    .Branch(Branch),
    .MemToReg(MemtoReg),
    .MemW(MemW),
    .TypeSrcB(TypeSrcB),
    .ImmSrc(ImmSrc),
    .RegvVW(RegvVW),
    .RegEW(RegEW), // [Ojito será señal?]
    .RegESrc(RegESrc),
    .ControllerOp(ControllerOp),
    .Floating(Floating),
    .Fpsize(Fpsize)
    );
	//VECTOR DECODER
	
	vectorlogic vectorlog(
    .cmd(Funct[3:1]), // cmd reducido
    .floating(Floating), //
    .RegVSrc(RegVSrc),
    .OpVResult(OpVResult)
    );
    
	//ALU DECODER
	aludecoder aludec(
	.ControllerOp(ControllerOp),
    .cmd(Funct[3:1]),
    .funct(Funct[0]),
    .ALUControl(ALUControl),
    .FlagW(FlagWalu),//Wire alu
    .MOVcontrol(MOVcontrol)
    );
    
    //FLOTING POINT DECODER: 
    fpdecoder fpdec(
    .Fpsize(Fpsize),
    .cmd(Funct[2:0]),
    .RegvVW(RegvVW),
    .funct(Funct[0]),
    .FPControl(FPControl),
    .FlagW(FlagWfp)//Flag del FP
    );
    //Asignación del Flag:
    assign FlagW = Floating?FlagWfp:FlagWalu;
    //Asignación del PC:    
	assign PCS = ((Rd == 4'b1111) & RegEW) | Branch;
endmodule

module aludecoder(
    input ControllerOp,
    input [2:0] cmd,
    input funct,
    output reg [2:0] ALUControl,
    output reg [1:0] FlagW,
    output reg MOVcontrol
    );
    
    always @(*) begin
        if (ControllerOp) begin
            case (cmd)
                3'b000: begin // ADD or VADD
                    ALUControl = 3'b000;
                    FlagW = (funct) ? 2'b11 : 2'b00;
                end
                3'b001: begin // SUB or VSUB
                    ALUControl = 3'b001;
                    FlagW = (funct) ? 2'b11 : 2'b00;
                end
                3'b010: begin // AND or VAND
                    ALUControl = 3'b010;
                    FlagW = (funct) ? 2'b10 : 2'b00;
                end
                3'b011: begin // ORR or VORR
                    ALUControl = 3'b011;
                    FlagW = (funct) ? 2'b10 : 2'b00;
                end
                3'b100: begin // XOR or VXOR
                    ALUControl = 3'b100;
                    FlagW = (funct) ? 2'b10 : 2'b00;
                end
                3'b110: begin // VMOV
                    MOVcontrol = 1'b0;
                end
                3'b111: begin // VMOVM
                    MOVcontrol = 1'b1;
                end
                default: begin
                    ALUControl = 3'bxxx;
                    FlagW = 2'bxx;
                    MOVcontrol = 1'bx;
                end
            endcase
        end else begin
            // NOT DP (Data Processing)
            ALUControl = 3'b000;
            FlagW = 2'b00;
        end
    end
endmodule

module fpdecoder(
    input Fpsize,
    input [2:0] cmd,
    input RegvVW,
    input funct,
    output reg [1:0] FPControl,
    output reg [1:0] FlagW
	);
	always @(*) begin
		case ({Fpsize, cmd, RegvVW})
			5'bx0001: begin // VFPADD
				if (funct)begin
					FPControl = 2'b00;
					FlagW = 2'b11;
				end
				else begin
					FPControl = 2'b00;
					FlagW = 2'b00;
				end
			end
			
			5'b10000: begin // FPADD16
				if (funct)begin
					FPControl = 2'b01;
					FlagW = 2'b11;
				end
				else begin
					FPControl = 2'b01;
					FlagW = 2'b00;
				end
			end
			
			5'b10010: begin // FP16MUL
				if (funct)begin
					FPControl = 2'b11;
					FlagW = 2'b11;
				end
				else begin
					FPControl = 2'b11;
					FlagW = 2'b00;
				end
			end
			
			5'b00000: begin // FPADD32
				if (funct)begin
					FPControl = 2'b00;
					FlagW = 2'b11;
				end
				else begin
					FPControl = 2'b00;
					FlagW = 2'b00;
				end
			end
			
			5'b00010: begin // FPMULL32
				if (funct)begin
					FPControl = 2'b10;
					FlagW = 2'b11;
				end
				else begin
					FPControl = 2'b10;
					FlagW = 2'b00;
				end
			end
			
			default: begin
				FPControl = 2'b00;
				FlagW = 2'b00;
			end
		endcase
	end
endmodule

module maindecoder(
	input [2:0] OP,
	input Funct4,
	input Funct0,
	output reg Branch,
	output reg MemToReg,
	output reg MemW,
	output reg TypeSrcB,
	output reg [1:0] ImmSrc,
	output reg RegvVW,
	output reg RegEW,
	output reg [1:0] RegESrc,
	output reg ControllerOp,
	output reg Floating,
	output reg Fpsize
	);
	always @(*) begin
			case (OP)
				3'b000: begin
					if (!Funct4) begin// DP escalar Reg 
					
						Branch = 0;
						MemToReg = 0;
						MemW = 0;
						TypeSrcB = 0;
						//ImmSrc = 00;
						RegvVW = 0;
						RegEW = 1;
						RegESrc = 2'b00;
						ControllerOp = 1;
						Floating = 0;
						//Fpsize = 0;
					end else begin // DP escalar Imm
						Branch = 0;
						MemToReg = 0;
						MemW = 0;
						TypeSrcB = 1;
						ImmSrc = 2'b00;
						RegvVW = 0;
						RegEW = 1;
						RegESrc[0] = 0;
						ControllerOp = 1;
						Floating = 0;
						//Fpsize = 0;
					end
				end
				3'b001: begin
					// memory STR o LDR
					if (!Funct0) begin
						// memory STR
						Branch = 0;
						//MemToReg = 0;
						MemW = 1;
						TypeSrcB = 1;
						ImmSrc = 2'b01;
						RegvVW = 0;
						RegEW = 0;
						RegESrc = 2'b10;
						ControllerOp = 0;
						Floating = 0;
						//Fpsize = 0;
					end else begin
						// memory LDR
						Branch = 0;
						MemToReg = 1;
						MemW = 0;
						TypeSrcB = 1;
						ImmSrc = 2'b01;
						RegvVW = 0;
						RegEW = 1;
						RegESrc[0] = 1'b0;
						ControllerOp = 0;
						Floating = 0;
						//Fpsize = 0;
					end
				end
				3'b010: begin
					// Branch
						Branch = 1;
						MemToReg = 0;
						MemW = 0;
						TypeSrcB = 1;
						ImmSrc = 2'b10;
						RegvVW = 0;
						RegEW = 0;
						RegESrc[0] = 1'b1;
						ControllerOp = 0;
						Floating = 0;
						//Fpsize = 0;
				end
				3'b100: begin
					// VDP
						Branch = 0;
						//MemToReg = 0;
						MemW = 0;
						//TypeSrcB = 1;
						//ImmSrc = 2'b10;
						RegvVW = 1;
						RegEW = 0;
						//RegESrc[0] = 1'b1;
						ControllerOp = 1;
						Floating = 0;
						//Fpsize = 0;
				end
				3'b011: begin
					// VFPADD
						Branch = 0;
						//MemToReg = 0;
						MemW = 0;
						//TypeSrcB = 1;
						//ImmSrc = 2'b10;
						RegvVW = 1;
						RegEW = 0;
						//RegESrc[0] = 1'b1;
						ControllerOp = 1;
						Floating = 1;
						Fpsize = 1;
				end
				3'b101: begin
					// FP16ADD y // FP16MUL
						Branch = 0;
						MemToReg = 0;
						MemW = 0;
						TypeSrcB = 0;
						//ImmSrc = 2'b10;
						RegvVW = 0;
						RegEW = 1;
						RegESrc = 2'b00;
						ControllerOp = 1;
						Floating = 1;
						Fpsize = 1;
				end
				3'b110: begin
					// FP32ADD Y FP32MUL
						Branch = 0;
						MemToReg = 0;
						MemW = 0;
						TypeSrcB = 0;
						//ImmSrc = 2'b10;
						RegvVW = 0;
						RegEW = 1;
						RegESrc = 2'b00;
						ControllerOp = 1;
						Floating = 1;
						Fpsize = 0;
				end
				
			endcase
		end
endmodule


module vectorlogic(
    input [2:0] cmd,
    input floating,
    output reg RegVSrc,
    output reg [1:0] OpVResult
    );
    
    always @(*) begin
        if ((cmd == 3'b111) || (cmd==3'b110) ) begin
            OpVResult = 2'b10; 
            RegVSrc = 1;
        end
        else if (floating == 1) begin
            OpVResult = 2'b01; 
            RegVSrc = 0;
        end
        else begin
            OpVResult = 2'b00; 
            RegVSrc = 0;
        end
    end
endmodule

//condlogic
module condlogic (
	clk,
	reset,
    PCS,
    MemW,
    RegEW,
    RegVW,
    FlagW,
    VFlags,
    Cond,
    PCSrc,
    MemWrite,
    RegEWrite,
    RegVWrite,
    ExecuteResult
    );
	input wire clk;
	input wire reset;
    input wire PCS;
    input wire MemW;
    input wire RegEW;
    input wire RegVW;
    input wire [1:0] FlagW;
    input wire [19:0] VFlags;
    input wire [3:0] Cond;
    output wire PCSrc;
    output wire MemWrite;
    output wire RegEWrite;
    output wire RegVWrite;
    output wire [4:0] ExecuteResult;

    wire [19:0] Flags;
    wire Execute;

    assign Execute = ExecuteResult[4] | ExecuteResult[3] | ExecuteResult[2] | ExecuteResult[1] | ExecuteResult[0];
    
    assign PCSrc = PCS & Execute;
    assign MemWrite = MemW & Execute;
    assign RegEWrite = RegEW & Execute;
    assign RegVWrite = RegVW & Execute;

    wire [9:0] FlagsEnable = {FlagW[1] && ExecuteResult[4], FlagW[0] && ExecuteResult[4], FlagW[1] && ExecuteResult[3], FlagW[0] && ExecuteResult[3], FlagW[1] && ExecuteResult[2], FlagW[0] && ExecuteResult[2], FlagW[1] && ExecuteResult[1], FlagW[0] && ExecuteResult[1], FlagW[1] && ExecuteResult[0], FlagW[0] && ExecuteResult[0]};

    flopenr #(2) flagreg1(
        .clk(clk),
        .reset(reset),
        .en(FlagsEnable[0]),
        .d(VFlags[19:18]),
        .q(Flags[19:18])
    );

    flopenr #(2) flagreg2(
        .clk(clk),
        .reset(reset),
        .en(FlagsEnable[1]),
        .d(VFlags[17:16]),
        .q(Flags[17:16])
    );

    flopenr #(2) flagreg3(
        .clk(clk),
        .reset(reset),
        .en(FlagsEnable[2]),
        .d(VFlags[15:14]),
        .q(Flags[15:14])
    );

    flopenr #(2) flagreg4(
        .clk(clk),
        .reset(reset),
        .en(FlagsEnable[3]),
        .d(VFlags[13:12]),
        .q(Flags[13:12])
    );

    flopenr #(2) flagreg5(
        .clk(clk),
        .reset(reset),
        .en(FlagsEnable[4]),
        .d(VFlags[11:10]),
        .q(Flags[11:10])
    );

    flopenr #(2) flagreg6(
        .clk(clk),
        .reset(reset),
        .en(FlagsEnable[5]),
        .d(VFlags[9:8]),
        .q(Flags[9:8])
    );

    flopenr #(2) flagreg7(
        .clk(clk),
        .reset(reset),
        .en(FlagsEnable[6]),
        .d(VFlags[7:6]),
        .q(Flags[7:6])
    );

    flopenr #(2) flagreg8(
        .clk(clk),
        .reset(reset),
        .en(FlagsEnable[7]),
        .d(VFlags[5:4]),
        .q(Flags[5:4])
    );

    flopenr #(2) flagreg9(
        .clk(clk),
        .reset(reset),
        .en(FlagsEnable[8]),
        .d(VFlags[3:2]),
        .q(Flags[3:2])
    );

    flopenr #(2) flagreg10(
        .clk(clk),
        .reset(reset),
        .en(FlagsEnable[9]),
        .d(VFlags[1:0]),
        .q(Flags[1:0])
    );

    condcheckV condcheckV1 (
        .Cond(Cond),
        .Flags(Flags),
        .ExecuteResult(ExecuteResult)
    );
endmodule

module condcheckV (
    Cond,
    Flags,
    ExecuteResult
    );
    input wire [3:0] Cond;
    input wire [19:0] Flags;
    output wire [4:0] ExecuteResult;

    condcheck condcheck1 (
        .Cond(Cond),
        .Flags(Flags[19:16]),
        .CondEx(ExecuteResult[4])
    );

    condcheck condcheck2 (
        .Cond(Cond),
        .Flags(Flags[15:12]),
        .CondEx(ExecuteResult[3])
    );
    
    condcheck condcheck3 (
        .Cond(Cond),
        .Flags(Flags[11:8]),
        .CondEx(ExecuteResult[2])
    );

    condcheck condcheck4 (
        .Cond(Cond),
        .Flags(Flags[7:4]),
        .CondEx(ExecuteResult[1])
    );

    condcheck condcheck5 (
        .Cond(Cond),
        .Flags(Flags[3:0]),
        .CondEx(ExecuteResult[0])
    );
endmodule

module condcheck (
	Cond,
	Flags,
	CondEx
	);
    input wire [3:0] Cond;
    input wire [3:0] Flags;
    output reg CondEx;
    wire neg;
    wire zero;
    wire carry;
    wire overflow;
    wire ge;
    assign {neg, zero, carry, overflow} = Flags;
    assign ge = neg == overflow;
    always @(*)
        case (Cond)
            4'b0000: CondEx = zero;
            4'b0001: CondEx = ~zero;
            4'b0010: CondEx = carry;
            4'b0011: CondEx = ~carry;
            4'b0100: CondEx = neg;
            4'b0101: CondEx = ~neg;
            4'b0110: CondEx = overflow;
            4'b0111: CondEx = ~overflow;
            4'b1000: CondEx = carry & ~zero;
            4'b1001: CondEx = ~(carry & ~zero);
            4'b1010: CondEx = ge;
            4'b1011: CondEx = ~ge;
            4'b1100: CondEx = ~zero & ge;
            4'b1101: CondEx = ~(~zero & ge);
            4'b1110: CondEx = 1'b1;
            default: CondEx = 1'bx;
        endcase
endmodule

//datapath
module datapath(
    clk,
    reset,
    PCSrc,
    RegESrc,
    RegVSrc,
    MemtoReg,
    ExecuteResult,
    MemWrite,
    OpVResult,
    MOVControl,
    ALUControl,
    FPControl,
    RegVWrite,
    TypeSourceB,
    ImmSrc,
    Floating,
    RegEWrite,
    Instr,
    ReadData,
    PC,
    WriteData,
    ALUEResult,
    VFlags,
	VectorResult
);

    input wire clk;
    input wire reset;
    input wire PCSrc;
    input wire [1:0] RegESrc;
    input wire RegVSrc;
    input wire MemtoReg;
    input wire [4:0] ExecuteResult;
    input wire MemWrite;
    input wire [1:0] OpVResult;
    input wire MOVControl;
    input wire [2:0] ALUControl;
    input wire [1:0] FPControl;
    input wire RegVWrite;
    input wire TypeSourceB;
    input wire [1:0] ImmSrc;
    input wire Floating;
    input wire RegEWrite;
    //raritos
    input wire [31:0] Instr;
    input wire [31:0] ReadData;

    output wire [31:0] PC;
    output wire [31:0] WriteData;
    output wire [31:0] ALUEResult;

    output wire [19:0] VFlags; 
	
    output wire [150:0] VectorResult;
    
    wire [31:0] PCNext;
    wire [31:0] PCPlus4;
    wire [31:0] PCPlus8;
    wire [3:0] RA1;
    wire [3:0] RA2;
    wire [3:0] VA2;
    wire [31:0] RD1Value;
    wire [31:0] ExtendImm;
    wire [159:0] RDV1Value;
    wire [159:0] RDV2Value;
    wire [2:0] Index;
    wire [31:0] SrcEB;
    wire [159:0] SrcA;
    wire [159:0] SrcB;
    wire [31:0] EResult;
    wire [159:0] VALUResult;
    wire [159:0] VFPResult;
    wire [159:0] MOVResult;
    wire [159:0] VResult;
    wire [159:0] VDestiny;
    wire [31:0] EscalarResult;
    wire [19:0] VFPFlags;
    wire [19:0] VALUFlags;

	assign Index = Instr[19:17];

	assign ALUEResult = VALUResult[31:0];

    // --- muxes
    //pcmux
	mux2 pcmux(
		.d0(PCPlus4),
		.d1(EscalarResult),
		.s(PCSrc),
		.y(PCNext)
	);	

    //ra1mux
	mux2 #(4) ra1mux(
		.d0(Instr[19:16]),
		.d1(4'b1111),
		.s(RegESrc[0]),
		.y(RA1)
	);

    //ra2mux
	mux2 #(4) ra2mux(
		.d0(Instr[3:0]),
		.d1(Instr[15:12]),
		.s(RegESrc[1]),
		.y(RA2)
	);

    //va2mux
	mux2 #(4) va2mux(
		.d0(Instr[3:0]),
		.d1(Instr[15:12]),
		.s(RegVSrc),
		.y(VA2)
	);

    //imm2mux
	mux2 imm2mux(
		.d0(WriteData),
		.d1(ExtendImm),
		.s(TypeSourceB),
		.y(SrcEB)
	);

    //srcamux
	mux2 #(160) srcamux(
		.d0({5{RD1Value}}),
		.d1(RDV1Value),
		.s(RegVWrite),
		.y(SrcA)
	);

    //srcbmux

	mux2 #(160) srcbmux(
		.d0({5{SrcEB}}),
		.d1(RDV2Value),
		.s(RegVWrite),
		.y(SrcB)
	);

    //eresultmux
	mux2 #(32) eresultmux(
		.d0(VALUResult[31:0]),
		.d1(VFPResult[31:0]),
		.s(Floating),
		.y(EResult)
	);


	//vfpunit
	VFPUnit vfpunit(
		.Va(SrcA),
		.Vb(SrcB),
		.FPControl(FPControl),
		.VResult(VFPResult),
		.ALUFlags(VFPFlags)
	);

	
    //vresultmux
	mux3 #(160) vresultmux(
		.d00(VALUResult),
		.d01(VFPResult),
		.d10(MOVResult),
		.s(OpVResult),
		.y(VResult)
	);

    //resultmux
	mux2 #(32) resultmux(
		.d0(EResult),
		.d1(ReadData),
		.s(MemtoReg),
		.y(EscalarResult)
	);

    //flagmux
	mux2 #(20) flagmux(
		.d0(VALUFlags),
		.d1(VFPFlags),
		.s(Floating),
		.y(VFlags)
	);

    //---flop
	
    //pcreg
	flopr #(32) pcreg(
		.clk(clk),
		.reset(reset),
		.d(PCNext),
		.q(PC)
	);

    //---adders
    //pcadd1
	adder #(32) pcadd1(
		.a(PC),
		.b(32'b100),
		.y(PCPlus4)
	);

    //pcadd2
	adder #(32) pcadd2(
		.a(PCPlus4),
		.b(32'b100),
		.y(PCPlus8)
	);

    //vecregfile
	RegFileVectorial vecregfile(
		.clk(clk),
		.we3(RegVWrite),
		.ra1(Instr[19:16]),
		.ra2(VA2),
		.a3(Instr[15:12]),
		.wd3(VectorResult),
		.rd1(RDV1Value),
		.rd2(RDV2Value),
		.rd3(VDestiny)
	);
	 
    //escregfile
	RegFileEscalar escregfile(
		.clk(clk),
		.we3(RegEWrite),
		.ra1(RA1),
		.ra2(RA2),
		.wa3(Instr[15:12]),
		.wd3(EscalarResult),
		.r15(PCPlus8),
		.rd1(RD1Value),
		.rd2(WriteData)
	);

    //extend
	extend ext(
		.Instr(Instr[23:0]),
		.ImmSrc(ImmSrc),
		.ExtImm(ExtendImm)
	);

    //VALU
	VALU valumod(
		.SrcA(SrcA),
		.SrcB(SrcB),
		.ALUControl(ALUControl),
		.VALUResult(VALUResult),
		.VALUFlags(VALUFlags)
	);

    //VFP

    //VMOV
	MOVUnit vmovunit(
		.Index(Index),
		.VectorBase(RDV2Value),
		.Value(ExtendImm),
		.MOVControl(MOVControl),
		.MOVResult(MOVResult)
	);
	
    //VectorSelecter
	VectorSelector vectorselec(
		.VResult(VResult),
		.VDestiny(VDestiny),
		.ExecuteResult(ExecuteResult),
		.MOVResult(VectorResult)
	);
	
endmodule


//Modulos grandes
module RegFileEscalar(
	clk,
	we3,
	ra1,
	ra2,
	wa3,
	wd3,
	r15,
	rd1,
	rd2
	);
	input wire clk;
	input wire we3;
	input wire [3:0] ra1;
	input wire [3:0] ra2;
	input wire [3:0] wa3;
	input wire [31:0] wd3;
	input wire [31:0] r15;
	output wire [31:0] rd1;
	output wire [31:0] rd2;
	reg [31:0] rf [14:0];
	//Paso de valores a los registros
	//6 y 7 ADD 32
	initial begin
	   //Add 32
        rf[6] = 32'b10111111101111001100110011001101; //-1.475
        rf[7] = 32'b00111010101000010011011111110100; //0.00123
        //Registro 8 es la suma [-1.47377002239227294921875]
        //Add16
        rf[9] = 32'b000000000000000_0100100100100110; //10.3
        rf[10] = 32'b000000000000000_0100101000011010; //12.2
        //Se espera: //22.5 en R11
        //MUL32
        rf[12] = 32'b1_01111101_00110011001100110011010; //-0.3 *
        rf[13] = 32'b0_10000111_11110100010000000000000; //500.25 
        // En R5 = -150.07501
        //Mull 16
        rf[2] = 32'b000000000000000_1011111000000000; //-1.5 
        rf[3] = 32'b0000000000000000_1100000000000000; //-2 
        //Resultado en el R14  3 
        

    end
	
	
	
	always @(posedge clk)
		if (we3)
			rf[wa3] <= wd3;
	assign rd1 = (ra1 == 4'b1111 ? r15 : rf[ra1]);
	assign rd2 = (ra2 == 4'b1111 ? r15 : rf[ra2]);
endmodule

module RegFileVectorial(
	clk,
	we3,
	ra1,
	ra2,
	a3,
	wd3,
	rd1,
	rd2,
	rd3
	);
	input wire clk;
	input wire we3;
	input wire [3:0] ra1;
	input wire [3:0] ra2;
	input wire [3:0] a3;
	input wire [159:0] wd3;
	output wire [159:0] rd1;
	output wire [159:0] rd2;
	output wire [159:0] rd3;
	reg [159:0] rf [15:0];

	always @(posedge clk)
		if (we3)
			rf[a3] <= wd3;

	assign rd1 = rf[ra1];
	assign rd2 = rf[ra2];
	assign rd3 = rf[a3];
endmodule

module extend (
	Instr,
	ImmSrc,
	ExtImm
	);
	input wire [23:0] Instr;
	input wire [1:0] ImmSrc;
	output reg [31:0] ExtImm;
	always @(*)
		case (ImmSrc)
			2'b00: ExtImm = {24'b000000000000000000000000, Instr[7:0]};
			2'b01: ExtImm = {20'b00000000000000000000, Instr[11:0]};
			2'b10: ExtImm = {{6 {Instr[23]}}, Instr[23:0], 2'b00};
			default: ExtImm = 32'bxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx;
		endcase
endmodule

module adder (
	a,
	b,
	y
	);
	parameter WIDTH = 8;
	input wire [WIDTH - 1:0] a;
	input wire [WIDTH - 1:0] b;
	output wire [WIDTH - 1:0] y;
	assign y = a + b;
endmodule

module alu ( input [31:0] a,b,
             input [2:0] ALUControl,
             output reg [31:0] Result, //assign always block
             output wire [3:0] ALUFlags); //explicit wire for assign with {}
  
  wire negative, zero, carry, overflow; // define wire for each flag (n,z,c,v)
  wire [32:0] sum;
  
  
  assign sum = a + (ALUControl[0]? ~b: b) + ALUControl[0]; //ADDER: two's complement
  
  always @(*)
    casex (ALUControl[2:0]) //case, casex, casez
      3'b00?: Result = sum;
      3'b010: Result = a & b;
      3'b011: Result = a | b;
      3'b100: Result = a ^ b;
    endcase
  
 //flags: result -> negative, zero
  assign negative = Result[31];
  assign zero = (Result == 32'b0);
  //flags: additional logic -> v, c
  assign carry = (ALUControl[1]==1'b0) & sum[32];
  assign overflow = (ALUControl[1]==1'b0) & ~(a[31] ^ b[31] ^ ALUControl[0]) & (a[31] ^ sum[31]);

  assign ALUFlags = {negative, zero, carry, overflow};
endmodule

module VALU(
	SrcA,
	SrcB,
	ALUControl,
	VALUResult,
	VALUFlags
	);

	input wire [159:0] SrcA;
	input wire [159:0] SrcB;
	input wire [2:0] ALUControl;
	output wire [159:0] VALUResult;
	output wire [19:0] VALUFlags;

	alu aluI5(
		.a(SrcA[159:128]),
		.b(SrcB[159:128]),
		.ALUControl(ALUControl),
		.Result(VALUResult[159:128]),
		.ALUFlags(VALUFlags[19:16])
	);

	alu aluI4(
		.a(SrcA[127:96]),
		.b(SrcB[127:96]),
		.ALUControl(ALUControl),
		.Result(VALUResult[127:96]),
		.ALUFlags(VALUFlags[15:12])
	);

	alu aluI3(
		.a(SrcA[95:64]),
		.b(SrcB[95:64]),
		.ALUControl(ALUControl),
		.Result(VALUResult[95:64]),
		.ALUFlags(VALUFlags[11:8])
	);

	alu aluI2(
		.a(SrcA[63:32]),
		.b(SrcB[63:32]),
		.ALUControl(ALUControl),
		.Result(VALUResult[63:32]),
		.ALUFlags(VALUFlags[7:4])
	);

	alu aluI1(
		.a(SrcA[31:0]),
		.b(SrcB[31:0]),
		.ALUControl(ALUControl),
		.Result(VALUResult[31:0]),
		.ALUFlags(VALUFlags[3:0])
	);
endmodule

module VectorSelector (
    VResult,
    VDestiny,
    ExecuteResult,
    MOVResult
    );
    input wire [159:0] VResult;
    input wire [159:0] VDestiny;
    input wire [4:0] ExecuteResult;
    output wire [159:0] MOVResult;
    wire [159:0] ERExtended;
  
    assign ERExtended = { 
        {32{ExecuteResult[4]}}, 
        {32{ExecuteResult[3]}}, 
        {32{ExecuteResult[2]}}, 
        {32{ExecuteResult[1]}}, 
        {32{ExecuteResult[0]}} 
    };
    
    assign MOVResult = (VResult & ERExtended) | (VResult & (~ERExtended));
endmodule

module MOVUnit(
	Index,
	VectorBase,
	Value,
	MOVControl,
	MOVResult
	);
	input wire [2:0] Index;
	input wire [159:0] VectorBase;
	input wire [31:0] Value;
	input wire MOVControl;
	output wire [159:0] MOVResult;
	wire [4:0] mascara;
	wire [159:0] mascaraExtended;


	wire [159:0] MOVR;

	mux5 #(5) mascaraMOVMux(
		.d000(5'b11110),
		.d001(5'b11101),
		.d010(5'b11011),
		.d011(5'b10111),
		.d100(5'b01111),
		.s(Index),
		.y(mascara)
	);

	assign mascaraExtended = { 
		{32{mascara[4]}}, 
		{32{mascara[3]}}, 
		{32{mascara[2]}}, 
		{32{mascara[1]}}, 
		{32{mascara[0]}} 
	};

    assign MOVR = ((VectorBase & mascaraExtended) | ({32{Value}} & (~mascaraExtended)));

	mux2 #(159) muxito(
		.d0(MOVR),
		.d1({5{Value}}),
		.s(MOVControl),
		.y(MOVResult)
	);

endmodule

//Modulos chicos
module flopenr (
	clk,
	reset,
	en,
	d,
	q
	);
	parameter WIDTH = 8;
	input wire clk;
	input wire reset;
	input wire en;
	input wire [WIDTH - 1:0] d;
	output reg [WIDTH - 1:0] q;
	always @(posedge clk or posedge reset)
		if (reset)
			q <= 0;
		else if (en)
			q <= d;
endmodule

module flopr (
	clk,
	reset,
	d,
	q
	);

	parameter WIDTH = 8;
		input wire clk;
		input wire reset;
		input wire [WIDTH - 1:0] d;
		output reg [WIDTH - 1:0] q;
		always @(posedge clk or posedge reset)
			if (reset)
				q <= 0;
			else
				q <= d;
endmodule

module mux3 (
	d00,
	d01,
	d10,
	s,
	y
	);

	parameter WIDTH = 32;
	input wire [WIDTH-1:0] d00;
	input wire [WIDTH-1:0] d01;
	input wire [WIDTH-1:0] d10;
	input wire [1:0] s;
	output wire [WIDTH-1:0] y;

	assign y = (s == 2'b00) ? d00 : (s == 2'b01) ? d01 : d10;
endmodule

module mux2 (
	d0,
	d1,
	s,
	y
	);

	parameter WIDTH = 32;
	input wire [WIDTH-1:0] d0;
	input wire [WIDTH-1:0] d1;
	input wire s;
	output wire [WIDTH-1:0] y;

	assign y = (s == 1'b0) ? d0 : d1;
endmodule

module mux5 (
	d000,
	d001,
	d010,
	d011,
	d100,
	s,
	y
	);

	parameter WIDTH = 32;
	input wire [WIDTH-1:0] d000;
	input wire [WIDTH-1:0] d001;
	input wire [WIDTH-1:0] d010;
	input wire [WIDTH-1:0] d011;
	input wire [WIDTH-1:0] d100;
	input wire [2:0] s;
	output wire [WIDTH-1:0] y;

	assign y = (s == 3'b000) ? d000 : (s == 3'b001) ? d001 : (s == 3'b010) ? d010 : (s == 3'b011) ? d011 : d100;
endmodule


// VFPUNIT

module VFPUnit(
    input wire [159:0] Va, Vb,
    input wire [1:0] FPControl,
    output wire [159:0] VResult,
    output wire [19:0] ALUFlags
);

    wire [31:0] a_1, a_2, a_3, a_4, a_5;
    wire [31:0] b_1, b_2, b_3, b_4, b_5;
    wire [31:0] Result_1, Result_2, Result_3, Result_4, Result_5;
    wire [3:0] ALUFlags_1, ALUFlags_2, ALUFlags_3, ALUFlags_4, ALUFlags_5;

    assign {a_1, a_2, a_3, a_4, a_5} = {Va[31:0], Va[63:32], Va[95:64], Va[127:96], Va[159:128]};
    assign {b_1, b_2, b_3, b_4, b_5} = {Vb[31:0], Vb[63:32], Vb[95:64], Vb[127:96], Vb[159:128]};

    FPUnit u1 (.a(a_1), .b(b_1), .FPControl(FPControl), .Result(Result_1), .ALUFlags(ALUFlags_1));
    FPUnit u2 (.a(a_2), .b(b_2), .FPControl(FPControl), .Result(Result_2), .ALUFlags(ALUFlags_2));
    FPUnit u3 (.a(a_3), .b(b_3), .FPControl(FPControl), .Result(Result_3), .ALUFlags(ALUFlags_3));
    FPUnit u4 (.a(a_4), .b(b_4), .FPControl(FPControl), .Result(Result_4), .ALUFlags(ALUFlags_4));
    FPUnit u5 (.a(a_5), .b(b_5), .FPControl(FPControl), .Result(Result_5), .ALUFlags(ALUFlags_5));

    assign VResult = {Result_5, Result_4, Result_3, Result_2, Result_1};
    assign ALUFlags = {ALUFlags_5, ALUFlags_4, ALUFlags_3, ALUFlags_2, ALUFlags_1};
endmodule



module FPUnit(
    input [31:0] a, b,
    input [1:0] FPControl,
    output reg [31:0] Result,
    output reg [3:0] ALUFlags
  );

  wire neg, zero, carry, overflow;
  wire [31:0] faddResult32;//FADD RESULT 32
  wire [15:0] faddResult16;//FADD RESULT 16
  wire [31:0] fmulResult32;// FMULT RESUL 32
  wire [15:0] fmulResult16;// FMULT RESUL 32

  //Instanciaci�n de los flags
  wire [3:0] ALUFlagsAdd32; //Alu flags 32 ADD
  wire [3:0] ALUFlagsAdd16; //Alu flags 16 ADD
  wire [3:0] ALUFlagsMul32; //Alu flags 32 MUL
  wire [3:0] ALUFlagsMul16; //Alu flags 16 MUL


  fadd fadd_instance32(
         .a(a),
         .b(b),
         .Result(faddResult32),
         .ALUFlags(ALUFlagsAdd32)
       );
  fadd16 fadd_instance16(
           .a(a[15:0]),//Solo le doy 16 bits del menos al mas significativ
           .b(b[15:0]),
           .Result(faddResult16),
           .ALUFlags(ALUFlagsAdd16)
         );

  fpmul32 fmul_instance32(
            .a(a),
            .b(b),
            .Result(fmulResult32),
            .ALUFlags(ALUFlagsMul32)
          );

  fpmul16 fmul_instance16(
      .a(a[15:0]),
      .b(b[15:0]),
      .Result(fmulResult16),
      .ALUFlags(ALUFlagsMul16)
  );

  always @(*)
  begin
    case (FPControl)
      2'b00:
      begin
        Result = faddResult32;
        ALUFlags = ALUFlagsAdd32;
      end

      2'b01:
      begin
        Result = faddResult16;
        ALUFlags = ALUFlagsAdd16;
      end

      2'b11:
      begin
        Result = fmulResult16;
        ALUFlags = ALUFlagsMul16;
      end
      2'b10:
      begin
        Result = fmulResult32;
        ALUFlags = ALUFlagsMul32;
      end
    endcase
  end



endmodule



module sub( m1,m2,q,em, ee, e1,e2, m_R, round, exp_R);//MODULE SUB
  input [22:0]m1;//mantisa sin bit implicito
  input [22:0]m2; //mantisa sin bit implicito

  input [7:0] q;//La cantidad q debo shiftear
  input [7:0] e1;
  input [7:0] e2;
  output [7:0] exp_R;
  input em; //Si el exponente de m1 es mayor que el de m2
  input ee; //Si los exponentes son iguales
  output [24:0]m_R;
  output round;

  //A�ado el bit implicito
  wire [255:0]m1b; //Para mejorar la precision m1b y m2b deberia ser de aprox tama�o 255
  wire [255:0]m2b;


  //-------
  assign m1b[254:232] = m1[22:0];
  assign m1b[231:0] = 232'b0;
  assign m2b[231:0] = 232'b0;
  assign m2b[254:232] = m2[22:0];
  assign m1b[255]=1'b1;
  assign m2b[255]=1'b1;
  //Ya tengo las mantisas bonitas

  //Ahora el shifteo:
  reg [255:0]shifted_m1b;
  reg [255:0]shifted_m2b;

  always @*
  begin
    if (em)
    begin
      shifted_m2b = (q > 0) ? (m2b >> q) : m2b;//Arreglando
      shifted_m1b = m1b; // m1 ya est� en la posici�n correcta
    end
    else
    begin
      // Desplazar m1 a la posici�n de m2 (|q| posiciones a la izquierda si q < 0)
      shifted_m1b = (q > 0) ? (m1b >> q) : m1b; //Siempre hago shifteo del menor al mayor
      shifted_m2b = m2b; // m2 ya est� en la posici�n correcta

    end
  end
  //Ahora por fin si puedo restar pipi
  reg [255:0]resta;

  always @*
  begin
    if (em)
    begin //Para no hacer el swap
      resta = shifted_m1b - shifted_m2b;
    end
    else
    begin
      if (ee)
        resta = shifted_m1b - shifted_m2b;
      else // Cuando la otra mantisa es mas grandecita
        resta = shifted_m2b - shifted_m1b;
    end
  end

  //Logica para el shifteo o normalizacion:
  reg [7:0]toshift;
  //AQUI toshit debe tener el valor del cuanto es necesario
  //el dhifteo en resta para llegar a 1, por ejemplo si es
  //0001, to shift debe ser 0000011
  integer i;
  reg found_one;

  always @*
  begin
    toshift = 0;
    found_one = 0;
    for (i = 255; i >= 0; i = i - 1)
    begin
      if (!found_one && resta[i] == 1'b1)
      begin
        toshift = 255 - i;
        found_one = 1;
      end
    end
  end
  //Hasta aqui ya tenemos toshift

  //assign m_R[24] = 1'b0;  //Aqui le doy 25 bits para no ajsutar la logica de los condicionales [0] no hay overflow virtual
  assign m_R[24:0] = {resta[255:231]<<toshift}; //Aqui le doy 25 bits para no ajsutar la logica de los condicionales
  //Necesito logica para el redondeo por eso
  assign round = resta[229]; //Si el 5 bit es 1 se redondea:

  //Calculo del nuevo exponente:
  reg [7:0] exp_R_reg;

  always @*
  begin
    if (m_R[24:0] == 25'b0 && toshift == 8'b0)
    begin
      exp_R_reg = 8'b0; // Si m_R es todo 0 y toshift es todo 0, exp_R se establece en 0.
    end
    else
    begin
      exp_R_reg = em ? (e1 - toshift) : (e2 - toshift);
    end
  end

  assign exp_R = exp_R_reg;






endmodule

module fadd(//MODULE ADD32
    input [31:0] a, b,
    output reg [31:0] Result,
    output reg [3:0] ALUFlags
  );
  wire s1, s2;
  wire [7:0] e1, e2;
  wire [23:0] m1, m2;
  //Asignaci�n de las partes:
  //Signos
  assign s1 = a[31];
  assign s2 = b[31];
  //Exponentes
  assign e1 = a[30:23];
  assign e2 = b[30:23];
  //Mantisas
  assign m1[22:0] = a[22:0];
  assign m2[22:0] = b[22:0];

  //1) El exponente mayor
  wire em; //Es 1 si e1 es mayor caso contrario toma 0
  assign em = (e1 > e2);
  wire ee; //Expos iguales
  assign ee = (e1 == e2);
  wire se;// Signos iguales
  assign se = (s1 == s2);


  // Calculo el valor que debo shitear
  reg [7:0] q; //valor de shifteo
  always @*
  begin
    if (em)
    begin
      // Calculo q = e1 - e2
      q = e1 - e2;
    end
    else
    begin
      // Calculo q = e2 - e1
      q = e2 - e1;
    end
  end

  //A�ado el bit implicito a las mantisas
  wire b1;
  assign b1 =1;
  assign m1[23] = b1;
  assign m2[23] = b1;
  //Uso el valor de q para shiftear el numero de posicions a la mantisa con el exponente menor
  wire round; //---IMPORTANTEEE--- o el redondeo positivo

  reg [23:0] shifted_m1, shifted_m2;
  reg [24:0] mantisa_for_round; //Mantisa q nos permite ver el redondeo
  always @*
  begin
    if (em)
    begin
      // Desplazar m2 a la posici�n de m1 (q posiciones a la izquierda si q > 0)
      shifted_m2 = (q > 0) ? (m2 >> q) : m2;//Arreglando
      shifted_m1 = m1; // m1 ya est� en la posici�n correcta
      mantisa_for_round = (q > 0) ? (m2 >> q) : m2;
    end
    else
    begin
      // Desplazar m1 a la posici�n de m2 (|q| posiciones a la izquierda si q < 0)
      shifted_m1 = (q > 0) ? (m1 >> q) : m1; //Siempre hago shifteo del menor al mayor
      shifted_m2 = m2; // m2 ya est� en la posici�n correcta
      mantisa_for_round = (q > 0) ? (m1 >> q) : m1;

    end
  end
  wire [24:0]m_R;
  wire [7:0]exp_R;
  wire roundn;//Para ver si se usa el redondeo negativo
  sub subtuki( .m1(m1),.m2(m2),.q(q),.em(em),.ee(ee),.e1(e1),.e2(e2), .m_R(m_R), .round(roundn), .exp_R(exp_R));

  //BIT PARA REDONDEO:
  assign round = (se?mantisa_for_round[0]:roundn);
  //Si son signos iguales redondeo de suma, sino el de resta



  //Sumar las mantisas adecuadamente
  reg [24:0] mantisaS; //Ahora es de 25 bits, para el caso de overflow de mantisas
  always @*
  begin
    if (se)
    begin
      mantisaS = shifted_m1 + shifted_m2;  // signos iguales
    end
    else
    begin
      mantisaS = m_R;  // signos diferentes
    end
  end

  //"Peque�o" bucle apra reducir la cantidad de ifs:
  reg [7:0]exp;

  always @*
  begin
    if (em) //e1>e2
      if (se)
        exp = e1;
      else // Caso de resta
        exp = exp_R;

    else //e1<=e2
    begin
      if (ee) //e1 == e2
        if (se)
          exp = e1+1;
        else // Caso de resta
          exp = exp_R;
      else //e1<e2
        if (se)
          exp = e2;
        else // Caso de resta
          exp = exp_R;
    end
  end



  //Aqui lo redondeo
  always @*
  begin
    if (em) //e1>e2
    begin
      if(!mantisaS[24])
        Result = {s1, exp, round ? {mantisaS[22:0]+1'b1} : { mantisaS[22:0]}};
      else
        Result = {s1, exp, round ? {mantisaS[23:1]+1'b1} : { mantisaS[23:1]}};
    end
    else //e1<=e2
    begin
      if (ee) //e1 == e2
      begin
        if(!mantisaS[24])
          Result = {s1, exp, round ? {mantisaS[22:0]+1'b1} : { mantisaS[22:0]}};

        else
          Result = {s1, exp, round ? {mantisaS[23:1]+1'b1} : { mantisaS[23:1]}};

      end
      else //e1<e2
      begin
        if(!mantisaS[24])
          Result = {s2, exp, round ? {mantisaS[22:0]+1'b1} : { mantisaS[22:0]}};
        else
          Result = {s2, exp, round ? {mantisaS[23:1]+1'b1}: { mantisaS[23:1]}};
      end


    end
  end

  //LOGICA DE FLAGS.
  always @*
  begin
    // Zero Flag
    ALUFlags[3] = (mantisaS == 0);

    // Negative Flag
    ALUFlags[2] = (mantisaS[24] == 1);

    // Carry Flag (puede no ser relevante en punto flotante, as� que lo dejamos en 0)
    ALUFlags[1] = 1'b0;

    // Overflow Flag
    ALUFlags[0] = ((mantisaS[24] == 1) && (se == 1) && (mantisaS[23:0] == 24'b01111111111111111111111)) ||
            ((mantisaS[24] == 0) && (se == 1) && (mantisaS[23:0] == 24'b10000000000000000000000)) ||
            ((mantisaS[24] == 0) && (se == 0) && (mantisaS[23:0] == 24'b01111111111111111111111));
  end

endmodule



module sub16(
    input [9:0] m1, //mantisa sin bit implicito
    input [9:0] m2, //mantisa sin bit implicito
    input [4:0] q, //La cantidad que debo shiftear
    input [4:0] e1,
    input [4:0] e2,
    output [4:0] exp_R,
    input em, //Si el exponente de m1 es mayor que el de m2
    input ee, //Si los exponentes son iguales
    output [11:0] m_R,
    output round
  );

  //A�ado el bit implicito
  wire [127:0] m1b; //Para mejorar la precision m1b y m2b deber�a ser de tama�o 127
  wire [127:0] m2b;

  assign m1b[126:116] = m1[9:0];
  assign m1b[115:0] = 116'b0;
  assign m2b[126:116] = m2[9:0];
  assign m2b[115:0] = 116'b0;
  assign m1b[127] = 1'b1;
  assign m2b[127] = 1'b1;

  //Ahora el shifteo:
  reg [127:0] shifted_m1b;
  reg [127:0] shifted_m2b;

  always @*
  begin
    if (em)
    begin
      shifted_m2b = (q > 0) ? (m2b >> q) : m2b;
      shifted_m1b = m1b; // m1 ya est� en la posici�n correcta
    end
    else
    begin
      shifted_m1b = (q > 0) ? (m1b >> q) : m1b;
      shifted_m2b = m2b; // m2 ya est� en la posici�n correcta
    end
  end

  //Ahora por fin se puede restar
  reg [127:0] resta;

  always @*
  begin
    if (em)
    begin
      resta = shifted_m1b - shifted_m2b;
    end
    else
    begin
      if (ee)
        resta = shifted_m1b - shifted_m2b;
      else // Cuando la otra mantisa es m�s grande
        resta = shifted_m2b - shifted_m1b;
    end
  end

  //Logica para el shifteo o normalizacion:
  reg [4:0] toshift;
  integer i;
  reg found_one;

  always @*
  begin
    toshift = 0;
    found_one = 0;
    for (i = 127; i >= 0; i = i - 1)
    begin
      if (!found_one && resta[i] == 1'b1)
      begin
        toshift = 127 - i;
        found_one = 1;
      end
    end
  end

  assign m_R[11:0] = {resta[127:116] << toshift};
  assign round = resta[115]; //Si el bit 5 es 1 se redondea

  //Calculo del nuevo exponente:
  reg [4:0] exp_R_reg;

  always @*
  begin
    if (m_R[11:0] == 12'b0 && toshift == 5'b0)
    begin
      exp_R_reg = 5'b0; // Si m_R es todo 0 y toshift es todo 0, exp_R se establece en 0.
    end
    else
    begin
      exp_R_reg = em ? (e1 - toshift) : (e2 - toshift);
    end
  end

  assign exp_R = exp_R_reg;

endmodule



//FLOTING POINT ADD 16 BITS:

module fadd16(
    input [15:0] a, b,
    output reg [15:0] Result,
    output reg [3:0] ALUFlags
  );
  wire s1, s2;
  wire [4:0] e1, e2;
  wire [10:0] m1, m2;

  //Asignaci�n de las partes:
  //Signos
  assign s1 = a[15];
  assign s2 = b[15];
  //Exponentes
  assign e1 = a[14:10];
  assign e2 = b[14:10];
  //Mantisas
  assign m1[9:0] = a[9:0];
  assign m2[9:0] = b[9:0];

  //1) El exponente mayor
  wire em; //Es 1 si e1 es mayor caso contrario toma 0
  assign em = (e1 > e2);
  wire ee; //Expos iguales
  assign ee = (e1 == e2);
  wire se; // Signos iguales
  assign se = (s1 == s2);

  // Calculo el valor que debo shitear
  reg [4:0] q; //valor de shifteo
  always @*
  begin
    if (em)
    begin
      // Calculo q = e1 - e2
      q = e1 - e2;
    end
    else
    begin
      // Calculo q = e2 - e1
      q = e2 - e1;
    end
  end

  //A�ado el bit implicito a las mantisas
  wire b1;
  assign b1 = 1;
  assign m1[10] = b1;
  assign m2[10] = b1;

  //Uso el valor de q para shiftear el numero de posicions a la mantisa con el exponente menor
  wire round; //---IMPORTANTEEE--- o el redondeo positivo

  reg [10:0] shifted_m1, shifted_m2;
  reg [11:0] mantisa_for_round; //Mantisa que nos permite ver el redondeo
  always @*
  begin
    if (em)
    begin
      // Desplazar m2 a la posici�n de m1 (q posiciones a la izquierda si q > 0)
      shifted_m2 = (q > 0) ? (m2 >> q) : m2;
      shifted_m1 = m1; // m1 ya est� en la posici�n correcta
      mantisa_for_round = (q > 0) ? (m2 >> q) : m2;
    end
    else
    begin
      // Desplazar m1 a la posici�n de m2 (|q| posiciones a la izquierda si q < 0)
      shifted_m1 = (q > 0) ? (m1 >> q) : m1;
      shifted_m2 = m2; // m2 ya est� en la posici�n correcta
      mantisa_for_round = (q > 0) ? (m1 >> q) : m1;
    end
  end

  wire [11:0] m_R;
  wire [4:0] exp_R;
  wire roundn; //Para ver si se usa el redondeo negativo
  sub16 subtuki( .m1(m1), .m2(m2), .q(q), .em(em), .ee(ee), .e1(e1), .e2(e2), .m_R(m_R), .round(roundn), .exp_R(exp_R));

  //BIT PARA REDONDEO:
  assign round = (se ? mantisa_for_round[0] : roundn);

  //Sumar las mantisas adecuadamente
  reg [11:0] mantisaS; //Ahora es de 12 bits, para el caso de overflow de mantisas
  always @*
  begin
    if (se)
    begin
      mantisaS = shifted_m1 + shifted_m2;  // signos iguales
    end
    else
    begin
      mantisaS = m_R;  // signos diferentes
    end
  end

  //"Peque�o" bucle para reducir la cantidad de ifs:
  reg [4:0] exp;

  always @*
  begin
    if (em) //e1>e2
      if (se)
        exp = e1;
      else // Caso de resta
        exp = exp_R;
    else //e1<=e2
    begin
      if (ee) //e1 == e2
        if (se)
          exp = e1 + 1;
        else // Caso de resta
          exp = exp_R;
      else //e1<e2
        if (se)
          exp = e2;
        else // Caso de resta
          exp = exp_R;
    end
  end

  //Aqui lo redondeo
  always @*
  begin
    if (em) //e1>e2
    begin
      if(!mantisaS[11])
        Result = {s1, exp, round ? {mantisaS[9:0] + 1'b1} : {mantisaS[9:0]}};
      else
        Result = {s1, exp, round ? {mantisaS[10:1] + 1'b1} : {mantisaS[10:1]}};
    end
    else //e1<=e2
    begin
      if (ee) //e1 == e2
      begin
        if(!mantisaS[11])
          Result = {s1, exp, round ? {mantisaS[9:0] + 1'b1} : {mantisaS[9:0]}};
        else
          Result = {s1, exp, round ? {mantisaS[10:1] + 1'b1} : {mantisaS[10:1]}};
      end
      else //e1<e2
      begin
        if(!mantisaS[11])
          Result = {s2, exp, round ? {mantisaS[9:0] + 1'b1} : {mantisaS[9:0]}};
        else
          Result = {s2, exp, round ? {mantisaS[10:1] + 1'b1} : {mantisaS[10:1]}};
      end
    end
  end

  //LOGICA DE FLAGS.
  always @*
  begin
    // Zero Flag
    ALUFlags[3] = (mantisaS == 0);

    // Negative Flag
    ALUFlags[2] = (mantisaS[11] == 1);

    // Carry Flag (puede no ser relevante en punto flotante, as� que lo dejamos en 0)
    ALUFlags[1] = 1'b0;

    // Overflow Flag
    ALUFlags[0] = ((mantisaS[11] == 1) && (se == 1) && (mantisaS[10:0] == 11'b01111111111)) ||
            ((mantisaS[11] == 0) && (se == 1) && (mantisaS[10:0] == 11'b10000000000)) ||
            ((mantisaS[11] == 0) && (se == 0) && (mantisaS[10:0] == 11'b01111111111));
  end

endmodule


module mantissa_multiplier32 (
    input [23:0] mant_a,
    input [23:0] mant_b,
    output [47:0] mant_result
  );
  assign mant_result = mant_a * mant_b;
endmodule

module exponent_adder32 (
    input [7:0] exp_a,
    input [7:0] exp_b,
    output [7:0] exp_result
  );
  assign exp_result = exp_a + exp_b - 8'd127;
endmodule

module sign_handler32 (
    input sign_a,
    input sign_b,
    output sign_result
  );
  assign sign_result = sign_a ^ sign_b;
endmodule

module normalizer32 (
    input [47:0] mant_result,
    input [7:0] exp_result,
    output [22:0] normalized_mant,
    output [7:0] normalized_exp
  );
  wire [47:0] shifted_mant;
  wire [7:0] adjusted_exp;

  assign shifted_mant = mant_result >> 1;
  assign adjusted_exp = exp_result + 1;

  assign normalized_mant = mant_result[47] ? shifted_mant[46:24] : mant_result[45:23];
  assign normalized_exp = mant_result[47] ? adjusted_exp : exp_result;
endmodule

//David
module fpmul32(
    input [31:0] a, 
    input [31:0] b,
    output reg [31:0] Result,
    output [3:0] ALUFlags
);

    reg sign;
    wire [47:0] mantissa_mult; // (1 + 23) + (1 + 23)
    wire [23:0] mantissa_a, mantissa_b; // mantissa + 1
    reg [25:0] mantissa_result;
    reg [8:0] exponent_sum; // 1 bit mas caso la suma  es de 9 bits
    reg normalized;
    reg [31:0] result;
    reg [26-1:0] temp;

    // Ver si exponente es cero, caso lo sea hidden bit es cero, caso contrario 1
    assign mantissa_a = (a[30:23] == 8'b00000000) ? {1'b0, a[22:0]} : {1'b1, a[22:0]}; 
    assign mantissa_b = (b[30:23] == 8'b00000000) ? {1'b0, b[22:0]} : {1'b1, b[22:0]}; 
    assign mantissa_mult = mantissa_a * mantissa_b; // Multiplicar mantissas

    always @(*) begin
        exponent_sum = (a[30:23] + b[30:23]) - 127; // -127 remover bias
        normalized = 1;

        // Normalizar
        if (mantissa_mult[47]) begin //Si el primer bit es 1 shifteo y sumo al exponente
            mantissa_result = mantissa_mult[47:21];
            exponent_sum = exponent_sum + 1;
        end else begin
            mantissa_result = mantissa_mult[46:20];
            normalized = 0;
        end

        // Checkear underflow y overflow
        if (exponent_sum < 128) begin 
            result = 32'b0; //Underflow todo cero
        end 
        else if (exponent_sum > 254) begin
            // Overflow: seteo a infinito (0 11111111 0000000...)
            result = {sign, 8'b11111111, 23'b0};
        end 
        else begin
            
            case(mantissa_result[2:0])
                3'b000: mantissa_result = mantissa_result;
                3'b001: mantissa_result = mantissa_result;
                3'b010: mantissa_result = mantissa_result;
                3'b011  : begin
                  mantissa_result={mantissa_result[25:3] ,3'b100};
				end
                3'b100  : begin
                    mantissa_result={mantissa_result[25:3] ,3'b100};
                end
                3'b101 : begin
                    mantissa_result={mantissa_result[25:3] ,3'b100};
                end
                3'b110 :begin
                    temp={mantissa_result[25:3] ,3'b100};
                    mantissa_result[25:2]=temp[25:2]+1;
                    mantissa_result[1:0]=2'b0;
                end
                3'b111:begin
                    temp={a[25:3] ,3'b100};
                    mantissa_result[25:2]=temp[25:2]+1;
                    mantissa_result[1:0]=2'b0; 
                end      
                default: begin
                        mantissa_result=mantissa_result; 
                end
            endcase

            sign = a[31] ^ b[31]; //Calcular signo
            result = {sign, exponent_sum[7:0], mantissa_result[25:3]}; // Concatenar resultado
        end
        
        Result = result;
    end

    assign negative = Result[31]; //MSB
    assign zero = (Result == 32'b0); //cero
    assign carry = 1'b0; //no aplica para fmul
    assign overflow = (exponent_sum < 0) | (exponent_sum > 254); //overflow
    //junto los flags
    assign ALUFlags = {negative, zero, carry, overflow};

endmodule




module mantissa_multiplier16 (
    input [10:0] mant_a,
    input [10:0] mant_b,
    output [21:0] mant_result
  );
  assign mant_result = mant_a * mant_b;
endmodule

module exponent_adder16 (
    input [4:0] exp_a,
    input [4:0] exp_b,
    output [4:0] exp_result
  );
  assign exp_result = exp_a + exp_b - 5'd15;
endmodule

module sign_handler16 (
    input sign_a,
    input sign_b,
    output sign_result
  );
  assign sign_result = sign_a ^ sign_b;
endmodule

module normalizer16 (
    input [21:0] mant_result,
    input [4:0] exp_result,
    output [9:0] normalized_mant,
    output [4:0] normalized_exp
  );
  wire [21:0] shifted_mant;
  wire [4:0] adjusted_exp;

  assign shifted_mant = mant_result >> 1;
  assign adjusted_exp = exp_result + 1;

  assign normalized_mant = mant_result[21] ? shifted_mant[20:11] : mant_result[19:10];
  assign normalized_exp = mant_result[21] ? adjusted_exp : exp_result;
endmodule



//David: 
module fpmul16(
    input [15:0] a, 
    input [15:0] b,
    output reg [15:0] Result,
    output [3:0] ALUFlags
);

    reg sign;
    wire [21:0] mantissa_mult; // (1 + 10) * (1 + 10)
    wire [10:0] mantissa_a, mantissa_b; // mantissa + hidden bit
    reg [10:0] mantissa_result;
    reg [4:0] exponent_sum; // 5 bits para el exponente
    reg [15:0] result;

    // Ver si exponente es cero, caso lo sea hidden bit es cero, caso contrario 1
    assign mantissa_a = (a[14:10] == 5'b00000) ? {1'b0, a[9:0]} : {1'b1, a[9:0]}; 
    assign mantissa_b = (b[14:10] == 5'b00000) ? {1'b0, b[9:0]} : {1'b1, b[9:0]}; 
    assign mantissa_mult = mantissa_a * mantissa_b; // Multiplicar mantissas

    always @(*) begin
        exponent_sum = (a[14:10] + b[14:10]) - 15; // -15 para remover bias
        sign = a[15] ^ b[15]; // Calcular signo

        // Normalizar y ajustar mantisa
        if (mantissa_mult[21]) begin // Si el primer bit es 1, shifteo y sumo al exponente
            mantissa_result = mantissa_mult[21:11];
            exponent_sum = exponent_sum + 1;
        end else begin
            mantissa_result = mantissa_mult[20:10];
        end

        // Checkear underflow y overflow
        if (exponent_sum < 0) begin 
            result = 16'b0; // Underflow todo cero
        end 
        else if (exponent_sum > 30) begin
            // Overflow: seteo a infinito (0 11111 0000000...)
            result = {sign, 5'b11111, 10'b0};
        end 
        else begin
            // Redondeo basado en los 3 bits menos significativos
            if (mantissa_mult[10] && (mantissa_mult[9:0] != 0 || mantissa_mult[11])) begin
                mantissa_result = mantissa_result + 1;
                // Ajustar por overflow del redondeo
                if (mantissa_result[10]) begin
                    mantissa_result = mantissa_result >> 1;
                    exponent_sum = exponent_sum + 1;
                end
            end

            result = {sign, exponent_sum[4:0], mantissa_result[9:0]};
        end
        
        Result = result;
    end

    assign negative = Result[15]; // MSB
    assign zero = (Result == 16'b0); // cero
    assign carry = 1'b0; // no aplica para fmul
    assign overflow = (exponent_sum < 0) | (exponent_sum > 30); // overflow
    // junto los flags
    assign ALUFlags = {negative, zero, carry, overflow};

endmodule
