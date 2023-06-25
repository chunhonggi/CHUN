`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2023/05/27 15:45:01
// Design Name: 
// Module Name: pipeline
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


module pipeline(
    input 	key,
    
	output 	[ 7:0] digit,
	output 	[ 7:0] fnd,
	output 	[15:0] LED,
    
    input clk, reset
    );
    wire c;
    wire [2:0] LED_clk;
    wire [31:0] pc, ins;
    wire ind_ctl_0, ind_ctl_1, ind_ctl_2, ind_ctl_3, ind_ctl_4, ind_ctl_5, ind_ctl_6, ind_ctl_7;
    wire exe_ctl_0, exe_ctl_1, exe_ctl_2, exe_ctl_3, exe_ctl_4, exe_ctl_5, exe_ctl_6, exe_ctl_7;
    wire mem_ctl_0, mem_ctl_1, mem_ctl_2, mem_ctl_3, mem_ctl_4, mem_ctl_5, mem_ctl_6, mem_ctl_7;
    wire wb_ctl_0, wb_ctl_1, wb_ctl_2, wb_ctl_3, wb_ctl_4, wb_ctl_5, wb_ctl_6, wb_ctl_7;
    
    wire[31:0] ind_pc, exe_pc, mem_pc;
    wire[31:0] ind_data1, ind_data2, ind_imm,   exe_data2, exe_addr, exe_result,    mem_addr, mem_result, mem_data,     wb_data;
    wire[4:0]  ind_rd, ind_Rs1, ind_Rs2,        exe_rd,                             mem_rd,                             wb_rd;
    wire[6:0] ind_funct7;
    wire[2:0] ind_funct3;
    wire       ind_jal, ind_jalr,               exe_jal, exe_jalr, exe_zero,        mem_jal, mem_jalr, mem_PCSrc  ;
    wire        stall;
    wire [1:0]  ForwardA;
    wire [1:0]  ForwardB;
    
	wire	[31:0] clk_address, clk_count;
	wire 	[31:0] data = (key)? mem_data : clk_count;
	wire 	[31:0] RAM_address = (key) ? (clk_address<<2) : exe_result;
	
	wire   rst;
	assign rst=~reset;
	
	assign LED =  (key) ? 16'b1000_0000_0000_0000 : 16'b0;
	
//////////////////////////////////////////////////////////////////////////////////////
LED_channel LED0(
	.data(data),							.digit(digit),
	.LED_clk(LED_clk),					.fnd(fnd));
//////////////////////////////////////////////////////////////////////////////////////
counter A0_counter(
	.key1(key),
	.mem_data(mem_data),
	.clk_address(clk_address),
	.indrs1(ind_Rs1),
	.inddata1(ind_data1),
	.clk(clk),								.LED_clk(LED_clk),
	.rst(rst),								.clk_out(c),
	.pc_in(pc),						        
	.clk_count_out(clk_count));
//////////////////////////////////////////////////////////////////////////////////////
    InFetch A1_InFetch(
        .PCSrc(mem_PCSrc),
        .PCimm_in(mem_addr),
        .PC_out(pc),
        .instruction_out(ins),
        .reset(rst),
        .clk(c),   
        .PCWrite(stall)
    );
    InDecode  A3_InDecode(
        .clk(c),
        .reset(rst),
        .Ctl_RegWrite_in(wb_ctl_5),
        .WriteReg(wb_rd),
        .PC_in(pc),
        .instruction_in(ins),
        .WriteData(wb_data),
        .stall(stall),
	    .flush(mem_PCSrc),
    
        .Ctl_ALUSrc_out(ind_ctl_7), 
	    .Ctl_MemtoReg_out(ind_ctl_6), 
	    .Ctl_RegWrite_out(ind_ctl_5), 
	    .Ctl_MemRead_out(ind_ctl_4), 
	    .Ctl_MemWrite_out(ind_ctl_3), 
	    .Ctl_Branch_out(ind_ctl_2), 
	    .Ctl_ALUOpcode1_out(ind_ctl_1), 
	    .Ctl_ALUOpcode0_out(ind_ctl_0),
	    .Rd_out(ind_rd),
	    .Rs1_out(ind_Rs1),
	    .Rs2_out(ind_Rs2),
	    .PC_out(ind_pc),
	    .ReadData1_out(ind_data1),
	    .ReadData2_out(ind_data2),
	    .Immediate_out(ind_imm),
	    .funct7_out(ind_funct7),
	    .funct3_out(ind_funct3),
	    .jalr_out(ind_jalr), 
	    .jal_out(ind_jal)
    );
	Execution A4_Execution(	
	   .ForwardA_in(ForwardA),
	   .ForwardB_in(ForwardB),
	   .jal_in(ind_jal),
	   .jalr_in(ind_jalr),
	   .clk(c),
	   .rst(rst),
	   .flush(mem_PCSrc),
	   .Ctl_ALUSrc_in(ind_ctl_7), 
	   .Ctl_MemtoReg_in(ind_ctl_6), 	
	   .Ctl_RegWrite_in(ind_ctl_5),
	   .Ctl_MemRead_in(ind_ctl_4), 
	   .Ctl_MemWrite_in(ind_ctl_3), 
	   .Ctl_Branch_in(ind_ctl_2), 
	   .Ctl_ALUOpcode1_in(ind_ctl_1), 
	   .Ctl_ALUOpcode0_in(ind_ctl_0),
	   .Rd_in(ind_rd),
	   .Immediate_in(ind_imm), 
	   .ReadData1_in(ind_data1), 
	   .ReadData2_in(ind_data2), 
	   .PC_in(ind_pc), 
	   .funct7_in(ind_funct7),
	   .funct3_in(ind_funct3), 
	   .mem_data(exe_result), 
	   .wb_data(wb_data),
					
	   .Ctl_MemtoReg_out(exe_ctl_6), 
	   .Ctl_RegWrite_out(exe_ctl_5), 
	   .Ctl_MemRead_out(exe_ctl_4),	
	   .Ctl_MemWrite_out(exe_ctl_3),	
	   .Ctl_Branch_out(exe_ctl_2),
	   .Zero_out(exe_zero),
	   .Rd_out(exe_rd),
	   .ALUresult_out(exe_result), 
	   .PCimm_out(exe_addr), 
	   .ReadData2_out(exe_data2),
	   .jal_out(exe_jal),
	   .jalr_out(exe_jalr),
	   .PC_out(exe_pc)
	);
    Memory  A6_Memory(
        .reset(rst),
        .clk(c),
        .Ctl_MemtoReg_in(exe_ctl_6),
        .Ctl_RegWrite_in(exe_ctl_5),
        .Ctl_MemRead_in(exe_ctl_4),
        .Ctl_MemWrite_in(exe_ctl_3),
        .Ctl_Branch_in(exe_ctl_2),
        .Rd_in(exe_rd),
        .Zero_in(exe_zero),
        .Write_Data(exe_data2),
        .ALUresult_in(RAM_address),
        .PCimm_in(exe_addr),
        .PC_in(exe_pc),
	    .jal_in(exe_jal),
	    .jalr_in(exe_jalr),
        
        .PC_out(mem_pc),
        .Ctl_MemtoReg_out(mem_ctl_6),
        .Ctl_RegWrite_out(mem_ctl_5),
        .Rd_out(mem_rd),
        .PCSrc(mem_PCSrc),
        .Read_Data(mem_data),
        .ALUresult_out(mem_result),
        .PCimm_out(mem_addr),
        .jal_out(mem_jal),
	    .jalr_out(mem_jalr)
    );  
    WB A7_WB(
        .Ctl_RegWrite_in(mem_ctl_5),
        .Ctl_MemtoReg_in(mem_ctl_6),
        .Rd_in(mem_rd),
        .ReadDatafromMem_in(mem_data),
        .ALUresult_in(mem_result),
        .jal_in(mem_jal),
	    .jalr_in(mem_jalr),
	    .PC_in(mem_pc),
        
        .Ctl_RegWrite_out(wb_ctl_5),
        .Rd_out(wb_rd),
        .WriteDataReg_out(wb_data)
    );  
    
    Hazard_detection_unit A9_Hazard_detection_unit(
    	.exe_Ctl_MemRead_in(ind_ctl_4),
	    .Rd_in(ind_rd),
	    .instruction_in(ins[24:15]),

	    .stall_out(stall)
    );
    Forwarding_unit A8_Forwarding_unit(
    	.mem_Ctl_RegWrite_in(exe_ctl_5),
	    .wb_Ctl_RegWrite_in(mem_ctl_5), 
	    .Rs1_in(ind_Rs1), 
	    .Rs2_in(ind_Rs2), 
	    .mem_Rd_in(exe_rd), 
	    .wb_Rd_in(mem_rd),
	    
	    .ForwardA_out(ForwardA), 
	    .ForwardB_out(ForwardB)
    );
    
endmodule

module  InFetch(
    input   clk, reset,
    input   PCSrc,
    input   [31:0] PCimm_in,
    input   PCWrite,            
    
    output  [31:0] instruction_out,
    output reg [31:0] PC_out
    );
    wire    [31:0] PC;
    wire    [31:0] PC4= (PCSrc) ?   PCimm_in:    PC+4;
    
    PC  B1_PC(
        .clk(clk),
        .reset(reset),
        .PCWrite(PCWrite),
        .PC_in(PC4),
        .PC_out(PC)
    );
    
    iMEM B2_iMEM(
        .clk(clk),
        .reset(reset),
        .IF_ID_Write(PCWrite),
        .PCSrc(PCSrc),
        .PC_in(PC),
        .instruction_out(instruction_out)
    );
    
    always@(posedge clk) begin
        if(reset||PCSrc)   PC_out <=0;
        else if (PCWrite)  PC_out <=PC_out;
        else               PC_out <=PC;
    end
endmodule

module iMEM(
    input   clk, reset,
    input   IF_ID_Write, PCSrc,
    input   [31:0] PC_in,
    output  reg[31:0] instruction_out
    );
    
    parameter   ROM_size=256;
    reg [31:0] ROM [0:ROM_size-1];
    integer i;
    initial begin
        for (i=0;   i!=ROM_size;    i=i+1) begin
            ROM[i]=32'b0;
        end
        $readmemh("darksocv.rom.mem", ROM);
    end 
    always @(posedge clk) begin
        if(!IF_ID_Write)    begin
            if(reset||PCSrc)   instruction_out <=32'b0;
            else               instruction_out <=ROM[PC_in[31:2]];
        end
    end
endmodule

module PC(
    input   clk, reset,
    input   PCWrite,
    input   [31:0] PC_in,
    output  reg [31:0] PC_out
    );
    always  @(posedge clk)  begin
        if      (reset)       PC_out <= 0;
        else if (PCWrite)     PC_out <= PC_out;
        else                  PC_out <= PC_in;
    end
endmodule


module InDecode(
	input clk, reset, stall, flush,
	input Ctl_RegWrite_in, 	
	
	input 		[ 4:0] WriteReg, 
	input 		[31:0] PC_in, instruction_in, WriteData,

	output reg Ctl_ALUSrc_out, Ctl_MemtoReg_out, Ctl_RegWrite_out, Ctl_MemRead_out, Ctl_MemWrite_out, Ctl_Branch_out, Ctl_ALUOpcode1_out, Ctl_ALUOpcode0_out,
	
	output reg 	[ 4:0] Rd_out, Rs1_out, Rs2_out,
	output reg 	[31:0] PC_out, ReadData1_out, ReadData2_out, Immediate_out,
	output reg  [ 6:0] funct7_out, 	// RISC-V
	output reg 	[ 2:0] funct3_out,	// RISC-V
	output reg         jalr_out, jal_out
	);	
	wire [ 6:0] opcode = instruction_in[6:0];
	wire [ 6:0] funct7 = instruction_in[31:25];
	wire [ 2:0] funct3 = instruction_in[14:12];
	wire [ 4:0] Rd     = instruction_in[11:7];
	wire [ 4:0] Rs1    = instruction_in[19:15];
	wire [ 4:0] Rs2    = instruction_in[24:20];
	wire [ 7:0] Ctl_out;
	wire        jal    = (opcode==7'b11011_11)?1:0;
	wire        jalr   = (opcode==7'b11001_11)?1:0;
	
	Control_unit B0 (.opcode(opcode),  .reset(reset), .Ctl_out(Ctl_out));
	
	reg [7:0] Control;
	always @(*) begin
	   Control = (flush)   ?   0    :
	             (stall)   ?   0    :   Ctl_out;
	end
	
				
	parameter reg_size = 32;
	reg [31:0] Reg[0:reg_size-1]; //32bit reg
	integer i;
	always@(posedge clk) begin
		if (reset) begin
			for(i=0;i<32;i=i+1) begin
				Reg[i] <= 32'b0;
			end
			
		end
		if(Ctl_RegWrite_in && WriteReg!=0) Reg[WriteReg] <= WriteData;

   end
	
	reg [31:0] Immediate;
	always@(*) begin
		case(opcode)
			7'b00000_11: Immediate	= $signed(instruction_in[31:20]); // I-type 
			7'b00100_11: Immediate	= $signed(instruction_in[31:20]); // I-type 
			7'b11001_11: Immediate	= $signed(instruction_in[31:20]); // I-type jalr ????
			7'b01000_11: Immediate	= $signed({instruction_in[31:25],instruction_in[11:7]}); // S-type
			7'b11000_11: Immediate	= $signed({instruction_in[31],instruction_in[7],instruction_in[30:25],instruction_in[11:8]}); // SB-type
			7'b11011_11: Immediate	= $signed({instruction_in[31],instruction_in[19:12],instruction_in[20],instruction_in[30:21]}); // jal    
			default: 	 Immediate	= 32'b0;
		endcase
	end
	
	always@(posedge clk) begin
		// RISC-V		
		PC_out 			     <=(reset||stall)? 0 :	PC_in;
		funct7_out 		     <=(reset||stall)? 0 : funct7;
		funct3_out 		     <=(reset||stall)? 0 : funct3;
		Rd_out 			     <=(reset||stall)? 0 : Rd;
		Rs1_out 			 <=(reset||stall)? 0 : Rs1;
		Rs2_out 			 <=(reset||stall)? 0 : Rs2;
		ReadData1_out 	     <=(reset||stall)? 0 : (Ctl_RegWrite_in && (WriteReg==Rs1)) ? WriteData : Reg[Rs1];
		ReadData2_out	     <=(reset||stall)? 0 : (Ctl_RegWrite_in && (WriteReg==Rs2)) ? WriteData : Reg[Rs2];
		jalr_out 		     <=(reset||stall)? 0 : jalr;
		jal_out 			 <=(reset||stall)? 0 : jal;
		Ctl_ALUSrc_out 		 <=(reset||stall)? 0 : Control[7];
		Ctl_MemtoReg_out 	 <=(reset||stall)? 0 : Control[6];
		Ctl_RegWrite_out 	 <=(reset||stall)? 0 : Control[5];
		Ctl_MemRead_out 	 <=(reset||stall)? 0 : Control[4];
		Ctl_MemWrite_out 	 <=(reset||stall)? 0 : Control[3];
		Ctl_Branch_out		 <=(reset||stall)? 0 : Control[2];
		Ctl_ALUOpcode1_out 	 <=(reset||stall)? 0 : Control[1];
		Ctl_ALUOpcode0_out 	 <=(reset||stall)? 0 : Control[0];
		Immediate_out		 <=(reset||stall)? 0 : Immediate;
	end
	
endmodule

module Control_unit(
	input [6:0] opcode,
	input reset,
	output reg [7:0] Ctl_out
	);

	always @(*) begin	
		if (reset)
			Ctl_out = 8'b0;
		else
			case(opcode)							
				7'b01100_11 : Ctl_out = 8'b001000_10;	// R-type	addi, slli									
				7'b00100_11 : Ctl_out = 8'b101000_11;	// I-type   lw 
				7'b00000_11 : Ctl_out = 8'b111100_00;	// I-type   lxx     rd, rs1,imm[11:0]	
				7'b01000_11 : Ctl_out = 8'b100010_00;	// S-type   sxx     rs1,rs2,imm[11:0]				 
				7'b11000_11 : Ctl_out = 8'b000001_01;	// SB-type  beq     rs1,rs2,imm[12:1]
				7'b11011_11 : Ctl_out = 8'b001001_00;	// UJ-type  jal	    rd, imm[20:1] 	
				7'b11001_11 : Ctl_out = 8'b101001_11;	// I-type   jalr    rd, rs1,imm[11:0] 	
				
				default 		: Ctl_out = 8'b0; 
			endcase
	end
endmodule

module Execution(
	input 	    clk, rst, 
	input 		Ctl_ALUSrc_in, Ctl_MemtoReg_in, 	Ctl_RegWrite_in, Ctl_MemRead_in, Ctl_MemWrite_in, Ctl_Branch_in, Ctl_ALUOpcode1_in, Ctl_ALUOpcode0_in,
	input 		[ 4:0] Rd_in,

	input 		[31:0] Immediate_in, ReadData1_in, ReadData2_in, PC_in, 
	input 		[31:0] mem_data, wb_data,
	input 		[ 6:0] funct7_in, 
	input 		[ 2:0] funct3_in, 
	
    input 	    flush,
    input 		[ 1:0] ForwardA_in, ForwardB_in,
    input		jal_in, jalr_in,

	output reg	Ctl_MemtoReg_out, Ctl_RegWrite_out, Ctl_MemRead_out,	Ctl_MemWrite_out,	Ctl_Branch_out, 
	output reg 	[ 4:0] Rd_out,
	output reg	Zero_out,
	
	output reg 	[31:0] ALUresult_out, PCimm_out, ReadData2_out, PC_out,
	output reg	jal_out, jalr_out
	);
	
	wire [3:0] ALU_ctl;
	wire [31:0] ALUresult;
	wire   zero;
	
    wire [31:0] ALU_input1= 
                            (ForwardA_in == 2'b10)? mem_data:  
                            (ForwardA_in == 2'b01)? wb_data:
							                        ReadData1_in;
	wire [31:0] ForwardB_input =  
	                       	(ForwardB_in == 2'b10)? mem_data: 
	                        (ForwardB_in == 2'b01)? wb_data:
	                                                ReadData2_in;		                        
	wire [31:0] ALU_input2 = 
	                        (Ctl_ALUSrc_in) ? Immediate_in: 
	                                          ForwardB_input;
	ALU_control B0 (.ALUop({Ctl_ALUOpcode1_in, Ctl_ALUOpcode0_in}), .funct7(funct7_in), .funct3(funct3_in), .ALU_ctl(ALU_ctl));
	ALU B1 (.ALU_ctl(ALU_ctl),.in1(ALU_input1),.in2(ALU_input2),.out(ALUresult),.zero(zero));

	always @(posedge clk)   begin
		  Ctl_MemtoReg_out	<= (rst||flush)? 0 : Ctl_MemtoReg_in;
		  Ctl_RegWrite_out	<= (rst||flush)? 0 : Ctl_RegWrite_in;
		  Ctl_MemRead_out	<= (rst||flush)? 0 : Ctl_MemRead_in;
		  Ctl_MemWrite_out	<= (rst||flush)? 0 : Ctl_MemWrite_in;
		  Ctl_Branch_out	<= (rst||flush)? 0 : Ctl_Branch_in;	
		
		  PC_out	 <= (rst) ? 1'b0 : PC_in;
		  jalr_out	 <= (rst) ? 1'b0 : jalr_in;
		  jal_out	 <= (rst) ? 1'b0 : jal_in;
		
		  Rd_out				<= Rd_in;
		  PCimm_out			    <= PC_in+((Immediate_in<<1));
		  ReadData2_out		    <= (rst) ? 0 : ForwardB_input;
		  ALUresult_out		    <= ALUresult; 
		  Zero_out				<= (rst)? 0: zero;
	end
endmodule

module ALU_control(
	input [1:0] ALUop,
	input [6:0] funct7,
	input [2:0] funct3,
	output reg [3:0] ALU_ctl
	);
	
	
	always @(*) begin
		casex ({ALUop,funct3,funct7})
			12'b00_xxx_xxxxxxx :	ALU_ctl	=	4'b0010;	// lb, lh, lw, sb, sh, sw 	=> ADDITION
			12'b01_00x_xxxxxxx :	ALU_ctl	=	4'b0110;	// beq, bne 			    => SUBTRACT (funct3==3'b000)	||	(funct3==3'b001)
			12'b01_100_xxxxxxx :	ALU_ctl	=	4'b0111;	// blt						=> BLT      (funct3==3'b100)
			12'b01_101_xxxxxxx :	ALU_ctl	=	4'b1000;	// bge						=> BGE      (funct3==3'b101)
			12'b10_000_0000000 :	ALU_ctl	=	4'b0010;	// add						=> ADDITION (funct3==3'b000 && funct7==7'b0000000)
			12'b10_000_0100000 :	ALU_ctl	=	4'b0110;	// sub						=> SUBTRACT (funct3==3'b000 && funct7==7'b0100000)
			12'b10_111_0000000 :	ALU_ctl	=	4'b0000;	// and						=> AND      (funct3==3'b111 && funct7==7'b0000000)
			12'b10_110_0000000 :	ALU_ctl	=	4'b0001;	// or						=> OR       (funct3==3'b110 && funct7==7'b0000000)
			12'b10_001_xxxxxxx :	ALU_ctl	=	4'b1001;	// sll					    => SHIFT_LEFT  (funct3==3'b001)
			12'b10_101_xxxxxxx :	ALU_ctl	=	4'b1010;	// srl						=> SHIFT_RIGHT (funct3==3'b101)
			12'b11_000_xxxxxxx :	ALU_ctl	=	4'b0010;	// addi, jalr				=> ADDITION (funct3==3'b000)
			12'b11_111_xxxxxxx :	ALU_ctl	=	4'b0000;	// andi						=> AND      (funct3==3'b111)	
			12'b11_001_xxxxxxx :	ALU_ctl	=	4'b1001;	// slli						=> SHIFT_LEFT  (funct3==3'b001)
			12'b11_101_xxxxxxx :	ALU_ctl	=	4'b1010;	// srli						=> SHIFT_RIGHT (funct3==3'b101)
			default : ALU_ctl = 4'bx;
		endcase
	end									
endmodule
module ALU(
	input [3:0] ALU_ctl,
	input [31:0] in1, in2,
	output reg [31:0] out,
	output zero
	);
	
	always @(*) begin
		case (ALU_ctl)
			4'b0000 :	out = in1 & in2;	                // and
			4'b0001 :	out = in1 | in2;					// or
			4'b0010 :	out = in1 + in2;					// add
			4'b0110 :	out = in1 - in2;					// sub
			4'b0111 :	out = (in1 < in2) ? 0:1;	        // blt (branch if less than)
			4'b1000 :	out = (in1 >= in2)? 0:1;	        // bge (branch if greater equal)
			4'b1100 :	out = ~(in1 | in2);				    // nor
			4'b1001 :	out = in1 << in2;					// shift left
			4'b1010 :	out = in1 >> in2;					// shift right
			default :	out = 32'b0;
		endcase
	end
						
	assign zero = 	~|out;	
endmodule						

module Memory(
    input reset, clk,
    input Ctl_MemtoReg_in, Ctl_RegWrite_in, Ctl_MemRead_in, Ctl_MemWrite_in, Ctl_Branch_in,
    input [4:0] Rd_in,
    input Zero_in,
    input [31:0] Write_Data, ALUresult_in, PCimm_in, PC_in,
 	input jal_in, jalr_in,
    
    output reg Ctl_MemtoReg_out, Ctl_RegWrite_out, 
    output reg [4:0] Rd_out,
    output     PCSrc,
    
    output reg [31:0] Read_Data, ALUresult_out,
    output     [31:0] PCimm_out,
 	output reg jal_out, jalr_out, 
 	output reg	[31:0] PC_out
    );
    reg [31:0] mem [1024:0];

    wire    branch;
    or(branch, jalr_in, jal_in, Zero_in);
    and(PCSrc, Ctl_Branch_in, branch);
    
//    parameter   RAM_size=1024;
//    reg [15:0] RAM [0:RAM_size-1];
    integer j;
    initial begin
        for (j=0;   j!=1024;    j=j+1) begin
            mem[j]=32'b0;
        end
        $readmemh("darksocv.ram.mem", mem);
    end 
    
    always @(posedge clk, posedge reset)    begin
        if (Ctl_MemWrite_in)
            mem[ALUresult_in>>2]    <= Write_Data;
        else if (reset)
            Read_Data <= 0;
        else 
            Read_Data <= mem[ALUresult_in>>2];
    end
              
          
    always @(posedge clk) begin
        Ctl_MemtoReg_out <= (reset) 	? 1'b0 : Ctl_MemtoReg_in;
        Ctl_RegWrite_out <= (reset) 	? 1'b0 : Ctl_RegWrite_in;
        Rd_out           <= (reset) 	? 1'b0 : Rd_in;
        ALUresult_out    <= (reset) 	? 1'b0 : ALUresult_in;      		
		PC_out           <= (reset) 	? 1'b0 : PC_in;
		jalr_out         <= (reset)	    ? 1'b0 : jalr_in;
		jal_out          <= (reset) 	? 1'b0 : jal_in;
    end
    assign PCimm_out = (jalr_in)    ?   ALUresult_in:   PCimm_in;
endmodule

module WB(
	// control signal
	input 		Ctl_RegWrite_in, 	Ctl_MemtoReg_in,
	input 		[31:0] PC_in,
	input 		[ 4:0] Rd_in,
	input 		[31:0] ReadDatafromMem_in, ALUresult_in,
	input 		jal_in, jalr_in,
	output reg 	Ctl_RegWrite_out, 
	output reg 	[ 4:0] Rd_out,
	output reg 	[31:0] WriteDataReg_out
	);	

	always @(*) begin 
		Ctl_RegWrite_out		<= Ctl_RegWrite_in;
		Rd_out					<= Rd_in;
		
		if (Ctl_MemtoReg_in)  WriteDataReg_out	 <= ReadDatafromMem_in;
	    else if (jalr_in || jal_in)    WriteDataReg_out    <=  PC_in+4;
	    else   begin
	       WriteDataReg_out			 <= ALUresult_in;
	    end
	end 
endmodule

module Forwarding_unit(
    input mem_Ctl_RegWrite_in, wb_Ctl_RegWrite_in,
    input [4:0] Rs1_in, Rs2_in, mem_Rd_in, wb_Rd_in,
    output [1:0] ForwardA_out, ForwardB_out
    );
    
    assign ForwardA_out = (mem_Ctl_RegWrite_in && (mem_Rd_in == Rs1_in)) ? 2'b10: (wb_Ctl_RegWrite_in && (wb_Rd_in == Rs1_in)) ? 2'b01: 2'b00;
    assign ForwardB_out = (mem_Ctl_RegWrite_in && (mem_Rd_in == Rs2_in)) ? 2'b10: (wb_Ctl_RegWrite_in && (wb_Rd_in == Rs2_in)) ? 2'b01: 2'b00;
    
endmodule

module Hazard_detection_unit(
    input exe_Ctl_MemRead_in, // ld instruction
    input [4:0] Rd_in,
    input [9:0] instruction_in,
    output stall_out
    );
    
    wire [4:0] Rs1_in = instruction_in[4:0];
    wire [4:0] Rs2_in = instruction_in[9:5];
    
    assign stall_out = (exe_Ctl_MemRead_in && (Rd_in == Rs1_in || Rd_in == Rs2_in));  
endmodule

module LED_channel(
	input [31:0] data,
	input [2:0] LED_clk,
	output reg [7:0] digit,
	output reg [7:0] fnd
    );
    reg [3:0] segment;
	always@(*) begin
		case(segment)
          4'b0001 : fnd = ~8'b01111001;   // 1
          4'b0010 : fnd = ~8'b00100100;   // 2
          4'b0011 : fnd = ~8'b00110000;   // 3
          4'b0100 : fnd = ~8'b00011001;   // 4
          4'b0101 : fnd = ~8'b00010010;   // 5
          4'b0110 : fnd = ~8'b00000010;   // 6
          4'b0111 : fnd = ~8'b01111000;   // 7
          4'b1000 : fnd = ~8'b00000000;   // 8
          4'b1001 : fnd = ~8'b00010000;   // 9 
          4'b1010 : fnd = ~8'b00001000;   // A
          4'b1011 : fnd = ~8'b00000011;   // b
          4'b1100 : fnd = ~8'b01000110;   // C
          4'b1101 : fnd = ~8'b00100001;   // d
          4'b1110 : fnd = ~8'b00000110;   // E
          4'b1111 : fnd = ~8'b00001110;   // F
          default : fnd = ~8'b01000000;   // 0
		endcase
	end

	always@(*) begin
	   case(LED_clk)
	   	3'b000 : begin
	   		digit = 8'b0000_0001;
			segment = data[3:0];
		end
		3'b001 : begin
	   		digit = 8'b0000_0010;
			segment = data[7:4];
		end
		3'b010 : begin
	   		digit = 8'b0000_0100;
			segment = data[11:8];
		end
		3'b011 : begin
	   		digit = 8'b0000_1000;
			segment = data[15:12];
		end
		3'b100 : begin
	   		digit = 8'b0001_0000;
			segment = data[19:16];
		end
		3'b101 : begin
	   		digit = 8'b0010_0000;
			segment = data[23:20];
		end
		3'b110 : begin
	   		digit = 8'b0100_0000;
			segment = data[27:24];
		end
		3'b111 : begin
	   		digit = 8'b1000_0000;
			segment = data[31:28];
		end
		default : begin
	   		digit = 8'b1111_1111;
			segment = 4'b1111;
		end
	   endcase
	end
endmodule

module counter(
	input 			 clk, rst,
	input 	[31:0] pc_in, 
	input					 key1,
	input	[31:0] mem_data,
    input  [4:0]  indrs1,
	input  [31:0] inddata1,
	output	reg [31:0] clk_address,
	output 	[ 2:0] LED_clk,
	output 			 clk_out,
	output	reg [31:0] clk_count_out
    );
	reg [31:0]count = 0;
	reg [31:0]led_count = 0;
	reg count_flag;
	
	wire   clk_check;
	
	assign clk_out = count[0]; //count[24];
	assign clk_check = count[0];

    assign LED_clk = led_count[2:0];
   
    
	always @(posedge clk) begin
		led_count <= led_count + 32'b1;
		count <= count + 32'b1;
	end
	
	always @(posedge clk_out) begin
		if(rst) begin
			clk_count_out <=  32'b0;
			count_flag <= 1'b0;
		end
		else begin
		    if((indrs1 == 5'd31)&&(inddata1 == 32'd400))
                count_flag <= 1'b1;
            if(!count_flag)
                clk_count_out <= clk_count_out + 32'b1;
		end
			
	end
	
	always@(posedge clk_check) begin
		if(key1)
		  clk_address <= 32'd2;
	end
endmodule


