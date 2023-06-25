`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2023/05/27 15:48:47
// Design Name: 
// Module Name: tb_pipeline
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

module tb_pipeline();
		// Inputs
		reg key;
		reg clk;
		reg reset;
		// Outputs
	    wire 	[ 7:0] digit;
	    wire 	[ 7:0] fnd;
	    wire 	[15:0] LED;
//		wire [31:0] pc;
//		wire [31:0] ins;
	
		pipeline uut (
//			.current_pc(pc), 
//			.instruction(ins), 
            .key(key),
			.clk(clk), 
			.reset(reset)
		);
	
    	initial begin
			reset =0;
			key =0;
			#54 reset = 1;
		end
	
		initial begin
			clk =0;
			forever #5 clk = ~clk;
		end
      
endmodule


