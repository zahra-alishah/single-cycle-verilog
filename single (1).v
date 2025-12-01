//===============================================================================
//
//			zahra_alishah (402102081)
//
//			Implemented Instructions are:
//			R format:  add(u), sub(u), and, or, xor, nor, slt, sltu;
//			I format:  beq, *bne, lw, sw, addi(u), *slti, *sltiu, *andi, *ori, *xori, *lui.
//
//===============================================================================

`timescale 1ns/1ns

   `define ADD  4'h0
   `define SUB  4'h1
   `define SLT  4'h2
   `define SLTU 4'h3
   `define AND  4'h4
   `define OR   4'h5
   `define NOR  4'h6
   `define XOR  4'h7
   `define LUI  4'h8

module single_cycle_mips 
(
	input clk,
	input reset
);
 
	initial begin
		$display("Single Cycle MIPS Implemention");
		$display("zahra_alishah (402102081)");
	end

	reg [31:0] PC;          // Keep PC as it is, its name is used in higher level test bench

   wire [31:0] instr;
   wire [ 5:0] op   = instr[31:26];       //opcode
   wire [ 5:0] func = instr[ 5: 0];       //function at the end of R-type

   wire [31:0] RD1, RD2, AluResult, MemReadData;

   wire AluZero;
   // wire Branch;

   // Control Signals

   wire PCSrc;

   reg SZEn, ALUSrc, RegDst, MemtoReg, RegWrite, MemWrite;


   reg [3:0] AluOP;

	
	// CONTROLLER COMES HERE

   always @(*) begin
      SZEn = 1'bx;
      AluOP = 4'hx;
      ALUSrc = 1'bx;
      RegDst = 1'bx;
      MemtoReg = 1'bx;
      RegWrite = 1'b0;
      MemWrite = 1'b0;
      // Branch = 1'b0;

      case(op)
         6'b000000 : begin            //R_type
            SZEn = 1'bx;
            // AluOP = instr[3:0];              //?????????????
            ALUSrc = 1'b0;
            RegDst = 1'b1;
            MemtoReg = 1'b0;
            RegWrite = 1'b1;
            MemWrite = 1'b0;
            // Branch = 1'b0;
            case(func)
               6'b100000 : AluOP = `ADD;
               6'b100001 : AluOP = `ADD;
               6'b100010 : AluOP = `SUB;
               6'b100011 : AluOP = `SUB;
               6'b100100 : AluOP = `AND;
               6'b100101 : AluOP = `OR;
               6'b100110 : AluOP = `XOR;
               6'b101010 : AluOP = `SLT;
               6'b101011 : AluOP = `SLTU;
               6'b100111 : AluOP = `NOR;
            endcase
         end
         6'b100011 : begin            //lw
            SZEn = 1'b1;
            AluOP = 4'h0;
            ALUSrc = 1'b1;
            RegDst = 1'b0;
            MemtoReg = 1'b1;
            RegWrite = 1'b1;
            MemWrite = 1'b0;
            // Branch = 1'b0;

         end
         6'b101011 : begin            //sw
            SZEn = 1'b1;               //*
            AluOP = 4'h0;
            ALUSrc = 1'b1;
            RegDst = 1'bx;
            MemtoReg = 1'bx;
            RegWrite = 1'b0;
            MemWrite = 1'b1;
            // Branch = 1'b0;

         end
         6'b000100 : begin
            SZEn = 1'b1;              //beq
            AluOP = 4'h1;
            ALUSrc = 1'b0;
            RegDst = 1'bx;
            MemtoReg = 1'bx;
            RegWrite = 1'b0;
            MemWrite = 1'b0;
            // Branch = 1'b1;
         end
         6'b001001 : begin            //addiu
            SZEn = 1'b1;              //1'b0???
            AluOP = 4'h0;
            ALUSrc = 1'b1;
            RegDst = 1'b0;
            MemtoReg = 1'b0;         //1'bx???
            RegWrite = 1'b1;
            MemWrite = 1'b0;
            // Branch = 1'b0;
         end
         6'b001000 : begin            //addi
            SZEn = 1'b1;
            AluOP = 4'h0;
            ALUSrc = 1'b1;
            RegDst = 1'b0;
            MemtoReg = 1'b0;         //1'bx???
            RegWrite = 1'b1;
            MemWrite = 1'b0;
            // Branch = 1'b0;
         end
         6'b001100 : begin           //andi
            SZEn = 1'b0;
            AluOP = 4'h4;
            ALUSrc = 1'b1;
            RegDst = 1'b0;
            MemtoReg = 1'b0;        //*
            RegWrite = 1'b1;         
            MemWrite = 1'b0;
            // Branch = 1'b0;
         end
         6'b001101 : begin           //ori
            SZEn = 1'b0;
            AluOP = 4'h5;
            ALUSrc = 1'b1;    
            RegDst = 1'b0;    
            MemtoReg = 1'b0;  
            RegWrite = 1'b1;         
            MemWrite = 1'b0;
            // Branch = 1'b0;
         end
         6'b001110 : begin           //xori
            SZEn = 1'b0;
            AluOP = 4'h7;
            ALUSrc = 1'b1;
            RegDst = 1'b0;
            MemtoReg = 1'b0;
            RegWrite = 1'b1;          
            MemWrite = 1'b0;
            // Branch = 1'b0;
         end
         6'b000101 : begin           //bne
            SZEn = 1'b1;
            AluOP = 4'h1;
            ALUSrc = 1'b0;
            RegDst = 1'bx;
            MemtoReg = 1'bx;
            RegWrite = 1'b0;
            MemWrite = 1'b0;
            // Branch = 1'b1;
         end
         6'b001111 : begin           //lui
            SZEn = 1'b0;
            AluOP = 4'h8;
            ALUSrc = 1'b1;
            RegDst = 1'b0;
            MemtoReg = 1'b0;        //*
            RegWrite = 1'b1;
            MemWrite = 1'b0;
            // Branch = 1'b0;
         end
         6'b001010 : begin           //slti
            SZEn = 1'b1;
            AluOP = 4'h2;
            ALUSrc = 1'b1;
            RegDst = 1'b0;
            MemtoReg = 1'b0;        //1'bx???
            RegWrite = 1'b1;
            MemWrite = 1'b0;
            // Branch = 1'b0;
         end
         6'b001011 : begin           //sltiu
            SZEn = 1'b0;
            AluOP = 4'h3;
            ALUSrc = 1'b1;
            RegDst = 1'b0;
            MemtoReg = 1'b0;     //*
            RegWrite = 1'b1;
            MemWrite = 1'b0;
            // Branch = 1'b0;
         end

         // 6'b000010 : begin            //j-type
         //    SZEn = 1'bx;
         //    AluOP = 4'h;
         //    ALUSrc = 1'bx;
         //    RegDst = 1'bx;
         //    MemtoReg = 1'bx;
         //    RegWrite = 1'b0;
         //    MemWrite = 1'b0;
         // end
      endcase

   end

   assign PCSrc = ( op == 6'b000100 && AluZero) || ( op == 6'b000101 && ~AluZero);


	// DATA PATH STARTS HERE

   wire [31:0] PCplus4 = PC + 4'h4;

   wire [31:0] WD = MemtoReg ? MemReadData : AluResult;

   wire [ 4:0] WR = RegDst ? instr[15:11] : instr[20:16];

   wire [31:0] Imm32 = SZEn ? {{16{instr[15]}},instr[15:0]} : {16'h0, instr[15:0]};
                    // ZSEn: 1 sign extend, 0 zero extend

   wire [31:0] PCbranch = PCplus4 + (Imm32 << 2);


   always @(posedge clk)
      if(reset)
         PC <= 32'h0;
      else
            PC <= PCSrc ? PCbranch : PCplus4;
      

//==========================================================//
//	instantiated modules
//==========================================================//

// Register File

   reg_file rf
   (
      .write ( RegWrite ),
      .RR1   ( instr[25:21] ),      //the address to read rs
      .RR2   ( instr[20:16] ),      //the address to read rt
      .*
	);

   my_alu alu
   (
      .Op( AluOP ),
      .A ( RD1 ),
      .B ( ALUSrc ? Imm32 : RD2),
      .X ( AluResult ),
      .Z ( AluZero )
   );
   
//	Instruction Memory
	async_mem imem			// keep the exact instance name
	(
		.clk		   (1'b0),
		.write		(1'b0),		// no write for instruction memory
		.address	   ( PC ),		   // address instruction memory with pc
		.write_data	(32'bx),
		.read_data	( instr )
	);
	
// Data Memory
	async_mem dmem			// keep the exact instance name
	(
		.clk		   ( clk ),
		.write		( MemWrite ),
		.address	   ( AluResult ),
		.write_data	( RD2 ),
		.read_data	( MemReadData )
	);

endmodule


//===============================================================================

module my_alu(
   input  [3:0] Op,
   input  [31:0] A,
   input  [31:0] B,
   output [31:0] X,
   output        Z
   );

   wire sub = Op != `ADD;
   wire [31:0] bb = sub ? ~B : B;
   wire [32:0] sum = A + bb + sub;
   wire sltu = ! sum[32];

   wire v = sub ?
            ( A[31] != B[31] && A[31] != sum[31] )
          : ( A[31] == B[31] && A[31] != sum[31] );

   wire slt = v ^ sum[31];

   reg [31:0] x;

   always @( * )
      case( Op )
         `ADD : x = sum;
         `SUB : x = sum;
         `SLT : x = slt;
         `SLTU: x = sltu;
         `AND : x = A & B;
         `OR  : x = A | B;
         `NOR : x = ~(A | B);
         `XOR : x = A ^ B;
         `LUI : x = {B[15:0], 16'h0};
         default : x = 32'hxxxxxxx;
      endcase

   assign X = x;
   assign Z = x == 32'h00000000;

endmodule

//===============================================================================

