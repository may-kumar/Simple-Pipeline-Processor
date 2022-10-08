//***************************************INSTRUCTION MEMORY(USED BY IF)***************************************
module Instruction_Memory(
    input [31:0] PC,
    input reset,
    output [31:0] Instruction_Code
);
reg [7:0] Mem [35:0];  //Byte addresssable memory 36 locations

//For normal memory read we use the following statement 
assign Instruction_Code= {Mem[PC+3],Mem[PC+2],Mem[PC+1],Mem[PC]};
//Reads Instruction Code specified by PC in Little Endian

//Handling reset condition
always @(reset)
    begin
    
    if(reset == 0)//reset logic is 0
        begin //Initialize memory with 4 instructions
        //Mem[3]=8'h01; Mem[2]=8'h49; Mem[1]=8'h83; Mem[0]=8'h33;
        //Contains add t1, s3, s4
        
        //Mem[7]=8'h00; Mem[6]=8'h53; Mem[5]=8'h03; Mem[4]=8'h13;
        //Contains addi t1, t1, 5
        
        //Mem[11]=8'h00; Mem[10]=8'h04; Mem[9]=8'h22; Mem[8]=8'h33;
        //Contains lw t0, 0(s0)
        
        //Mem[15]=8'h00; Mem[14]=8'h62; Mem[13]=8'h82; Mem[12]=8'hb3;
        //Contains add t0,t0,t1
        $readmemh("Instruction.mem", Mem);
        end
    end
endmodule


//***************************************INSTRUCTION FETCH***************************************
module Instruction_Fetch(
    input clk,
    input reset,
    output [31:0] Instruction_Code
);

reg [31:0] PC;

//Initiate Instruction Memory here
Instruction_Memory instr_mem(PC, reset, Instruction_Code);

always @(posedge clk, negedge reset)
begin
    //if reset = 0, restart PC
    if(reset==0)
        PC<=0;
    else
        PC<=PC+4;
end

endmodule

//***************************************INSTRUCTION DECODE***************************************
module Instruction_Decode(
  input [31:0] Instruction_Code,
  output [5:0] Opcode,
  output [4:0] Rdst,Rsrc1,Rsrc2,Shamt,
  output [5:0] Function,
  output [20:0] Constant);
  
  assign Opcode = Instruction_Code[31:26];
  assign Rdst = Instruction_Code[25:21];
  assign Rsrc1 = Instruction_Code[20:16];
  assign Rsrc2 = Instruction_Code[15:11];
  assign Shamt = Instruction_Code[10:6];
  assign Function = Instruction_Code[5:0];
  assign Constant = Instruction_Code[20:0];
endmodule


//***************************************SIGN EXTEND***************************************
module Sign_Extend(
    input [20:0] In,
    output [31:0] Out);
    reg [10:0] extend;

    always@(In)
    begin
        if(In[20]==1'b1)
            extend = 11'b11111111111;
        else
            extend = 11'b00000000000;
    end

    assign Out = {extend,In};
endmodule

//***************************************CONTROL UNIT***************************************
module Control_Unit(
    input [5:0] Opcode,Function,
    output [3:0] ALUOp,
    output Shift_Sel,WB_Sel
);
  
    assign ALUOp = {Function[5],Function[2:0]};
    assign Shift_Sel = Function[5];
    assign WB_Sel = Opcode[5];

endmodule 

//***************************************REGISTER FILE***************************************
module Register_file(
    input [4:0] Read_Reg_Num_1,
    input [4:0] Read_Reg_Num_2,
    input [4:0] Write_Reg_Num,
    input [31:0] Write_Data,
    input RegWrite,
    input clk,
    input reset,
    
    output [31:0] Read_Data_1,
    output [31:0] Read_Data_2
);

    reg [31:0] RegMemory [31:0];

    //Handling reset condition
    always @(reset)
        begin
        
        if(reset == 0)//reset logic is 0
            begin 
            $readmemh("registerCon.mem", RegMemory);
            end
        end
    
    //always @(Read_Reg_Num_1)
        assign Read_Data_1 = RegMemory[Read_Reg_Num_1];  
        
    //always @(Read_Reg_Num_2)
        assign Read_Data_2 = RegMemory[Read_Reg_Num_2];  
        
    //assuming at positive edge new instruction is loaded, and control signals are ready
    //negative edge is when data is laoded into registers
    always @(negedge clk)
        begin
        if(RegWrite == 1)
            RegMemory[Write_Reg_Num] = Write_Data;  
        
        end

endmodule




//***************************************32 BIT MUX***************************************
module mux_2x1(
  input [31:0] in1,in2,
  input select,
  output reg [31:0] out);
  
  always@(in1, in2)
  begin
    if(select == 1'b0)
      out = in1;
    else
      out = in2;
  end
endmodule




//***************************************ALU UNIT***************************************
module ALU(
	input signed [31:0]A,
	input signed [31:0]B,
	input [3:0]control,
	output reg zero_val,
	output reg signed [31:0]out
	);
    
	always@(A or B or control)
	begin
	case(control)
        4'b1000: out = A+B;
        4'b1010: out = A-B;
        4'b1100: out = A&B;
        4'b1101: out = A|B;
        4'b0000: out = A<<B;
        4'b0010: out = A>>B;
        default: out = 0;
	endcase
    end
    
    
    always@(out)
    begin
        if(out == 0)
            zero_val = 1;
        else
            zero_val = 0;
    end
endmodule


//***************************************MAIN PROCESSOR IMPLEMENTATION***************************************

module processor(
    input clk,
    input reset
);

    wire [31:0] Instruction_Code;
    
    wire [5:0] OpCode;
    wire [4:0] Rdst, Rsrc1, Rsrc2, Shamt;
    wire [5:0] Function;
    wire [20:0] Constant;
    
    wire [31:0] SEConstant;
    wire [31:0] EShamt;
    
    wire [31:0] Read_Data_1, Read_Data_2;
    wire Shift_Sel,WB_Sel;
    
    wire zero_val;
    wire [31:0] ALU_In2, ALU_Out;
    wire [3:0] ALUOp;
    
    wire [31:0] WB_output;
    //opcode is always 31:26
    //write register is always 25:21
    //IF OPCODE IS 111111 -> then load 20:0 into Rd
    //IF OPCODE IS 000000 -> then folow function code

    Instruction_Fetch IF(clk, reset, Instruction_Code);
    
    Instruction_Decode ID(
        Instruction_Code,
        OpCode,
        Rdst,
        Rsrc1,
        Rsrc2,
        Shamt,
        Function,
        Constant
    );
    
    //Main Control Unit
    Control_Unit CU(OpCode,Function,ALUOp,Shift_Sel,WB_Sel);
    
    //Extend values to 32 bit
    assign EShamt = {27'b0, Shamt};
    Sign_Extend SE(Constant, SEConstant);
    
    //Reg stage
    Register_file RF(
        Rsrc1,
        Rsrc2,
        Rdst,
        WB_output,
        1'b1,
        clk,
        reset,
        Read_Data_1,
        Read_Data_2
    );
    
    
    //To select rs2 or shift immediate
    mux_2x1 shift_sel_mux(EShamt, Read_Data_2, Shift_Sel, ALU_In2);
    
    //ALU stage
    ALU EX(Read_Data_1, ALU_In2, ALUOp, zero_val, ALU_Out);
    
    
    //Writeback
    mux_2x1 wb_sel_mux(ALU_Out, SEConstant, WB_Sel, WB_output);
    
endmodule


module tb_processor;
reg clk;
reg reset;


    processor uut(clk, reset);
    //Get the clock up
    initial
    begin  
        clk = 0;
        forever #10 clk = ~clk;
    end

    
    //reset a few times 
    initial
    begin  
        reset = 0;
        #12 reset = 1;
        #250 reset = 0;
        #10 reset = 1;
    end


endmodule

//FC000046          = li r0,46h
//FC2BD102          = li r1,BD102h
//00410020          = add r2,r1,r0
//00620822          = sub r3,r2,r1
//00831024          = and r4,r3,r2 
//00A41825          = or r5,r4,r3
//00C500C0          = sll r6,r5,3
//00E60202          = srl r7,r6,8