// Top module of your design, you cannot modify this module!!
module CHIP (	clk,
				rst_n,
//----------for slow_memD------------
				mem_read_D,
				mem_write_D,
				mem_addr_D,
				mem_wdata_D,
				mem_rdata_D,
				mem_ready_D,
//----------for slow_memI------------
				mem_read_I,
				mem_write_I,
				mem_addr_I,
				mem_wdata_I,
				mem_rdata_I,
				mem_ready_I,
//----------for TestBed--------------				
				DCACHE_addr, 
				DCACHE_wdata,
				DCACHE_wen,
				PC   
			);
input			clk, rst_n;
//--------------------------

output			mem_read_D;
output			mem_write_D;
output	[31:4]	mem_addr_D;
output	[127:0]	mem_wdata_D;
input	[127:0]	mem_rdata_D;
input			mem_ready_D;
//--------------------------
output			mem_read_I;
output			mem_write_I;
output	[31:4]	mem_addr_I;
output	[127:0]	mem_wdata_I;
input	[127:0]	mem_rdata_I;
input			mem_ready_I;
//----------for TestBed--------------
output	[29:0]	DCACHE_addr;
output	[31:0]	DCACHE_wdata;
output			DCACHE_wen;
output	[31:0]	PC;
//--------------------------

// wire declaration
wire        ICACHE_ren;
wire        ICACHE_wen;
wire [29:0] ICACHE_addr;
wire [31:0] ICACHE_wdata;
wire        ICACHE_stall;
wire [31:0] ICACHE_rdata;

wire        DCACHE_ren;
wire        DCACHE_wen;
wire [29:0] DCACHE_addr;
wire [31:0] DCACHE_wdata;
wire        DCACHE_stall;
wire [31:0] DCACHE_rdata;
wire [31:0] PC;


//=========================================
	// Note that the overall design of your RISCV includes:
	// 1. pipelined RISCV processor
	// 2. data cache
	// 3. ini_instuction cache


	RISCV_Pipeline i_RISCV(
		// control interface
		.clk            (clk)           , 
		.rst_n          (rst_n)         ,
//----------I cache interface-------		
		.ICACHE_ren     (ICACHE_ren)    ,
		.ICACHE_wen     (ICACHE_wen)    ,
		.ICACHE_addr    (ICACHE_addr)   ,
		.ICACHE_wdata   (ICACHE_wdata)  ,
		.ICACHE_stall   (ICACHE_stall)  ,
		.ICACHE_rdata   (ICACHE_rdata)  ,
//----------D cache interface-------
		.DCACHE_ren     (DCACHE_ren)    ,
		.DCACHE_wen     (DCACHE_wen)    ,
		.DCACHE_addr    (DCACHE_addr)   ,
		.DCACHE_wdata   (DCACHE_wdata)  ,
		.DCACHE_stall   (DCACHE_stall)  ,
		.DCACHE_rdata   (DCACHE_rdata)	,
//--------------PC-----------------
		.PC(PC)
	);
	
	Dcache D_cache(
        .clk        (clk)         ,
        .proc_reset (~rst_n)      ,
        .proc_read  (DCACHE_ren)  ,
        .proc_write (DCACHE_wen)  ,
        .proc_addr  (DCACHE_addr) ,
        .proc_rdata (DCACHE_rdata),
        .proc_wdata (DCACHE_wdata),
        .proc_stall (DCACHE_stall),
        .mem_read   (mem_read_D)  ,
        .mem_write  (mem_write_D) ,
        .mem_addr   (mem_addr_D)  ,
        .mem_wdata  (mem_wdata_D) ,
        .mem_rdata  (mem_rdata_D) ,
        .mem_ready  (mem_ready_D)
	);

	Icache I_cache(
        .clk        (clk)         ,
        .proc_reset (~rst_n)      ,
        .proc_read  (ICACHE_ren)  ,
        .proc_write (ICACHE_wen)  ,
        .proc_addr  (ICACHE_addr) ,
        .proc_rdata (ICACHE_rdata),
        .proc_wdata (ICACHE_wdata),
        .proc_stall (ICACHE_stall),
        .mem_read   (mem_read_I)  ,
        .mem_write  (mem_write_I) ,
        .mem_addr   (mem_addr_I)  ,
        .mem_wdata  (mem_wdata_I) ,
        .mem_rdata  (mem_rdata_I) ,
        .mem_ready  (mem_ready_I)
	);
endmodule
//---------------------------------------------------code start here--------------------------------------------------
module RISCV_Pipeline (
//----------control interface-------
	clk                             , 
	rst_n                           ,
//----------I cache interface-------		
	ICACHE_ren                      ,
	ICACHE_wen                      ,
	ICACHE_addr                     ,
	ICACHE_wdata                    ,
	ICACHE_stall                    ,
	ICACHE_rdata                    ,
//----------D cache interface-------
	DCACHE_ren                      ,
	DCACHE_wen                      ,
	DCACHE_addr                     ,
	DCACHE_wdata                    ,
	DCACHE_stall                    ,
	DCACHE_rdata                 	,
//--------------PC-----------------
	PC
);
	input         clk;
	input         rst_n;
	output        ICACHE_ren, ICACHE_wen;
	output [29:0] ICACHE_addr;
	output [31:0] ICACHE_wdata;
	input         ICACHE_stall;
	input  [31:0] ICACHE_rdata;
	output		  DCACHE_ren, DCACHE_wen;
	output [29:0] DCACHE_addr ;
	output [31:0] DCACHE_wdata;
	input	      DCACHE_stall;
	input  [31:0] DCACHE_rdata;
	output [31:0] PC;
	// -----------------
	wire stall;
    wire PreWrong;
    wire BrPre_IF;
    reg  BrPre_ID;
	wire flush;

	wire [1:0] PCoff;
	wire [31:0] PC_w, PC4_IF, PCimm_ID;
	reg [31:0] PC_IF;
	wire load_hazard;
	wire branch_hazard;
    wire [31:0] inst_IF;
    wire [31:0] immB_IF;
    wire [31:0] BPU_PC_IF;
    wire [31:0] imm_IF;
    wire [6:0]  op_IF;
    wire B_IF;
	wire DEC_stall;

    reg [31:0] BPU_PC_ID;
	reg [31:0] inst_ID;
	wire [4:0] rs1_ID, rs2_ID, rd_ID;
	wire [31:0] data1_ID, data2_ID;
	wire ALUsrc_ID;
	wire [3:0] ALUctrl_ID;
	wire RegWrite_ID;
	wire [1:0] Regsrc_ID;
	wire [1:0] forwardA_ID, forwardB_ID;
	wire [6:0] funct7;
	wire [2:0] funct3;
	wire [6:0] op;
	wire R, I, JALR, LW, S, B, J, shift;
	wire [31:0] immI, immS, immB, immJ, immshift, imm_ID;
	wire eq, branch;
	wire [31:0] eq_data1, eq_data2;
	wire memread_ID;
	wire memwrite_ID;
	reg [31:0] PC4_ID;
    wire [31:0] PC_correct;
	
	reg [31:0] PC_ID;
	reg [4:0] rs1_EX, rs2_EX, rd_EX;
	reg [31:0] data1_EX, data2_EX;
	reg [31:0] imm_EX;
	reg ALUsrc_EX;
	reg [3:0] ALUctrl_EX;
	reg RegWrite_EX;
	reg [1:0] Regsrc_EX;
	wire [31:0] reg_data1, reg_data2;
	wire [31:0] ALUin1, ALUin2;
	wire [31:0] ALUout_EX;
	wire [1:0] forwardA_EX, forwardB_EX;
	reg JALR_EX;
	reg memread_EX;
	reg memwrite_EX;
	reg [31:0] PC4_EX;

	reg [4:0] rs1_ME, rs2_ME, rd_ME;
	reg [31:0] data2_ME;
	reg RegWrite_ME;
	reg [1:0] Regsrc_ME;
	reg memread_ME;
	reg memwrite_ME;
	reg [31:0] PC4_ME;
	reg [31:0] ALUout_ME;

	reg [4:0] rs1_WB, rs2_WB, rd_WB;
	reg [31:0] ALUout_WB;
	reg RegWrite_WB;
	reg [1:0] Regsrc_WB;
	reg [31:0] PC4_WB;
	reg [31:0] memdata_WB;
	wire [31:0] Regdata_WB;
	
	// -------- --------
	assign PC = PC_IF;
	assign ICACHE_ren = ~stall;
	assign ICACHE_wen = 0;
	//assign ICACHE_addr = PC_IF[31:2];
	assign ICACHE_wdata = 0;

	// --------PC--------
	assign PC_w = (J)           ? PCimm_ID  : 
				  (JALR_EX)     ? ALUout_EX : 
                  (PreWrong)    ? PC_correct: 
                  (BrPre_IF)    ? BPU_PC_IF : PC4_IF;
                  
	always @(posedge clk or negedge rst_n) begin
		if (~rst_n) begin
			PC_IF <= 0;
		end
		else if (stall||DEC_stall||DCACHE_stall) begin
			PC_IF <= PC_IF;
		end
		else begin
			PC_IF <= PC_w;
		end
	end

	// --------hazard detect--------
	assign load_hazard = memread_EX && rd_EX!=0 && (rd_EX==rs1_ID || rd_EX == rs2_ID);
	assign branch_hazard = B&&(rd_EX!=0 &&RegWrite_EX && (rd_EX==rs1_ID || rd_EX == rs2_ID));
	assign stall = load_hazard || branch_hazard;
	assign flush = (branch&&~branch_hazard&&PreWrong) || J || JALR || JALR_EX || PreWrong;

	// --------IF--------
    assign PC4_IF = PC_IF + {29'b0, PCoff, 1'b0};
    //assign inst_IF = {ICACHE_rdata[7-:8], ICACHE_rdata[15-:8], ICACHE_rdata[23-:8], ICACHE_rdata[31-:8]};
    assign immB_IF = {{20{inst_IF[31]}}, inst_IF[7], inst_IF[30:25], inst_IF[11:8], 1'b0};
    assign op_IF = inst_IF[6:0];
    assign B_IF  = &{op_IF[6], op_IF[5], ~op_IF[4], ~op_IF[3], ~op_IF[2], op_IF[1], op_IF[0]};    // 1100011
    assign imm_IF = {32{B_IF}} & immB_IF;
    assign BPU_PC_IF = PC_IF + imm_IF;
    
    BranchPredictionUnit BPU(
        .clk(clk),
        .rst_n(rst_n),
        .PreWrong(PreWrong),
        .B(B_IF),
        .BrPre(BrPre_IF)
    );

	Decompressor DecIF(
		.clk(clk),
		.rst_n(rst_n),
		.inst(ICACHE_rdata),
		.PC(PC_IF),
		.stall(ICACHE_stall),
		.o_inst(inst_IF),
		.o_addr(ICACHE_addr),
		.o_stall(DEC_stall),
		.o_PCoff(PCoff)
	);

	// --------IF/ID--------
	always @(posedge clk or negedge rst_n) begin
		if (~rst_n) begin
			PC_ID <= 0;
			PC4_ID <= 0;
			inst_ID <= 32'h00000013;
            BrPre_ID <= 0;
            BPU_PC_ID <= 0;
		end
		else if (stall||DEC_stall||DCACHE_stall) begin
			PC_ID <= PC_ID;
			PC4_ID <= PC4_ID;
			inst_ID <= inst_ID;
            BrPre_ID <= BrPre_ID;
            BPU_PC_ID <= BPU_PC_ID;
		end
		else if (flush) begin
			PC_ID <= 0;
			PC4_ID <= 0;
			inst_ID <= 32'h00000013;
            BrPre_ID <= 0;
            BPU_PC_ID <= 0;
		end
		else begin
			PC_ID <= PC_IF;
			PC4_ID <= PC4_IF;
			inst_ID <= inst_IF;
            BrPre_ID <= BrPre_IF; 
            BPU_PC_ID <= BPU_PC_IF;
		end
	end
	// --------register file--------
	registerfile registerfile (
		.clk(clk),
		.rst_n(rst_n),
		.wen(RegWrite_WB),
		.wdata(Regdata_WB),
		.waddr(rd_WB),
		.raddr1(rs1_ID),
		.raddr2(rs2_ID),
		.rdata1(data1_ID),
		.rdata2(data2_ID)
	);
	// --------ID--------
	forwarding_unit_ID forwarding_unit_ID(
		.B(B),
		.RegWrite_ME(RegWrite_ME),
		.rd_ME(rd_ME),
		.RegWrite_WB(RegWrite_WB),
		.rd_WB(rd_WB),
		.rs1_ID(rs1_ID),
		.rs2_ID(rs2_ID),
		.forwardA(forwardA_ID),
		.forwardB(forwardB_ID)
	);

	assign funct7    = inst_ID[31:25];
	assign rs2_ID    = inst_ID[24:20];
	assign rs1_ID    = inst_ID[19:15];
	assign funct3    = inst_ID[14:12];
	assign rd_ID     = inst_ID[11:7];
	assign op        = inst_ID[6:0];

	assign R    = &{~op[6], op[5], op[4], ~op[3], ~op[2], op[1], op[0]};    // 0110011
	assign I    = &{~op[6], ~op[5], op[4], ~op[3], ~op[2], op[1], op[0]};   // 0010011
	assign JALR = &{op[6], op[5], ~op[4], ~op[3], op[2], op[1], op[0]};     // 1100111
	assign LW   = &{~op[6], ~op[5], ~op[4], ~op[3], ~op[2], op[1], op[0]};  // 0000011
	assign S    = &{~op[6], op[5], ~op[4], ~op[3], ~op[2], op[1], op[0]};   // 0100011
	assign B    = &{op[6], op[5], ~op[4], ~op[3], ~op[2], op[1], op[0]};    // 1100011
	assign J    = &{op[6], op[5], ~op[4], op[3], op[2], op[1], op[0]};      // 1101111
	assign shift = &{~funct3[2], ~funct3[1], funct3[0]} || // 001
				   &{funct3[2], ~funct3[1], funct3[0]};    // 101

	assign memread_ID = LW;
	assign memwrite_ID = S;

	assign immI = {{21{inst_ID[31]}}, inst_ID[30:20]};
    assign immS = {{21{inst_ID[31]}}, inst_ID[30:25], inst_ID[11:7]};
    assign immB = {{20{inst_ID[31]}}, inst_ID[7], inst_ID[30:25], inst_ID[11:8], 1'b0};
    assign immJ = {{12{inst_ID[31]}}, inst_ID[19:12], inst_ID[20], inst_ID[30:21], 1'b0};
	assign immshift = {27'b0, inst_ID[24:20]};

	assign imm_ID = ({32{I && shift}} & immshift) |
					({32{(I&&~shift)||JALR||LW}} & immI) |
					({32{S}} & immS) |
					({32{B}} & immB) |
					({32{J}} & immJ);

	assign eq_data1 = (forwardA_ID==2'b10) ? ALUout_ME :
					  (forwardA_ID==2'b01) ? Regdata_WB : data1_ID;
	assign eq_data2 = (forwardB_ID==2'b10) ? ALUout_ME :
					  (forwardB_ID==2'b01) ? Regdata_WB : data2_ID;
	assign eq = eq_data1 == eq_data2;
	assign branch = (eq^funct3[0])&&B;
    assign PreWrong = !(branch==BrPre_ID);
    assign PC_correct = (!BrPre_ID && branch) ? BPU_PC_ID :  
                        (BrPre_ID && !branch) ? PC4_ID : 0;
	assign PCimm_ID = PC_ID + imm_ID;

	assign ALUsrc_ID = I||JALR||LW||S;
	assign ALUctrl_ID[3] = R&&(&{funct7[5], ~funct3} || &{~funct3[2], funct3[1], ~funct3[0]});
	assign ALUctrl_ID[2] = (R||I)&&(&{~funct3[2], funct3[1], ~funct3[0]})||(I&&shift);
	assign ALUctrl_ID[1] = R&&(&{~funct3[2], ~funct3[0]}||&{~funct3[1], ~funct3[0]}) || I&&(&{~funct3[2], ~funct3[0]}||&{~funct3[1], ~funct3[0]}||(funct7[5]&&(&{funct3[2], ~funct3[1], funct3[0]})))||JALR||LW||S;
	assign ALUctrl_ID[0] = R&&(&{funct3[2], ~funct3[0]}) || I&&(&{funct3[2], ~funct3[0]}||&{funct3[2], ~funct3[1]});

	assign RegWrite_ID = (R||I||LW||JALR||J);
	assign Regsrc_ID = (R||I) ? 2'd0 : ((LW) ? 2'd1 : 2'd2); // 0 for ALUout, 1 for memdata, 2 for PC+4

	// -------ID/EX-------
	always @(posedge clk or negedge rst_n) begin
		if (~rst_n) begin
			rs1_EX <= 0;
			rs2_EX <= 0;
			rd_EX <= 0;
			data1_EX <= 0;
			data2_EX <= 0;
			imm_EX <= 0;
			ALUsrc_EX <= 0;
			ALUctrl_EX <= 4'b0010;
			RegWrite_EX <= 0;
			Regsrc_EX <= 0;
			JALR_EX <= 0;
			memread_EX <= 0;
			memwrite_EX <= 0;
			PC4_EX <= 0;
		end
		else if (stall) begin
			rs1_EX <= 0;
			rs2_EX <= 0;
			rd_EX <= 0;
			data1_EX <= 0;
			data2_EX <= 0;
			imm_EX <= 0;
			ALUsrc_EX <= 0;
			ALUctrl_EX <= 4'b0010;
			RegWrite_EX <= 0;
			Regsrc_EX <= 0;
			JALR_EX <= 0;
			memread_EX <= 0;
			memwrite_EX <= 0;
			PC4_EX <= 0;
		end
		else if (DEC_stall||DCACHE_stall) begin
			rs1_EX <= rs1_EX;
			rs2_EX <= rs2_EX;
			rd_EX <= rd_EX;
			data1_EX <= data1_EX;
			data2_EX <= data2_EX;
			imm_EX <= imm_EX;
			ALUsrc_EX <= ALUsrc_EX;
			ALUctrl_EX <= ALUctrl_EX;
			RegWrite_EX <= RegWrite_EX;
			Regsrc_EX <= Regsrc_EX;
			JALR_EX <= JALR_EX;
			memread_EX <= memread_EX;
			memwrite_EX <= memwrite_EX;
			PC4_EX <= PC4_EX;
		end
		else begin
			rs1_EX <= rs1_ID;
			rs2_EX <= rs2_ID;
			rd_EX <= rd_ID;
			data1_EX <= data1_ID;
			data2_EX <= data2_ID;
			imm_EX <= imm_ID;
			ALUsrc_EX <= ALUsrc_ID;
			ALUctrl_EX <= ALUctrl_ID;
			RegWrite_EX <= RegWrite_ID;
			Regsrc_EX <= Regsrc_ID;
			JALR_EX <= JALR;
			memread_EX <= memread_ID;
			memwrite_EX <= memwrite_ID;
			PC4_EX <= PC4_ID;
		end
	end
	// --------EX--------
	forwarding_unit_EX forwarding_unit_EX (
		.RegWrite_ME(RegWrite_ME),
		.rd_ME(rd_ME),
		.RegWrite_WB(RegWrite_WB),
		.rd_WB(rd_WB),
		.rs1_EX(rs1_EX),
		.rs2_EX(rs2_EX),
		.forwardA(forwardA_EX),
		.forwardB(forwardB_EX)
	);

	ALU ALU (
		.A(ALUin1),
		.B(ALUin2),
		.ALUctrl(ALUctrl_EX),
		.ALUout(ALUout_EX)
	);

	assign reg_data1 = (forwardA_EX==2'b10) ? ALUout_ME :
					   (forwardA_EX==2'b01) ? Regdata_WB : data1_EX;
	assign reg_data2 = (forwardB_EX==2'b10) ? ALUout_ME :
					   (forwardB_EX==2'b01) ? Regdata_WB : data2_EX;
	assign ALUin1 = reg_data1;
	assign ALUin2 = (ALUsrc_EX) ? imm_EX : reg_data2;

	// --------EX/ME--------
	always @(posedge clk or negedge rst_n) begin
		if (~rst_n) begin
			rs1_ME <= 0;
			rs2_ME <= 0;
			rd_ME <= 0;
			RegWrite_ME <= 0;
			Regsrc_ME <= 0;
			memread_ME <= 0;
			memwrite_ME <= 0;
			PC4_ME <= 0;
			ALUout_ME <= 0;
			data2_ME <= 0;
		end
		else if (DEC_stall||DCACHE_stall) begin
			rs1_ME <= rs1_ME;
			rs2_ME <= rs2_ME;
			rd_ME <= rd_ME;
			RegWrite_ME <= RegWrite_ME;
			Regsrc_ME <= Regsrc_ME;
			memread_ME <= memread_ME;
			memwrite_ME <= memwrite_ME;
			PC4_ME <= PC4_ME;
			ALUout_ME <= ALUout_ME;
			data2_ME <= data2_ME;
		end
		else begin
			rs1_ME <= rs1_EX;
			rs2_ME <= rs2_EX;
			rd_ME <= rd_EX;
			RegWrite_ME <= RegWrite_EX;
			Regsrc_ME <= Regsrc_EX;
			memread_ME <= memread_EX;
			memwrite_ME <= memwrite_EX;
			PC4_ME <= PC4_EX;
			ALUout_ME <= ALUout_EX;
			data2_ME <= reg_data2;
		end
	end
	// --------ME--------
	assign DCACHE_ren = memread_ME;
	assign DCACHE_wen = memwrite_ME;
	assign DCACHE_addr = ALUout_ME[31:2];
	assign DCACHE_wdata = {data2_ME[7-:8], data2_ME[15-:8], data2_ME[23-:8], data2_ME[31-:8]};

	// --------ME/WB--------
	always @(posedge clk or negedge rst_n) begin
		if (~rst_n) begin
			rs1_WB <= 0;
			rs2_WB <= 0;
			rd_WB <= 0;
			RegWrite_WB <= 0;
			Regsrc_WB <= 0;
			PC4_WB <= 0;
			ALUout_WB <= 0;
			memdata_WB <= 0;
		end
		else if (DEC_stall||DCACHE_stall) begin
			rs1_WB <= rs1_WB;
			rs2_WB <= rs2_WB;
			rd_WB <= rd_WB;
			RegWrite_WB <= RegWrite_WB;
			Regsrc_WB <= Regsrc_WB;
			PC4_WB <= PC4_WB;
			ALUout_WB <= ALUout_WB;
			memdata_WB <= memdata_WB;
		end
		else begin
			rs1_WB <= rs1_ME;
			rs2_WB <= rs2_ME;
			rd_WB <= rd_ME;
			RegWrite_WB <= RegWrite_ME;
			Regsrc_WB <= Regsrc_ME;
			PC4_WB <= PC4_ME;
			ALUout_WB <= ALUout_ME;
			memdata_WB <= DCACHE_rdata;
		end
	end

	// --------WB--------
	assign Regdata_WB = (Regsrc_WB==2) ? PC4_WB :
						(Regsrc_WB==1) ? {memdata_WB[7-:8], memdata_WB[15-:8], memdata_WB[23-:8], memdata_WB[31-:8]} : ALUout_WB;
	
endmodule

module registerfile (
	input         clk,
	input         rst_n,
	input         wen,
	input  [31:0] wdata,
	input  [ 4:0] waddr,
	input  [ 4:0] raddr1,
	input  [ 4:0] raddr2,
	output [31:0] rdata1,
	output [31:0] rdata2
);	
	
	reg  [31:0] register_r [0:31];
	wire [31:0] register_w [0:31];
	genvar i;
	integer j;

	assign rdata1 = register_r[raddr1];
	assign rdata2 = register_r[raddr2];

	assign register_w[0] = 0;
	generate
		for (i=1; i<32; i=i+1) begin : rf
			assign register_w[i] = (waddr == i && wen) ? wdata : register_r[i]; 
		end
	endgenerate

	always @(negedge clk or negedge rst_n) begin
		if (~rst_n) begin
			for (j=0; j<32; j=j+1) begin
				register_r[j] = 0;
			end
		end
		else begin
			for (j=0; j<32; j=j+1) begin
				register_r[j] = register_w[j];
			end
		end
	end

endmodule

module ALU (
	input  [31:0] A,
	input  [31:0] B,
	input  [ 3:0] ALUctrl,
	output reg [31:0] ALUout
);

	wire [31:0] ALUA, ALUB;
	wire Cin;
	wire [31:0] OP000, OP001, OP010, OP011, OP100, OP101, OP110, OP111;

	assign ALUA = A;
	assign ALUB = ALUctrl[3] || &{ALUctrl[2], ALUctrl[1], ~ALUctrl[0]} ? ~B : B;
	assign Cin = ALUctrl[3];

	assign OP000 = ALUA & ALUB;
	assign OP001 = ALUA | ALUB;
	assign OP010 = ALUA + ALUB + Cin;
	assign OP011 = ALUA ^ ALUB;
	assign OP100 = ALUA << ALUB;
	assign OP101 = ALUA >> ALUB;
	assign OP110 = {31'b0, OP010[31]};
	assign OP111 = $signed(ALUA) >>> ALUB;

	always @(*) begin
		case(ALUctrl[2:0])
			3'b000: ALUout = OP000; // and
			3'b001: ALUout = OP001; // or
			3'b010: ALUout = OP010; // add
			3'b011: ALUout = OP011; // xor
			3'b100: ALUout = OP100; // slli
			3'b101: ALUout = OP101; // srli
			3'b110: ALUout = OP110; // slt
			3'b111: ALUout = OP111; // srai
		    default: ALUout = 0; 
		endcase
	end
	
endmodule

module forwarding_unit_EX (
	input RegWrite_ME,
	input [4:0] rd_ME,
	input RegWrite_WB,
	input [4:0] rd_WB,
	input [4:0] rs1_EX,
	input [4:0] rs2_EX,
	output [1:0] forwardA,
	output [1:0] forwardB
);
	assign forwardA = (RegWrite_ME && |rd_ME && rd_ME==rs1_EX) ? (2'b10) :	// EX hazard
					  (RegWrite_WB && |rd_WB && rd_WB==rs1_EX) ? (2'b01) : 	// ME hazard
					  (2'b00);												// no data hazard
	assign forwardB = (RegWrite_ME && |rd_ME && rd_ME==rs2_EX) ? (2'b10) :	// EX hazard
					  (RegWrite_WB && |rd_WB && rd_WB==rs2_EX) ? (2'b01) : 	// ME hazard
					  (2'b00);												// no data hazard
endmodule

module forwarding_unit_ID (
	input B,
	input RegWrite_ME,
	input [4:0] rd_ME,
	input RegWrite_WB,
	input [4:0] rd_WB,
	input [4:0] rs1_ID,
	input [4:0] rs2_ID,
	output [1:0] forwardA,
	output [1:0] forwardB
);
	assign forwardA = (RegWrite_ME && |rd_ME && rd_ME==rs1_ID) ? (2'b10) :	// EX hazard
					  (RegWrite_WB && |rd_WB && rd_WB==rs1_ID) ? (2'b01) : 	// ME hazard
					  (2'b00);								  				// no data hazard
	assign forwardB = (RegWrite_ME && |rd_ME && rd_ME==rs2_ID) ? (2'b10) :	// EX hazard
					  (RegWrite_WB && |rd_WB && rd_WB==rs2_ID) ? (2'b01) : 	// ME hazard
					  (2'b00);												// no data hazard
endmodule


/*******************************
Branch Prediction Unit
*******************************/
module BranchPredictionUnit (
    input  clk,
    input  rst_n,
    input  PreWrong,
    input  B,
    output BrPre
); 
    parameter UNTAKEN_2 = 0;
    parameter UNTAKEN_1 = 1;
    parameter TAKEN_1 = 2;
    parameter TAKEN_2 = 3;
    
    reg [1:0] state_r,state_w;

    assign BrPre = (state_r[1] && B) ? 1'b1 : 1'b0;

    always@ (*) begin
        case(state_r) 
            UNTAKEN_1: begin
                if(PreWrong) 
                    state_w = UNTAKEN_2;
                else
                    state_w = state_r;
            end
            UNTAKEN_2: begin
                if(PreWrong) 
                    state_w = TAKEN_1;
                else
                    state_w = UNTAKEN_1;
            end
            TAKEN_1: begin
                if(PreWrong)
                    state_w = UNTAKEN_2;
                else
                    state_w = TAKEN_2;
            end
            TAKEN_2: begin
                if(PreWrong)
                    state_w = TAKEN_1;
                else
                    state_w = state_r;
            end
            default: state_w = state_r;
        endcase
    end

    always@ (posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            state_r <= UNTAKEN_2;
        end
        else begin
            state_r <= state_w;
        end
    end

endmodule


/*************************************
CACHE DESIGN                      
ICACHE FOR READ ONLY CACHE        
DCACHE FOR NORMAL CACHE            
*************************************/
module Icache(
    clk,
    proc_reset,
    proc_read,
    proc_write,
    proc_addr,
    proc_rdata,
    proc_wdata,
    proc_stall,
    mem_read,
    mem_write,
    mem_addr,
    mem_rdata,
    mem_wdata,
    mem_ready
);
    
//==== input/output definition ============================
    input          clk;
    // processor interface
    input             proc_reset;
    input             proc_read, proc_write;
    input   [29:0]    proc_addr;
    input   [31:0]    proc_wdata;
    output reg        proc_stall;
    output reg [31:0] proc_rdata;
    // memory interface
    input  [127:0]     mem_rdata;
    input              mem_ready;
    output reg         mem_read, mem_write;
    output reg [27:0]  mem_addr;
    output reg [127:0] mem_wdata;
    
//==== wire/reg definition ================================
    parameter COMPARE = 0;
    parameter ALLOCATE = 1;
    
    integer i;
    reg   state_r,state_w;
    // valid, tag(26bits), data(32x4 bits)
    reg [154:0] data_r[0:7];   
    reg [154:0] data_w[0:7];
    wire [25:0] tag_i;
    wire [1:0]  index_i;
    wire [2:0]  index_even,index_odd;
    wire [1:0]  offset_i;
    wire valid_i_1, valid_i_2;
    wire hit_i_1,   hit_i_2;

    //LRU 
    reg [3:0] lru;
    reg [3:0] lru_next;
    reg [2:0] index_next_r,index_next_w;

//==== combinational circuit ==============================
    assign tag_i = proc_addr[29:4];
    assign index_i = proc_addr[3:2];
    assign offset_i = proc_addr[1:0];
    assign index_even = index_i << 1;
    assign index_odd = index_even + 1'b1;

    assign valid_i_1 = data_r[index_even][154];
    assign valid_i_2 = data_r[index_odd][154];
    assign hit_i_1 = (data_r[index_even][153:128] == tag_i && valid_i_1); 
    assign hit_i_2 = (data_r[index_odd][153:128] == tag_i && valid_i_2); 

    always@ (*) begin
        case(state_r) 
            COMPARE: state_w = (!(proc_read) || (hit_i_1)||(hit_i_2)) ? COMPARE : ALLOCATE;
            ALLOCATE: state_w = (mem_ready) ? COMPARE : ALLOCATE;
            default: state_w = state_r;
        endcase
    end

    always@ (*) begin
        proc_stall = 0;
        mem_read = 0;
        mem_write = 0;
        mem_addr = 0;
        mem_wdata = 0;

        case(state_r)
            COMPARE: begin
                if((proc_read) && !hit_i_1 && !hit_i_2)
                    proc_stall = 1;
            end
            ALLOCATE: begin
                mem_read = !mem_ready;
                proc_stall = 1;
                mem_addr = proc_addr[29:2];
            end
        endcase
    end

    always@ (*) begin
        for(i=0;i<8;i=i+1) begin
            data_w[i] = data_r[i];
        end
        proc_rdata = 0;
        lru_next = lru;
        index_next_w = index_next_r;

        if(state_r == COMPARE) begin
            if(proc_read) begin
                if((hit_i_1)) begin
                    case(offset_i)
                        2'b00: proc_rdata = data_r[index_even][31:0];
                        2'b01: proc_rdata = data_r[index_even][63:32];
                        2'b10: proc_rdata = data_r[index_even][95:64];
                        2'b11: proc_rdata = data_r[index_even][127:96];
                    endcase
                end 
                else if(hit_i_2) begin
                    case(offset_i)
                        2'b00: proc_rdata = data_r[index_odd][31:0];
                        2'b01: proc_rdata = data_r[index_odd][63:32];
                        2'b10: proc_rdata = data_r[index_odd][95:64];
                        2'b11: proc_rdata = data_r[index_odd][127:96];
                    endcase
                end
            end
        end
        else if (state_r == ALLOCATE) begin
            if(mem_ready) begin
                data_w[index_next_r] = {1'b1,proc_addr[29:4],mem_rdata};

                case(index_i)
                    2'b00: begin
                        if(lru[0]) lru_next[0] = 0;
                        else       lru_next[0] = 1;
                    end
                    2'b01: begin
                        if(lru[1]) lru_next[1] = 0;
                        else       lru_next[1] = 1;
                    end
                    2'b10: begin
                        if(lru[2]) lru_next[2] = 0;
                        else       lru_next[2] = 1;
                    end
                    2'b11: begin
                        if(lru[3]) lru_next[3] = 0;
                        else       lru_next[3] = 1;
                    end
                    default: lru_next = lru;
                endcase    
            end
            else begin
                case(index_i) 
                    2'b00: index_next_w = (lru[0]) ? 3'b000 : 3'b001;
                    2'b01: index_next_w = (lru[1]) ? 3'b010 : 3'b011;
                    2'b10: index_next_w = (lru[2]) ? 3'b100 : 3'b101;
                    2'b11: index_next_w = (lru[3]) ? 3'b110 : 3'b111;
                    default : index_next_w = index_next_r;
                endcase
            end
        end
    end

//==== sequential circuit =================================
    always@( posedge clk ) begin
        if( proc_reset ) begin
            state_r <= COMPARE;
            lru <= 0;
            index_next_r <= 0;
            
            for(i=0;i<8;i=i+1) begin
                data_r[i] <= 0;
            end
        end
        else begin
            state_r <= state_w;
            lru <= lru_next;
            index_next_r <= index_next_w;

            for(i=0;i<8;i=i+1) begin
                data_r[i] <= data_w[i];
            end
        end
    end

endmodule

module Dcache(
    clk,
    proc_reset,
    proc_read,
    proc_write,
    proc_addr,
    proc_rdata,
    proc_wdata,
    proc_stall,
    mem_read,
    mem_write,
    mem_addr,
    mem_rdata,
    mem_wdata,
    mem_ready
);
    
//==== input/output definition ============================
    input          clk;
    // processor interface
    input          proc_reset;
    input          proc_read, proc_write;
    input   [29:0] proc_addr;
    input   [31:0] proc_wdata;
    output reg     proc_stall;
    output reg [31:0] proc_rdata;
    // memory interface
    input  [127:0]     mem_rdata;
    input              mem_ready;
    output reg         mem_read, mem_write;
    output reg [27:0]  mem_addr;
    output reg [127:0] mem_wdata;
    
//==== wire/reg definition ================================
    parameter COMPARE = 0;
    parameter ALLOCATE = 1;
    parameter WRITE_BACK = 2;
    
    integer i;
    reg [1:0]   state_r,state_w;
    // valid, dirty, tag(26bits), data(32x4 bits)
    reg [155:0] data_r[0:7];   
    reg [155:0] data_w[0:7];
    wire [25:0] tag_i;
    wire [1:0]  index_i;
    wire [2:0]  index_even,index_odd;
    wire [1:0]  offset_i;
    wire valid_i_1, valid_i_2;
    wire dirty_i_1, dirty_i_2;
    wire hit_i_1,   hit_i_2;

    //LRU 
    reg [3:0] lru;
    reg [3:0] lru_next;
    reg [2:0] index_next_r,index_next_w;

//==== combinational circuit ==============================
    assign tag_i = proc_addr[29:4];
    assign index_i = proc_addr[3:2];
    assign offset_i = proc_addr[1:0];
    assign index_even = index_i << 1;
    assign index_odd = index_even + 1'b1;

    assign valid_i_1 = data_r[index_even][155];
    assign valid_i_2 = data_r[index_odd][155];
    assign dirty_i_1 = data_r[index_even][154];
    assign dirty_i_2 = data_r[index_odd][154];
    assign hit_i_1 = (data_r[index_even][153:128] == tag_i && valid_i_1); 
    assign hit_i_2 = (data_r[index_odd][153:128] == tag_i && valid_i_2); 

    always@ (*) begin
        case(state_r) 
            COMPARE: state_w = (!(proc_read ^ proc_write) || (hit_i_1)||(hit_i_2)) ? COMPARE : ((dirty_i_1)||(dirty_i_2)) ? WRITE_BACK : ALLOCATE;
            ALLOCATE: state_w = (mem_ready) ? COMPARE : ALLOCATE;
            WRITE_BACK: state_w = (mem_ready) ? ALLOCATE : WRITE_BACK;
            default: state_w = state_r;
        endcase
    end

    always@ (*) begin
        proc_stall = 0;
        mem_read = 0;
        mem_write = 0;
        mem_addr = 0;
        mem_wdata = 0;

        case(state_r)
            COMPARE: begin
                if((proc_read ^ proc_write) && !hit_i_1 && !hit_i_2)
                    proc_stall = 1;
            end
            WRITE_BACK: begin
                mem_write = !mem_ready;
                proc_stall = 1;

                if(dirty_i_1 && !lru[index_i]) begin
                    mem_wdata = data_r[index_even][127:0];
                    mem_addr = {data_r[index_even][153:128],index_i};
                end
                else if(dirty_i_2 && lru[index_i]) begin
                    mem_wdata = data_r[index_odd][127:0];
                    mem_addr = {data_r[index_odd][153:128],index_i};
                end
            end
            ALLOCATE: begin
                mem_read = !mem_ready;
                proc_stall = 1;
                mem_addr = proc_addr[29:2];
            end
        endcase
    end

    always@ (*) begin
        for(i=0;i<8;i=i+1) begin
            data_w[i] = data_r[i];
        end
        proc_rdata = 0;
        lru_next = lru;
        index_next_w = index_next_r;

        if(state_r == COMPARE) begin
            if(proc_read) begin
                if((hit_i_1)) begin
                    case(offset_i)
                        2'b00: proc_rdata = data_r[index_even][31:0];
                        2'b01: proc_rdata = data_r[index_even][63:32];
                        2'b10: proc_rdata = data_r[index_even][95:64];
                        2'b11: proc_rdata = data_r[index_even][127:96];
                    endcase
                end 
                else if(hit_i_2) begin
                    case(offset_i)
                        2'b00: proc_rdata = data_r[index_odd][31:0];
                        2'b01: proc_rdata = data_r[index_odd][63:32];
                        2'b10: proc_rdata = data_r[index_odd][95:64];
                        2'b11: proc_rdata = data_r[index_odd][127:96];
                    endcase
                end
            end
            else if(proc_write) begin
                if((hit_i_1)) begin
                    case(offset_i)
                        2'b00: data_w[index_even] = {2'b11,data_r[index_even][153:128],data_r[index_even][127:32],proc_wdata};
                        2'b01: data_w[index_even] = {2'b11,data_r[index_even][153:128],data_r[index_even][127:64],proc_wdata,data_r[index_even][31:0]};
                        2'b10: data_w[index_even] = {2'b11,data_r[index_even][153:128],data_r[index_even][127:96],proc_wdata,data_r[index_even][63:0]};
                        2'b11: data_w[index_even] = {2'b11,data_r[index_even][153:128],proc_wdata,data_r[index_even][95:0]};
                    endcase
                end  
                else if((hit_i_2)) begin
                    case(offset_i)
                        2'b00: data_w[index_odd] = {2'b11,data_r[index_odd][153:128],data_r[index_odd][127:32],proc_wdata};
                        2'b01: data_w[index_odd] = {2'b11,data_r[index_odd][153:128],data_r[index_odd][127:64],proc_wdata,data_r[index_odd][31:0]};
                        2'b10: data_w[index_odd] = {2'b11,data_r[index_odd][153:128],data_r[index_odd][127:96],proc_wdata,data_r[index_odd][63:0]};
                        2'b11: data_w[index_odd] = {2'b11,data_r[index_odd][153:128],proc_wdata,data_r[index_odd][95:0]};
                    endcase
                end  
            end
        end
        else if (state_r == ALLOCATE) begin
            if(mem_ready) begin
                data_w[index_next_r] = {2'b10,proc_addr[29:4],mem_rdata};

                case(index_i)
                    2'b00: begin
                        if(lru[0]) lru_next[0] = 0;
                        else       lru_next[0] = 1;
                    end
                    2'b01: begin
                        if(lru[1]) lru_next[1] = 0;
                        else       lru_next[1] = 1;
                    end
                    2'b10: begin
                        if(lru[2]) lru_next[2] = 0;
                        else       lru_next[2] = 1;
                    end
                    2'b11: begin
                        if(lru[3]) lru_next[3] = 0;
                        else       lru_next[3] = 1;
                    end
                    default: lru_next = lru;
                endcase    
            end
            else begin
                case(index_i) 
                    2'b00: index_next_w = (lru[0]) ? 3'b000 : 3'b001;
                    2'b01: index_next_w = (lru[1]) ? 3'b010 : 3'b011;
                    2'b10: index_next_w = (lru[2]) ? 3'b100 : 3'b101;
                    2'b11: index_next_w = (lru[3]) ? 3'b110 : 3'b111;
                    default : index_next_w = index_next_r;
                endcase
            end
        end
    end

//==== sequential circuit =================================
    always@( posedge clk ) begin
        if( proc_reset ) begin
            state_r <= COMPARE;
            lru <= 0;
            index_next_r <= 0;
            
            for(i=0;i<8;i=i+1) begin
                data_r[i] <= 0;
            end
        end
        else begin
            state_r <= state_w;
            lru <= lru_next;
            index_next_r <= index_next_w;

            for(i=0;i<8;i=i+1) begin
                data_r[i] <= data_w[i];
            end
        end
    end

endmodule

/*****************************
Decompressor 
Used for decompressing the instuction
*****************************/
module Decompressor(
	input               clk,
    input               rst_n,
    input        [31:0] inst,
    input        [31:0] PC,
    input               stall,
    output  reg  [31:0] o_inst,
    output       [29:0] o_addr,
    output              o_stall,
    output  reg   [1:0] o_PCoff
);

	wire [31:0]  i_inst;
	wire [15:0]  C_inst;
	reg  [15:0]  i_inst_before_r;
	wire [15:0]  i_inst_before_w;
	reg   I_not_aligned_r;
	wire  I_not_aligned_w;

    // Long instuction but not aligned
	assign i_inst = {inst[7:0], inst[15:8], inst[23:16], inst[31:24]};
	assign i_inst_before_w = i_inst[31:16];
	assign C_inst = PC[1] ? i_inst[31:16] : i_inst[15:0];
	assign I_not_aligned_w = (!stall) & (!I_not_aligned_r) & (PC[1]) & (C_inst[1]) & (C_inst[0]);
	assign o_stall  = stall || I_not_aligned_w;
	assign o_addr = I_not_aligned_r ? (PC[31:2] + 30'd1) : PC[31:2];

	always@(*) begin
		if (I_not_aligned_r) begin
			// PC + 4 
			o_PCoff = 2'b10; 
			o_inst = {i_inst[15:0],i_inst_before_r};
		end
		else if (C_inst[1:0] == 2'b11) begin
			// PC + 4
			o_PCoff = 2'b10; 
			o_inst = i_inst;
		end
		else begin
			// PC + 2
			o_PCoff = 2'b01; 

			case({C_inst[15:13],C_inst[1:0]})
				5'b01000:
				// C.lw
					o_inst ={5'b00000,C_inst[5],C_inst[12:10],C_inst[6],2'b0,
							2'b01,C_inst[9:7],
							3'b010,
							2'b01,C_inst[4:2],
							7'b0000011}; 
				5'b11000:
				// C.sw
					o_inst ={5'b00000,C_inst[5],C_inst[12],
							2'b01,C_inst[4:2],
							2'b01,C_inst[9:7],
							3'b010,
							C_inst[11:10],C_inst[6],2'b0,
							7'b0100011}; 
				5'b00001:
				// C.addi
					o_inst ={{7{C_inst[12]}},C_inst[6:2],
							C_inst[11:7],
							3'b000,
							C_inst[11:7],
							7'b0010011}; 
				5'b00101:
				// C.jal
					o_inst ={C_inst[12],C_inst[8],C_inst[10:9],C_inst[6],C_inst[7],C_inst[2],C_inst[11],C_inst[5:3],{9{C_inst[12]}},
							5'b00001,
							7'b1101111}; 
				5'b10101:
				// C.j
					o_inst ={C_inst[12],C_inst[8],C_inst[10:9],C_inst[6],C_inst[7],C_inst[2],C_inst[11],C_inst[5:3],{9{C_inst[12]}},
							5'b00000,
							7'b1101111}; 
				5'b10001: begin
					case(C_inst[11:10])
						2'b00:
						// C.srli
							o_inst ={7'b0000000,
								C_inst[6:2],
								2'b01,C_inst[9:7],
								3'b101,
								2'b01,C_inst[9:7],
								7'b0010011}; 
						2'b01:
						// C.srai
							o_inst ={7'b0100000,
								C_inst[6:2],
								2'b01,C_inst[9:7],
								3'b101,
								2'b01,C_inst[9:7],
								7'b0010011}; 
						2'b10:
						// C.andi
							o_inst ={{7{C_inst[12]}},C_inst[6:2],
								2'b01,C_inst[9:7],
								3'b111,
								2'b01,C_inst[9:7],
								7'b0010011}; 
						default:
							o_inst = 32'h00000013;
					endcase
				end
				5'b11001:
				// C.beqz
					o_inst ={{4{C_inst[12]}},C_inst[6:5],C_inst[2],
							5'b00000,
							2'b01,C_inst[9:7],
							3'b000,
							C_inst[11:10],C_inst[4:3],C_inst[12],
							7'b1100011}; 
				5'b11101:
				// C.bnez
					o_inst ={{4{C_inst[12]}},C_inst[6:5],C_inst[2],
							5'b00000,
							2'b01,C_inst[9:7],
							3'b001,
							C_inst[11:10],C_inst[4:3],C_inst[12],
							7'b1100011}; 
				5'b00010:
				// C.slli
					o_inst ={7'b0000000,
							C_inst[6:2],
							2'b01,C_inst[9:7],
							3'b001,
							2'b01,C_inst[9:7],
							7'b0010011}; 
				5'b10010: begin
					case({C_inst[12], C_inst[6:2]!=5'd0})
						2'b00:
						// C.jr
							o_inst ={12'b0,
									C_inst[11:7],
									3'b000,
									5'b00000,
									7'b1100111}; 
						2'b10:
						// C.jalr
							o_inst ={12'b0,
									C_inst[11:7],
									3'b000,
									5'b00001,
									7'b1100111}; 
						2'b01:
						// C.mv
							o_inst ={7'b0000000,
									C_inst[6:2],
									5'b00000,
									3'b000,
									C_inst[11:7],
									7'b0110011}; 
						2'b11:
						// C.add
							o_inst ={7'b0000000,
									C_inst[6:2],
									C_inst[11:7],
									3'b000,
									C_inst[11:7],
									7'b0110011}; 
					endcase
				end
				default:
					o_inst = 32'h00000013;
			endcase
		end
	end

	always@(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			I_not_aligned_r <= 1'b0;
			i_inst_before_r <= 16'b0;
		end
		else if(stall)begin
			I_not_aligned_r <= I_not_aligned_r;
			i_inst_before_r <= i_inst_before_r;
		end
		else begin
			I_not_aligned_r <= I_not_aligned_w;
			i_inst_before_r <= i_inst_before_w;
		end
	end

endmodule
