// Ctrl完成isue阶段
`timescale 1ns / 1ps
`include "CtrlDefine.vh"

module CtrlUnit(

    input clk,
    input rst,

    input[31:0] PC_IF,
    input[31:0] inst_IF,

    input[31:0] PC_IS,
    input[31:0] inst,
    input[31:0] imm,
    output IS_en,
    output[2:0] ImmSel,

    // register
    input[31:0] rs1_from_reg,
    input[31:0] rs2_from_reg,
    output register_write_en,
    output[4:0] register_to_write,
    output[31:0] register_data_to_write,


    // FU control
    output[31:0] rs1_to_alu, rs2_to_alu, imm_to_alu, pc_to_alu,
    output[31:0] rs1_to_alu2, rs2_to_alu2, imm_to_alu2, pc_to_alu2,

    output[31:0] rs1_to_mul, rs2_to_mul,
    output[31:0] rs1_to_mul2, rs2_to_mul2,

    output[31:0] rs1_to_div, rs2_to_div,
    output[31:0] rs2_to_mem, imm_to_mem, rs1_to_mem,
    output[31:0] rs2_to_mem2, imm_to_mem2, rs1_to_mem2,

    output[2:0] b_h_w_to_mem2,
    output mem2_ctrl,

    output[2:0] b_h_w_to_mem,
    output mem_ctrl,
    output[31:0] rs1_to_jump, rs2_to_jump, imm_to_jump,pc_to_jump,

    input ALU_done,
    input MEM_done,
    input MUL_done,
    input DIV_done,
    input JUMP_done,
    input ALU2_done,
    input MEM2_done,
    input MUL2_done,
    input sd_finish,
    input sd_finish2,
    output reg [8 :0] CDB_result,

    output  ALU_en, MEM_en, MUL_en, DIV_en, JUMP_en, ALU2_en, MEM2_en, MUL2_en,
    input [31:0] ALU_res,
    input [31:0] MEM_res,
    input [31:0] MUL_res,
    input [31:0] DIV_res,
    input [31:0] JUMP_res,
    input [31:0] ALU2_res,
    input [31:0] MEM2_res,
    input [31:0] MUL2_res,

    // ALU
    output[3:0] ALU_op_to_fu,
    output[3:0] ALU2_op_to_fu,
    output[3:0] Jump_op_to_fu,
    output ALUSrcB_to_fu,
    output ALUSrcA_to_fu,


    //branch
    input cmp_res_FU,
    output branch_ctrl, branch_flush,
    input branch_finish,


    //debug
    input[4:0] debug_addr,
    output[31:0] Testout
  );

  parameter FU_NUM = 8;
  parameter ALU_INDEX = 4'd1;
  parameter MEM_INDEX = 4'd2;
  parameter MUL_INDEX = 4'd3;
  parameter DIV_INDEX = 4'd4;
  parameter JUMP_INDEX = 4'd5;
  parameter ALU2_INDEX = 4'd6;
  parameter MEM2_INDEX = 4'd7;
  parameter MUL2_INDEX = 4'd8;
  parameter Q_WIDTH = 5 ;
  // instruction field
  wire[6:0] funct7 = inst[31:25];
  wire[2:0] funct3 = inst[14:12];
  wire[6:0] opcode = inst[6:0];
  wire[4:0] rd = inst[11:7];
  wire[4:0] rs1 = inst[19:15];
  wire[4:0] rs2 = inst[24:20];


  // type specification
  wire Rop = opcode == 7'b0110011;
  wire Iop = opcode == 7'b0010011;
  wire Bop = opcode == 7'b1100011;
  wire Lop = opcode == 7'b0000011;
  wire Sop = opcode == 7'b0100011;

  wire funct7_0  = funct7 == 7'h0;
  wire funct7_1  = funct7 == 7'h1;
  wire funct7_32 = funct7 == 7'h20;

  wire funct3_0 = funct3 == 3'h0;
  wire funct3_1 = funct3 == 3'h1;
  wire funct3_2 = funct3 == 3'h2;
  wire funct3_3 = funct3 == 3'h3;
  wire funct3_4 = funct3 == 3'h4;
  wire funct3_5 = funct3 == 3'h5;
  wire funct3_6 = funct3 == 3'h6;
  wire funct3_7 = funct3 == 3'h7;

  wire ADD  = Rop & funct3_0 & funct7_0;
  wire SUB  = Rop & funct3_0 & funct7_32;
  wire SLL  = Rop & funct3_1 & funct7_0;
  wire SLT  = Rop & funct3_2 & funct7_0;
  wire SLTU = Rop & funct3_3 & funct7_0;
  wire XOR  = Rop & funct3_4 & funct7_0;
  wire SRL  = Rop & funct3_5 & funct7_0;
  wire SRA  = Rop & funct3_5 & funct7_32;
  wire OR   = Rop & funct3_6 & funct7_0;
  wire AND  = Rop & funct3_7 & funct7_0;

  wire MUL    = Rop & funct3_0 & funct7_1;
  wire MULH   = Rop & funct3_1 & funct7_1;
  wire MULHSU = Rop & funct3_2 & funct7_1;
  wire MULHU  = Rop & funct3_3 & funct7_1;
  wire DIV    = Rop & funct3_4 & funct7_1;
  wire DIVU   = Rop & funct3_5 & funct7_1;
  wire REM    = Rop & funct3_6 & funct7_1;
  wire REMU    = Rop & funct3_7 & funct7_1;

  wire ADDI  = Iop & funct3_0;
  wire SLTI  = Iop & funct3_2;
  wire SLTIU = Iop & funct3_3;
  wire XORI  = Iop & funct3_4;
  wire ORI   = Iop & funct3_6;
  wire ANDI  = Iop & funct3_7;
  wire SLLI  = Iop & funct3_1 & funct7_0;
  wire SRLI  = Iop & funct3_5 & funct7_0;
  wire SRAI  = Iop & funct3_5 & funct7_32;

  wire BEQ = Bop & funct3_0;
  wire BNE = Bop & funct3_1;
  wire BLT = Bop & funct3_4;
  wire BGE = Bop & funct3_5;
  wire BLTU = Bop & funct3_6;
  wire BGEU = Bop & funct3_7;

  wire LB =  Lop & funct3_0;
  wire LH =  Lop & funct3_1;
  wire LW =  Lop & funct3_2;
  wire LBU = Lop & funct3_4;
  wire LHU = Lop & funct3_5;

  wire SB = Sop & funct3_0;
  wire SH = Sop & funct3_1;
  wire SW = Sop & funct3_2;

  wire LUI   = opcode == 7'b0110111;
  wire AUIPC = opcode == 7'b0010111;

  wire JAL  =  opcode == 7'b1101111;
  wire JALR = (opcode == 7'b1100111) && funct3_0;

  wire R_valid = AND | OR | ADD | XOR | SLL | SRL | SRA | SUB | SLT | SLTU
       | MUL | MULH | MULHSU | MULHU | DIV | DIVU | REM | REMU;
  wire I_valid = ANDI | ORI | ADDI | XORI | SLLI | SRLI | SRAI | SLTI | SLTIU;
  wire B_valid = BEQ | BNE | BLT | BGE | BLTU | BGEU;
  wire L_valid = LW | LH | LB | LHU | LBU;
  wire S_valid = SW | SH | SB;

  // function unit specification
  wire use_ALU = AND | OR | ADD | XOR | SLL | SRL | SRA | SUB | SLT | SLTU
       | I_valid | LUI | AUIPC;
  wire use_MEM = L_valid | S_valid;
  wire use_MUL = MUL | MULH | MULHSU | MULHU;
  wire use_DIV = DIV | DIVU | REM | REMU;
  wire use_JUMP = B_valid | JAL | JALR;


  localparam ALU_ADD  = 4'b0001;
  localparam ALU_SUB  = 4'b0010;
  localparam ALU_AND  = 4'b0011;
  localparam ALU_OR   = 4'b0100;
  localparam ALU_XOR  = 4'b0101;
  localparam ALU_SLL  = 4'b0110;
  localparam ALU_SRL  = 4'b0111;
  localparam ALU_SLT  = 4'b1000;
  localparam ALU_SLTU = 4'b1001;
  localparam ALU_SRA  = 4'b1010;
  localparam ALU_Ap4  = 4'b1011;
  localparam ALU_Bout = 4'b1100;
  wire [3:0] ALU_op;
  assign ALU_op = {4{ADD | ADDI | AUIPC}} & ALU_ADD  |
         {4{SUB}}                & ALU_SUB  |
         {4{AND | ANDI}}         & ALU_AND  |
         {4{OR | ORI}}           & ALU_OR   |
         {4{XOR | XORI}}         & ALU_XOR  |
         {4{SLL | SLLI}}         & ALU_SLL  |
         {4{SRL | SRLI}}         & ALU_SRL  |
         {4{SLT | SLTI}}         & ALU_SLT  |
         {4{SLTU | SLTIU}}       & ALU_SLTU |
         {4{SRA | SRAI}}         & ALU_SRA  |
         {4{LUI}}                & ALU_Bout ;
  localparam JUMP_BEQ  = 4'b0_001;
  localparam JUMP_BNE  = 4'b0_010;
  localparam JUMP_BLT  = 4'b0_011;
  localparam JUMP_BGE  = 4'b0_100;
  localparam JUMP_BLTU = 4'b0_101;
  localparam JUMP_BGEU = 4'b0_110;
  localparam JUMP_JAL  = 4'b0_000;
  localparam JUMP_JALR = 4'b1_000;
  wire[3:0] JUMP_op;
  assign JUMP_op ={4{BEQ}}  & JUMP_BEQ  |
         {4{BNE}}  & JUMP_BNE  |
         {4{BLT}}  & JUMP_BLT  |
         {4{BGE}}  & JUMP_BGE  |
         {4{BLTU}} & JUMP_BLTU |
         {4{BGEU}} & JUMP_BGEU |
         {4{JAL}}  & JUMP_JAL  |
         {4{JALR}} & JUMP_JALR ;

  wire[2:0] inst_fu =  {3{use_ALU}}  & 3'd1 |
      {3{use_MEM}}  & 3'd2 |
      {3{use_MUL}}  & 3'd3 |
      {3{use_DIV}}  & 3'd4 |
      {3{use_JUMP}} & 3'd5 ;

  assign ImmSel = {3{JALR | L_valid | I_valid}} & `Imm_type_I |
         {3{B_valid}}                  & `Imm_type_B |
         {3{JAL}}                      & `Imm_type_J |
         {3{S_valid}}                  & `Imm_type_S |
         {3{LUI | AUIPC}}              & `Imm_type_U ;




  wire[4:0] dst = {5{R_valid | I_valid | L_valid | LUI | AUIPC | JAL | JALR}} & rd;
  wire[4:0] src1 = {5{R_valid | I_valid | S_valid | L_valid | B_valid | JALR}} & rs1;
  wire[4:0] src2 = {5{R_valid | S_valid | B_valid}} & rs2;
  wire ALUSrcA,  ALUSrcB;
  assign ALUSrcA = AUIPC;

  assign ALUSrcB = I_valid | LUI | AUIPC;

  reg mem_ctrl_reg;
  assign mem_ctrl = mem_ctrl_reg;

  reg mem2_ctrl_reg;
  assign mem2_ctrl = mem2_ctrl_reg;

  reg[3:0] use_FU;





  // used in for loop
  integer i;




  reg[FU_NUM:0] busy;
  reg [31:0] vj[0:FU_NUM ];
  reg [31:0] vk[0:FU_NUM ];
  reg [Q_WIDTH - 1:0] qj[0:FU_NUM ];
  reg [Q_WIDTH - 1:0] qk[0:FU_NUM ];
  reg [4:0] rd_RS[0:FU_NUM ];
  reg [31:0] pc_RS[0:FU_NUM ];
  reg [31:0] imm_RS[0:FU_NUM ];
  reg [4:0] CDB_addr;
  reg [31:0] CDB_data;

  reg[Q_WIDTH - 1:0] RRS[0:31];

  reg [FU_NUM :0] CDB_request;
  reg [4:0]register_to_write_reg;

  always @(*)
  begin
    if (inst_fu == 3'd1)
    begin
      if (!busy[ALU_INDEX] | (busy[ALU_INDEX] && CDB_result[ALU_INDEX]))
        use_FU = ALU_INDEX;
      else
        use_FU = ALU2_INDEX;
    end
    else if (inst_fu == 3'd2)
    begin
      if (!busy[MEM_INDEX] | (busy[MEM_INDEX] && (CDB_result[MEM_INDEX] | sd_finish)))
        use_FU = MEM_INDEX;
      else
        use_FU = MEM2_INDEX;
    end
    else if (inst_fu == 3'd3)
    begin
      if (!busy[MUL_INDEX] | (busy[MUL_INDEX] && CDB_result[MUL_INDEX]))
        use_FU = MUL_INDEX;
      else
        use_FU = MUL2_INDEX;
    end
    else
      use_FU = inst_fu;
  end

  //WB
  /******************************WB control***************************/

  /*************************处理总线****************/
  initial
  begin
    CDB_request = 8'b000000;
    for (i=0;i<=FU_NUM;i=i+1)
    begin
      busy[i] <= 1'b0;
      vj[i] <= 32'b0;
      vk[i] <= 32'b0;
      qj[i] <= 5'b0;
      qk[i] <= 5'b0;
      rd_RS[i] <= 5'b0;
      pc_RS[i] <= 32'b0;
      imm_RS[i] <= 32'b0;
    end
    for (i=0;i<32;i=i+1)
    begin
      RRS[i] <= 5'b0;
    end
  end
  always @(*)
  begin
    CDB_request[ALU_INDEX] <= ALU_done;
    CDB_request[MEM_INDEX] <= MEM_done;
    CDB_request[MUL_INDEX] <= MUL_done;
    CDB_request[DIV_INDEX] <= DIV_done;
    CDB_request[JUMP_INDEX] <= JUMP_done;
    CDB_request[ALU2_INDEX] <= ALU2_done;
    CDB_request[MEM2_INDEX] <= MEM2_done;
    CDB_request[MUL2_INDEX] <= MUL2_done;
  end
  //! 各个表都是在Ctrl更新的，而不是各个模�????
  assign register_data_to_write = CDB_data;
  // 总线仲裁
  always @(*)
  begin
    case(1'b1)
      CDB_request[1]:
      begin
        CDB_result <= 9'b00000010;
        CDB_data <= ALU_res;
        CDB_addr <= 4'd1;
        register_to_write_reg <= rd_RS[1];
      end
      CDB_request[2]:
      begin
        CDB_result <= 9'b00000100;
        CDB_data <= MEM_res;
        CDB_addr <= 4'd2;
        register_to_write_reg <= rd_RS[2];
      end
      CDB_request[3]:
      begin
        CDB_result <= 9'b00001000;
        // 补全
        CDB_data <= MUL_res;
        CDB_addr <= 4'd3;
        register_to_write_reg <= rd_RS[3];
      end

      CDB_request[4]:
      begin
        CDB_result <= 9'b00010000;
        CDB_data <= DIV_res;
        CDB_addr <= 4'd4;
        register_to_write_reg <= rd_RS[4];
      end
      CDB_request[5]:
      begin
        CDB_result <= 9'b00100000;
        CDB_data <= JUMP_res;
        CDB_addr <= 4'd5;
        register_to_write_reg <= rd_RS[5];
      end
      CDB_request[6]:
      begin
        CDB_result <= 9'b01000000;
        CDB_data <= ALU2_res;
        CDB_addr <= 4'd6;
        register_to_write_reg <= rd_RS[6];
      end
      CDB_request[7]:
      begin
        CDB_result <= 9'b10000000;
        CDB_data <= MEM2_res;
        CDB_addr <= 4'd7;
        register_to_write_reg <= rd_RS[7];
      end
      CDB_request[8]:
      begin
        CDB_result <= 9'b100000000;
        // 补全
        CDB_data <= MUL2_res;
        CDB_addr <= 4'd8;
        register_to_write_reg <= rd_RS[8];
      end
      default:
      begin
        CDB_result <= 9'b000000;
        CDB_data <= 32'b0;
        CDB_addr <= 4'd0;
        register_to_write_reg <= 5'b0;
      end
    endcase
  end
  /*****************************寄存器控制信号处�????**************/

  assign register_to_write = register_to_write_reg;


  reg register_write_en_reg;
  assign register_write_en = register_write_en_reg;
  // always @ (*)
  // begin
  //   if (rst)
  //     register_write_en_reg <= 1'b0;
  //   else
  //   begin
  //     if ((CDB_result[1] && RRS[rd_RS[1]] == 1) | (CDB_result[2] && !mem_ctrl_reg)  | CDB_result[3] | CDB_result[4] | CDB_result[5])
  //       register_write_en_reg <= 1'b1;
  //     else
  //       register_write_en_reg <= 1'b0;
  //   end
  // end
  reg[Q_WIDTH - 1:0] RRS_old[0:31];

  always @(posedge clk or posedge rst)
  begin
    if (rst)
    begin
      for (i = 0; i < 32; i = i + 1)
      begin
        RRS_old[i] <= 5'b0;
      end
    end
    else
    begin
      for (i = 0; i < 32; i = i + 1)
      begin
        RRS_old[i] <= RRS[i];
      end
    end
  end

  always @ (*)
  begin
    if (rst)
      register_write_en_reg <= 1'b0;
    else
    begin
      if ((CDB_result[1] && (RRS_old[rd_RS[1]] == 1)) | (CDB_result[2] && (!mem_ctrl_reg) && RRS_old[rd_RS[2]] == 2)  | (CDB_result[3] && (RRS_old[rd_RS[3]] == 3)) | (CDB_result[4] && (RRS_old[rd_RS[4]] == 4)) | (CDB_result[5] && (RRS_old[rd_RS[5]] == 5)) | (CDB_result[6] && (RRS_old[rd_RS[6]] == 6)) | (CDB_result[7] && (RRS_old[rd_RS[7]] == 7)&& (!mem2_ctrl_reg) ) | (CDB_result[8] && (RRS_old[rd_RS[8]] == 8)))
        register_write_en_reg <= 1'b1;
      else
        register_write_en_reg <= 1'b0;
    end
  end



  /************************** issue control**********************/
  reg StructureHazard;
  // assign  StructureHazard = use_FU == 2 ? (busy[7] & ~sd_finish2 & ~CDB_result[7]):
  //         use_FU == 5 ? (busy[5] & ~branch_finish & ~CDB_result[5]):
  //         (busy[use_FU] && ~CDB_result[use_FU]);
  always @(*)
  begin
    if (use_FU <= 3 )
      StructureHazard <= 0;
    else if (use_FU == 5)
      StructureHazard <= busy[5] & ~branch_finish & ~CDB_result[5];
    else if (use_FU == 7)
      StructureHazard <= busy[7] & ~sd_finish2 & ~CDB_result[7];
    else
      StructureHazard <= busy[use_FU] & ~CDB_result[use_FU];
  end

  // assign  StructureHazard = busy[use_FU] && ~CDB_result[use_FU];
  wire branch_stall;
  wire jump_in_wb;
  reg[3:0] Jump_op_reg;
  assign IS_en = ~StructureHazard & ~branch_stall;
  // For branch stall
  assign jump_in_wb = !Jump_op_reg[2:0] | Jump_op_reg[3];
  assign branch_stall = busy[JUMP_INDEX] & ~CDB_result[JUMP_INDEX] & ~(branch_finish);
  assign branch_ctrl = (busy[JUMP_INDEX] & branch_finish & cmp_res_FU) | (busy[JUMP_INDEX] & branch_finish & jump_in_wb);
  reg branch_flush_reg;
  assign branch_flush = branch_ctrl;
  // For mem
  reg[2:0] b_h_w_to_mem_reg;
  assign b_h_w_to_mem = b_h_w_to_mem_reg;
  reg[2:0] b_h_w_to_mem2_reg;
  assign b_h_w_to_mem2 = b_h_w_to_mem2_reg;
  reg[3:0] ALU_op_reg;
  reg[3:0] ALU2_op_reg;
  reg ALU_src_a_reg, ALU_src_b_reg;
  assign ALUSrcA_to_fu = ALU_src_a_reg;
  assign ALUSrcB_to_fu =  ALU_src_b_reg;

  reg[9:0] counter;
  always @(posedge clk or posedge rst)
  begin
    counter <= rst ? 10'd0 : counter + 1;
  end

  // 先ALU_src_b_reg想�?�么issue，�?�么stall
  always @(posedge clk or posedge rst)
  begin
    if (rst)
    begin
      // Reset the scoreboard
      for (i = 0; i < 32; i = i + 1)
      begin
        RRS[i] <= 5'b0;
      end

      // Reset other signals
      for (i = 0; i <= FU_NUM; i = i + 1)
      begin
        busy[i] <= 1'b0;
        vj[i] <= 32'b0;
        vk[i] <= 32'b0;
        qj[i] <= 5'b0;
        qk[i] <= 5'b0;
        rd_RS[i] <= 5'b0;
        pc_RS[i] <= 32'b0;
        imm_RS[i] <= 31'b0;
        ALU_op_reg <= 4'b0 ;
        ALU2_op_reg <= 4'b0 ;
        Jump_op_reg <= 4'b0;
        mem_ctrl_reg <= 1'b0;
      end
    end
    else
    begin
      // issue
      for (i = 1; i <= FU_NUM; i=i+1)
      begin
        if (!(branch_stall | branch_ctrl )&& ((CDB_result[i] && i == use_FU) | ((sd_finish) && i == 2 && i == use_FU) | (branch_finish && i == 5 && i == use_FU))) // issue 和 wb 重叠
          // if (CDB_result[i] && i == use_FU) // issue 和 wb 重叠
        begin
          busy[use_FU] <= 1'b1;
          rd_RS[use_FU] <= rd;
          pc_RS[use_FU] <= PC_IS;
          imm_RS[use_FU] <= imm;
          if ((use_FU == ALU_INDEX) || (use_FU == ALU2_INDEX))
          begin
            if (use_FU == ALU_INDEX)
              ALU_op_reg <= ALU_op;
            else
              ALU2_op_reg <= ALU_op;
            ALU_src_a_reg <= ALUSrcA;
            ALU_src_b_reg <= ALUSrcB;
          end
          if (use_FU == JUMP_INDEX)
          begin
            Jump_op_reg <= JUMP_op;
          end

          if (use_FU == MEM_INDEX )
          begin
            b_h_w_to_mem_reg <= inst[14:12];
            mem_ctrl_reg <= S_valid;
          end
          if (use_FU ==  MEM2_INDEX)
          begin
            b_h_w_to_mem2_reg <= inst[14:12];
            mem2_ctrl_reg <= S_valid;
          end
          if (~S_valid & dst != 0) // Write RRS
          begin
            RRS[rd] <= use_FU;
          end
          if (dst != register_to_write && RRS[register_to_write] == i)
            RRS[register_to_write] <= 0;
          // Write qk,qj,vj,vk
          if (src1 == register_to_write && src1 != 0)
          begin
            vj[use_FU] <= CDB_data;
            qj[use_FU] <= 5'b0;
          end
          else if (!RRS[src1] && src1 != 0) // blank
          begin
            vj[use_FU] <= rs1_from_reg;
            qj[use_FU] <= 5'b0;
          end
          else
          begin
            vj[use_FU] <= 32'b0;
            qj[use_FU] <= RRS[src1];
          end

          if (src2 == register_to_write && src2 != 0)
          begin
            vk[use_FU] <= CDB_data;
            qk[use_FU] <= 5'b0;
          end
          else if (!RRS[src2] && src2 != 0) // blank
          begin
            vk[use_FU] <= rs2_from_reg;
            qk[use_FU] <= 5'b0;
          end
          else
          begin
            vk[use_FU] <= 32'b0;
            qk[use_FU] <= RRS[src2];
          end
        end
        else if (CDB_result[i] | (sd_finish && i == MEM_INDEX ) | (sd_finish2 && i == MEM2_INDEX ) |((branch_finish && ~jump_in_wb) && i == 5  )) // wb
          // else if (CDB_result[i]) // wb
        begin
          busy[i] <= 1'b0;
          if (RRS[register_to_write] == i && dst != register_to_write)
            RRS[register_to_write] <= 0;
          rd_RS[i] <= 5'b0;
          imm_RS[i] <= 32'b0;
          pc_RS[i] <= 32'b0;
          vj[i] <= 32'b0;
          vk[i] <= 32'b0;
          if (i == use_JUMP)
            Jump_op_reg <= 4'b0111;
        end
        else if (i == use_FU && !busy[i] && !(branch_stall | branch_ctrl )) //issue
        begin
          busy[use_FU] <= 1'b1;
          // write vj,vk,qj,qk,
          if (!RRS[src1] && src1 != 0) // blank
          begin
            vj[use_FU] <= rs1_from_reg;
            qj[use_FU] <= 5'b0;
          end
          else
          begin
            vj[use_FU] <= 32'b0;
            qj[use_FU] <= RRS[src1];
          end
          if (!RRS[src2] && src2 != 0) // blank, if src==0, then q = 0
          begin
            vk[use_FU] <= rs2_from_reg;
            qk[use_FU] <= 5'b0;
          end
          else
          begin
            vk[use_FU] <= 32'b0;
            qk[use_FU] <= RRS[src2];
          end
          // write rd,pc,imm
          rd_RS[use_FU] <= rd;
          pc_RS[use_FU] <= PC_IS;
          imm_RS[use_FU] <= imm;
          // write RRS
          if (~S_valid & dst != 0)
            RRS[dst] <= use_FU;

          if ((use_FU == ALU_INDEX) || (use_FU == ALU2_INDEX))
          begin
            if (use_FU == ALU_INDEX)
              ALU_op_reg <= ALU_op;
            else
              ALU2_op_reg <= ALU_op;
            ALU_src_a_reg <= ALUSrcA;
            ALU_src_b_reg <= ALUSrcB;
          end
          if (use_FU == JUMP_INDEX)
          begin
            Jump_op_reg <= JUMP_op;
          end

          if (use_FU == MEM_INDEX )
          begin
            b_h_w_to_mem_reg <= inst[14:12];
            mem_ctrl_reg <= S_valid;
          end
          if (use_FU ==  MEM2_INDEX)
          begin
            b_h_w_to_mem2_reg <= inst[14:12];
            mem2_ctrl_reg <= S_valid;
          end
        end
        else if(CDB_result != 0)
        begin
          if (busy[i])
          begin
            if (qj[i] != 0 && qj[i] == CDB_addr)
            begin
              vj[i] <= CDB_data;
              qj[i] <= 4'b0;
            end
            if (qk[i] != 0 && qk[i] == CDB_addr)
            begin
              vk[i] <= CDB_data;
              qk[i] <= 4'b0;
            end
          end
        end
      end
      // WB
    end
  end


  assign ALU_op_to_fu = ALU_op_reg;
  assign ALU2_op_to_fu = ALU2_op_reg;
  assign Jump_op_to_fu = Jump_op_reg;


  /************************** exe control  ***********************/
  assign ALU_en = busy[ALU_INDEX] && !qj[ALU_INDEX] && !qk[ALU_INDEX] ;
  assign ALU2_en = busy[ALU2_INDEX] && !qj[ALU2_INDEX] && !qk[ALU2_INDEX] ;
  assign MUL_en = busy[MUL_INDEX] && !qj[MUL_INDEX] && !qk[MUL_INDEX] ;
  assign MUL2_en = busy[MUL2_INDEX] && !qj[MUL2_INDEX] && !qk[MUL2_INDEX] ;
  assign DIV_en = busy[DIV_INDEX] && !qj[DIV_INDEX] && !qk[DIV_INDEX] ;
  assign MEM_en = busy[MEM_INDEX] && !qj[MEM_INDEX] && !qk[MEM_INDEX] ;
  assign MEM2_en = busy[MEM2_INDEX] && !qj[MEM2_INDEX] && !qk[MEM2_INDEX] ;
  assign JUMP_en = busy[JUMP_INDEX] && !qj[JUMP_INDEX] && !qk[JUMP_INDEX] ;
  assign rs1_to_alu = vj[ALU_INDEX];
  assign rs2_to_alu = vk[ALU_INDEX];
  assign imm_to_alu = imm_RS[ALU_INDEX];
  assign pc_to_alu = pc_RS[ALU_INDEX];

  assign rs1_to_alu2 = vj[ALU2_INDEX];
  assign rs2_to_alu2 = vk[ALU2_INDEX];
  assign imm_to_alu2 = imm_RS[ALU2_INDEX];
  assign pc_to_alu2 = pc_RS[ALU2_INDEX];

  assign rs1_to_mul = vj[MUL_INDEX];
  assign rs2_to_mul = vk[MUL_INDEX];
  assign rs1_to_mul2 = vj[MUL2_INDEX];
  assign rs2_to_mul2 = vk[MUL2_INDEX];

  assign rs1_to_div = vj[DIV_INDEX];
  assign rs2_to_div = vk[DIV_INDEX];

  assign rs2_to_mem = vk[MEM_INDEX];
  assign imm_to_mem = imm_RS[MEM_INDEX];
  assign rs1_to_mem = vj[MEM_INDEX];

  assign rs2_to_mem2= vk[MEM2_INDEX];
  assign imm_to_mem2= imm_RS[MEM2_INDEX];
  assign rs1_to_mem2= vj[MEM2_INDEX];

  // output[31:0] rs1_to_jump, rs2_to_jump, imm_to_jump,pc_to_jump,
  assign rs1_to_jump = vj[JUMP_INDEX];
  assign rs2_to_jump = vk[JUMP_INDEX];
  assign imm_to_jump = imm_RS[JUMP_INDEX];
  assign pc_to_jump = pc_RS[JUMP_INDEX];
  // jal x[rd] = pc + 4, pc += offset
  // jalr x[rd] = pc + 4, pc = x[rs1] + offset
  // branch pc += offset
  // always @(posedge clk or posedge rst)
  // begin
  //   if (rst)
  //   begin
  //     rs1_to_fu_reg<=32'b0;
  //     rs2_to_fu_reg<=32'b0;
  //   end
  //   else
  //   begin
  //     if (ALU_en)
  //     begin
  //       rs1_to_fu_reg<=vj[ALU_INDEX];
  //       rs2_to_fu_reg<=vk[ALU_INDEX];
  //     end
  //     if (MUL_en)
  //     begin
  //       rs1_to_fu_reg<=vj[MUL_INDEX];
  //       rs2_to_fu_reg<=vk[MUL_INDEX];
  //     end
  //     if (DIV_en)
  //     begin
  //       rs1_to_fu_reg<=vj[DIV_INDEX];
  //       rs2_to_fu_reg<=vk[DIV_INDEX];
  //     end
  //     if (MEM_en)
  //     begin
  //       rs1_to_fu_reg<=vj[MEM_INDEX];
  //       rs2_to_fu_reg<=vk[MEM_INDEX];
  //     end
  //     if (JUMP_en)
  //     begin
  //       rs1_to_fu_reg<=vj[MEM_INDEX];
  //       rs2_to_fu_reg<=vk[MEM_INDEX];
  //     end
  //   end
  // end



  //------------------------------------------------------------------------------------



  //  VGA Display  (see VGATEST.v line208-246)

  //  |  PC---IF    (32'h-PC)   |  INST-IF  (32'h-Inst) |  PC---IS    (32'h-PC)   |  INST-IS  (32'h-Inst) |

  parameter SEPERATION = 4'HF;
  //  F is seperation character in the following lines

  //  |  1-ALU-I    (32'h-Inst) |  B/D/WAR -FFF-FF-     |  F/R/Q/j  --FF-FF-      |    F/R/Q/k  --FF-FF-  |
  //  |  2-MEM-I    ...
  //  |  3-MUL-I    ...
  //  |  4-DIV-I
  //  |  5-JMP-I

  //  RRS -F-F-F-F
  //  |  R/01-03  |   R/04-07 |   R/08-11 |   R/12-15 |
  //  |  R/16-19  |   R/20-23 |   R/24-27 |   R/28/31 |

  reg[31:0] Test_signal;



endmodule
