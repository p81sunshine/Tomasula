`timescale 1ns / 1ps

module  RV32core(
    input debug_en,  // debug enable
    input debug_step,  // debug step clock
    input [6:0] debug_addr,  // debug address
    output[31:0] debug_data,  // debug data
    input clk,  // main clock
    input rst,  // synchronous reset
    input interrupter  // interrupt source, for future use
  );

  wire debug_clk;
  wire[31:0] debug_regs;
  wire[31:0] Test_signal;
  wire[4:0] Jump_ctrl;
  wire cmp_res_to_ctrl;
  wire register_write_en;
  wire[4:0] register_to_write;
  assign debug_data = debug_addr[5] ? Test_signal : debug_regs;

  debug_clk clock(.clk(clk),.debug_en(debug_en),.debug_step(debug_step),.debug_clk(debug_clk));

  wire PC_EN_IF, IS_EN, FU_ALU_EN, FU_mem_EN, FU_mul_EN, FU_div_EN, FU_jump_EN;
  wire[2:0] ImmSel_ctrl, bhw_ctrl;

  wire FU_ALU_finish, FU_mem_finish, FU_mul_finish, FU_div_finish, FU_jump_finish, is_jump_FU;
  wire rs1_to_fu, rs2_to_fu;

  wire[3:0] ALU_op_to_fu;
  wire[31:0] ALU_out_fu, MEM_data_fu, mulres_FU, divres_FU, PC_jump_FU, PC_wb_FU;
  wire[31:0] ALU2_out_fu, MEM2_data_fu, mulres2_FU;
  wire ALUSrcA_ctrl, ALUSrcB_ctrl;
  wire [31:0] rs1_to_alu, rs2_to_alu, imm_to_alu, pc_to_alu;
  wire [31:0] rs1_to_mul, rs2_to_mul;
  wire [31:0] rs1_to_div, rs2_to_div;
  wire [31:0] rs2_to_mem, imm_to_mem ,rs1_to_mem;
  wire [31:0] rs1_to_jump, rs2_to_jump, imm_to_jump,pc_to_jump;
  wire [2:0]b_hw_to_mem;
  wire mem_write_en;
  wire sd_finish;
  wire branch_finish;

  // alu2
  wire FU_ALU2_EN, FU_ALU2_finish;
  wire[3:0] ALU2_op_to_fu;
  wire [31:0] rs1_to_alu2, rs2_to_alu2, imm_to_alu2, pc_to_alu2;

  // mem2
  wire [31:0] rs2_to_mem2, imm_to_mem2 ,rs1_to_mem2;
  wire sd_finish2;
  wire mem_write_en2;
  wire FU_mem2_EN, FU_mem2_finish;
  wire[2:0] b_hw_to_mem2;

  // mul2
  wire [31:0] rs1_to_mul2, rs2_to_mul2;
  wire FU_mul2_finish;
  wire FU_mul2_EN;
  //  ALU

  //  Register file JUMP

  wire[31:0] PC_ctrl_JUMP ;

  //  Register file MEM

  //  Register file MUL

  //  Register file DIV


  wire [31:0] PC_IF, next_PC_IF, PC_4_IF, inst_IF;

  wire[31:0]inst_IS, PC_IS, Imm_out_IS;

  wire[31:0]ALUA_RO, ALUB_RO;




  // IF
  assign PC_EN_IF = IS_EN ;

  REG32 REG_PC(.clk(debug_clk),.rst(rst),.CE(PC_EN_IF),.D(next_PC_IF),.Q(PC_IF));

  add_32 add_IF(.a(PC_IF),.b(32'd4),.c(PC_4_IF));

  wire branch_ctrl;
  MUX2T1_32 mux_IF(.I0(PC_4_IF),.I1(PC_jump_FU),.s(branch_ctrl),.o(next_PC_IF));

  ROM_D inst_rom(.a(PC_IF[8:2]),.spo(inst_IF));


  parameter FU_NUM = 8;
  wire [FU_NUM : 0] CDB_result;
  wire[4:0] CDB_addr;
  wire[31:0] CDB_data;

  wire [31:0] rs1_data, rs2_data;
  Regs register(.clk(debug_clk),.rst(rst),
                .R_addr_A(inst_IS[19:15]),.rdata_A(rs1_data),
                .R_addr_B(inst_IS[24:20]),.rdata_B(rs2_data),
                .EN(register_write_en),.Wt_addr(register_to_write),.Wt_data(CDB_data),
                .Debug_addr(debug_addr[4:0]),.Debug_regs(debug_regs));

  wire branch_flush;
  //Issue
  REG_IF_IS reg_IF_IS(.clk(debug_clk),.rst(rst),.EN(IS_EN),
                      .flush(branch_flush),.PCOUT(PC_IF),.IR(inst_IF),

                      .IR_IS(inst_IS),.PCurrent_IS(PC_IS));

  ImmGen imm_gen(.ImmSel(ImmSel_ctrl),.inst_field(inst_IS),.Imm_out(Imm_out_IS));

  CtrlUnit ctrl(.clk(debug_clk), .rst(rst), .PC_IF(PC_IF), .inst_IF(inst_IF),
                .PC_IS(PC_IS), .inst(inst_IS), .imm(Imm_out_IS),
                // IS
                .IS_en(IS_EN),
                .CDB_result(CDB_result),
                // imm
                .ImmSel(ImmSel_ctrl),
                // FU control
                .ALU_en(FU_ALU_EN), .MEM_en(FU_mem_EN), .MUL_en(FU_mul_EN), .DIV_en(FU_div_EN), .JUMP_en(FU_jump_EN),
                .ALU_done(FU_ALU_finish), .MEM_done(FU_mem_finish), .MUL_done(FU_mul_finish), .DIV_done(FU_div_finish), .JUMP_done(FU_jump_finish),
                // ALU2
                .ALU2_done(FU_ALU2_finish), .ALU2_en(FU_ALU2_EN),
                .ALU2_res(ALU2_out_fu),
                .ALU2_op_to_fu(ALU2_op_to_fu),
                .rs1_to_alu2(rs1_to_alu2), .rs2_to_alu2(rs2_to_alu2),.imm_to_alu2(imm_to_alu2),.pc_to_alu2(pc_to_alu2),
                //ALU
                .ALU_res(ALU_out_fu), .MEM_res(MEM_data_fu), .MUL_res(mulres_FU), .DIV_res(divres_FU), .JUMP_res(PC_wb_FU),
                .ALU_op_to_fu(ALU_op_to_fu),
                .Jump_op_to_fu(Jump_ctrl),
                .ALUSrcB_to_fu(ALUSrcB_ctrl),
                .ALUSrcA_to_fu(ALUSrcA_ctrl),
                .rs1_to_alu(rs1_to_alu), .rs2_to_alu(rs2_to_alu),.imm_to_alu(imm_to_alu),.pc_to_alu(pc_to_alu),
                //mul
                .rs1_to_mul(rs1_to_mul), .rs2_to_mul(rs2_to_mul),
                // mul2
                .rs1_to_mul2(rs1_to_mul2), .rs2_to_mul2(rs2_to_mul2),
                .MUL2_res(mulres2_FU),.MUL2_done(FU_mul2_finish), .MUL2_en(FU_mul2_EN),
                //div
                .rs1_to_div(rs1_to_div), .rs2_to_div(rs2_to_div),
                //mem
                .rs1_to_mem(rs1_to_mem), .rs2_to_mem(rs2_to_mem), .imm_to_mem(imm_to_mem),
                .b_h_w_to_mem(b_hw_to_mem),.mem_ctrl(mem_write_en),
                .sd_finish(sd_finish),
                //mem2
                .rs1_to_mem2(rs1_to_mem2), .rs2_to_mem2(rs2_to_mem2), .imm_to_mem2(imm_to_mem2),
                .b_h_w_to_mem2(b_hw_to_mem2),.mem2_ctrl(mem_write_en2),
                .sd_finish2(sd_finish2), .MEM2_res(MEM2_data_fu), .MEM2_done(FU_mem2_finish),.MEM2_en(FU_mem2_EN),
                //jump
                .rs1_to_jump(rs1_to_jump), .rs2_to_jump(rs2_to_jump), .imm_to_jump(imm_to_jump), .pc_to_jump(pc_to_jump),
                .branch_flush(branch_flush),
                // register
                .rs1_from_reg(rs1_data), .rs2_from_reg(rs2_data),
                .register_write_en(register_write_en), .register_to_write(register_to_write),
                .register_data_to_write(CDB_data),
                //branch
                .cmp_res_FU(cmp_res_to_ctrl),
                .branch_ctrl(branch_ctrl),
                .branch_finish(branch_finish),

                // debug
                .debug_addr(debug_addr),
                .Testout(Test_signal)
               );


  wire [31:0] ALUA_ID, ALUB_ID;
  wire [31:0] ALUA2_ID, ALUB2_ID;
  MUX2T1_32 mux_imm_ALU_ID_A(.I0(rs1_to_alu),.I1(pc_to_alu),.s(ALUSrcA_ctrl),.o(ALUA_ID));       // FILLED

  MUX2T1_32 mux_imm_ALU_ID_B(.I0(rs2_to_alu),.I1(imm_to_alu),.s(ALUSrcB_ctrl),.o(ALUB_ID));  // FILLED

  MUX2T1_32 mux_imm_ALU2_ID_A(.I0(rs1_to_alu2),.I1(pc_to_alu2),.s(ALUSrcA_ctrl),.o(ALUA2_ID));       // FILLED

  MUX2T1_32 mux_imm_ALU2_ID_B(.I0(rs2_to_alu2),.I1(imm_to_alu2),.s(ALUSrcB_ctrl),.o(ALUB2_ID));  // FILLED
  // FU

  FU_ALU alu(.clk(debug_clk),.EN(FU_ALU_EN),.finish(FU_ALU_finish), .index(4'd1),
             .Control(ALU_op_to_fu),.A(ALUA_ID),.B(ALUB_ID),.res_out(ALU_out_fu),
             .zero(),.overflow(),.CDB_result(CDB_result));

  FU_ALU alu2(.clk(debug_clk),.EN(FU_ALU2_EN),.finish(FU_ALU2_finish),.index(4'd6),
              .Control(ALU2_op_to_fu),.A(ALUA2_ID),.B(ALUB2_ID),.res_out(ALU2_out_fu),
              .zero(),.overflow(),.CDB_result(CDB_result));

  FU_mem mem(.clk(debug_clk),.EN(FU_mem_EN),
             .bhw(b_hw_to_mem),.rs1_data(rs1_to_mem),.rs2_data(rs2_to_mem),.imm(imm_to_mem),
             .mem_data(MEM_data_fu),.finish(FU_mem_finish),.CDB_result(CDB_result),.mem_w(mem_write_en),
             .finish_sd(sd_finish), .index(4'd2)
            );
  FU_mem mem2(.clk(debug_clk),.EN(FU_mem2_EN),
              .bhw(b_hw_to_mem2),.rs1_data(rs1_to_mem2),.rs2_data(rs2_to_mem2),.imm(imm_to_mem2),
              .mem_data(MEM2_data_fu),.finish(FU_mem2_finish),.CDB_result(CDB_result),.mem_w(mem_write_en2),
              .finish_sd(sd_finish2), .index(4'd7)
             );

  FU_mul mu(.clk(debug_clk),.EN(FU_mul_EN),.finish(FU_mul_finish),
            .A(rs1_to_mul),.B(rs2_to_mul),.res(mulres_FU), .CDB_result(CDB_result),.index(4'd3)
           );

  FU_mul mu2(.clk(debug_clk),.EN(FU_mul2_EN),.finish(FU_mul2_finish),
             .A(rs1_to_mul2),.B(rs2_to_mul2),.res(mulres2_FU), .CDB_result(CDB_result), .index(4'd2)
            );
  FU_div du(.clk(debug_clk),.EN(FU_div_EN),.finish(FU_div_finish),
            .A(rs1_to_div),.B(rs2_to_div),.res_u(divres_FU), .CDB_result(CDB_result));


  FU_jump ju(.clk(debug_clk),.EN(FU_jump_EN),.finish(FU_jump_finish),
             .JALR(Jump_ctrl[3]),.cmp_ctrl(Jump_ctrl[2:0]),.rs1_data(rs1_to_jump),.rs2_data(rs2_to_jump),
             .imm(imm_to_jump),.PC(pc_to_jump),.PC_jump(PC_jump_FU),.PC_wb(PC_wb_FU),.cmp_res(cmp_res_to_ctrl),
             .CDB_result(CDB_result), .branch_finish(branch_finish));


endmodule
