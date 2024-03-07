`timescale 1ns / 1ps

module FU_jump(
    input clk, EN, JALR,
    input[2:0] cmp_ctrl,
    input[31:0] rs1_data, rs2_data, imm, PC,
    output[31:0] PC_jump, PC_wb,
    output cmp_res,
    output finish,
    output branch_finish,
    input[8:0] CDB_result
  );

  reg state;
  reg finish_reg;
  reg JALR_reg;
  reg[2:0] cmp_ctrl_reg;
  reg[31:0] rs1_data_reg, rs2_data_reg, imm_reg, PC_reg;
  reg branch_finish_reg;
  assign finish = finish_reg;
  assign branch_finish = branch_finish_reg;


  initial
  begin
    state = 0;
    finish_reg = 0;
    PC_reg = 0;
    branch_finish_reg = 0;
  end

  wire jump;
  assign  jump = JALR_reg | !cmp_ctrl_reg;
  always@(posedge clk)
  begin
    if(EN & ~state)
    begin //state == 0
      JALR_reg <= JALR;
      cmp_ctrl_reg <= cmp_ctrl;
      rs1_data_reg <= rs1_data;
      rs2_data_reg <= rs2_data;
      imm_reg <= imm;
      PC_reg <= PC;
      state <= 1;
      branch_finish_reg <= 1;
    end
    else
    begin
      if (state == 1)
      begin
        if (branch_finish_reg && ~jump)
        begin
          state <= 0;
          finish_reg <= 0;
          branch_finish_reg <= 0;
        end
        else
        begin
          finish_reg <= 1;
          branch_finish_reg <= 0;
          if (CDB_result[5])
          begin
            state <= 0;
            finish_reg <= 0;
          end
        end
      end
    end
  end

  cmp_32 cmp(.a(rs1_data_reg), .b(rs2_data_reg), .ctrl(cmp_ctrl_reg), .c(cmp_res));  // 输出比较结果
  assign PC_jump = JALR_reg ? (rs1_data_reg + imm_reg) : (PC_reg + imm_reg); // JALR：JAL两种方式跳转地址不同
  assign PC_wb = PC_reg + 32'd4;


endmodule
