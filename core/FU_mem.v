`timescale 1ns / 1ps

module FU_mem(
    input clk, EN, mem_w,
    input[2:0] bhw,
    input[31:0] rs1_data, rs2_data, imm,
    output[31:0] mem_data,
    output finish,
    output finish_sd,
    input[8:0] CDB_result,
    input[3:0] index
  );

  reg [1:0]state;
  initial
  begin
    state = 0;
  end
  wire stall;
  wire ack;
  wire[31:0] data;

  reg mem_w_reg;
  reg[2:0] bhw_reg;
  reg[31:0] rs1_data_reg, rs2_data_reg, imm_reg;
  reg[31:0] addr;
  reg [31:0] mem_data_reg;
  assign mem_data = mem_data_reg;



  reg finish_reg;
  reg finish_sd_reg;
  assign finish_sd = finish_sd_reg;

  reg mem_w_next;
  initial
  begin
    finish_reg = 1'b0;
    finish_sd_reg = 1'b0;
    mem_w_next = 1'b0;
  end
  assign finish = finish_reg;
  always@(posedge clk)
  begin
    if(EN & (state == 0))
    begin //state == 0
      rs1_data_reg <= rs1_data;
      rs2_data_reg <= rs2_data;
      imm_reg <= imm;
      mem_w_reg <= mem_w;
      mem_w_next <= mem_w_reg;
      bhw_reg <= bhw;
      addr <= imm + rs1_data;
      state <= 2'd1;
      finish_sd_reg <= mem_w;
    end
    else if (state == 2'd1)
    begin
      if (!mem_w_reg)
      begin
        mem_data_reg <= data;
        finish_reg <= 1'b1;
        state <= 2'd2;
        mem_w_reg <= 1'b0;
        mem_w_next <= mem_w_reg;
      end
      else
      begin
        mem_data_reg <= 32'b0;
        finish_reg <= 1'b0;
        mem_w_reg <= 1'b0;
        finish_sd_reg <= 1'b0;
        mem_w_next <= mem_w_reg;
        state <= 0;
      end
    end
    else if (state == 2'd2)
    begin
      if (CDB_result[index])
      begin
        state <= 0;
        mem_data_reg <= 32'b0;
        finish_reg <= 1'b0;
      end
    end
  end


  RAM_B ram(.clka(clk),.addra(addr),.dina(rs2_data_reg),.wea(mem_w_next),
            .douta(data),.mem_u_b_h_w(bhw_reg));

endmodule
