`timescale 1ns / 1ps

module FU_mul(
    input clk, EN,
    input[31:0] A, B,
    output[31:0] res,
    input[8:0] CDB_result,
    output finish,
    input[3:0] index
  );

  reg[6:0] counter;
  reg [1:0] actual_state;
  reg [31:0] res_reg;
  reg finish_reg;
  wire [63:0] mulres;
  assign  finish = finish_reg;
  // assign res = res_reg;
  reg[31:0] A_reg, B_reg;

  initial
  begin
    counter = 0;
    actual_state = 0;
    res_reg = 0;
    finish_reg = 0;
  end


  wire [31:0] res_t;

  always@(posedge clk)
  begin
    if(EN & actual_state == 0)
    begin //state == 0
      A_reg <= A;
      B_reg <= B;
      counter[5] <= 1;
      actual_state <= 2'b1;
    end
    if (actual_state== 1)
    begin
      counter <= (counter >> 1);
      res_reg <= res_t;
      if (counter == 1)
      begin
        actual_state <= 2'd2;
        finish_reg <= 1'b1;
        counter <= 0;
      end
    end
    if (actual_state == 2'd2)
    begin
      if (CDB_result[index])
      begin
        actual_state <= 2'b0;
        finish_reg <= 1'b0;
        res_reg <= 0;
      end
    end
  end


  multiplier mul(.CLK(clk),.A(A_reg),.B(B_reg),.P(mulres));



  assign res = mulres[31:0];

endmodule
