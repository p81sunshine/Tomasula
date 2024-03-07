// `timescale 1ns / 1ps

// module FU_div(
//     input clk, EN,
//     input[31:0] A, B,
//     output[31:0] res_u,
//     input[5:0] CDB_result,
//     output finish
//   );

//   wire res_valid;
//   wire[63:0] divres;
//   reg[31:0] res_reg;
//   reg finish_reg;
//   reg[1:0] state;
//   initial
//   begin
//     state = 2'b0;
//     res_reg = 32'b0;
//   end

//   assign finish = finish_reg;
//   assign res_u = res_reg;
//   reg A_valid, B_valid;
//   reg[31:0] A_reg, B_reg;


//   initial
//   begin
//     finish_reg = 1'b0;
//   end
//   always@(posedge clk)
//   begin
//     if(EN & ~state)
//     begin //state == 0
//       A_reg <= A;
//       A_valid <= 1;
//       B_reg <= B;
//       B_valid <= 1;
//       state <= 2'b1;
//     end
//     else
//     begin
//       if (res_valid)
//       begin
//         res_reg <= res;
//         finish_reg <= 1'b1;
//       end
//       if (CDB_result[4])
//       begin
//         res_reg <= 32'b0;
//         finish_reg <= 1'b0;
//         state <= 2'b0;
//       end
//     end
//   end


//   divider div(.aclk(clk),
//               .s_axis_dividend_tvalid(A_valid),
//               .s_axis_dividend_tdata(A_reg),
//               .s_axis_divisor_tvalid(B_valid),
//               .s_axis_divisor_tdata(B_reg),
//               .m_axis_dout_tvalid(res_valid),
//               .m_axis_dout_tdata(divres)
//              );

//   assign res = divres[63:32];

// endmodule

`timescale 1ns / 1ps

module FU_div(
    input clk, EN,
    input[31:0] A, B,
    output[31:0] res_u,
    input[5:0] CDB_result,
    output finish
  );

  wire[63:0] divres;
  reg finish_reg;
  reg[1:0] state;
  reg[5:0] counter; // 添加一个计数器


  assign finish = finish_reg;
  reg A_valid, B_valid;
  reg[31:0] A_reg, B_reg;

  initial
  begin
    state = 2'b0;
    counter = 6'b0; // 初始化计数器
    finish_reg = 1'b0;
    A_valid = 1'b0;
    B_valid = 1'b0;
    A_reg = 32'b0;
    B_reg = 32'b0;
  end

  always@(posedge clk)
  begin
    case (state)
      2'b00:
      begin //state == 0
        if(EN)
        begin
          A_reg <= A;
          A_valid <= 1;
          B_reg <= B;
          B_valid <= 1;
          state <= 2'b01;
          counter <= 6'b0; // 重置计数器
        end
      end
      2'b01:
      begin //state == 1
        if (counter == 6'd22) // 如果计数器达到22
        begin
          finish_reg <= 1'b1;
          state <= 2'b10; // 进入新的状态
        end
        else
        begin
          counter <= counter + 1; // 增加计数器
        end
      end
      2'b10:
      begin //state == 2
        if (CDB_result[4])
        begin
          finish_reg <= 1'b0;
          state <= 2'b00;
        end
      end
      default:
        state <= 2'b00;
    endcase
  end


  wire res_valid;
  divider div(.aclk(clk),
              .s_axis_dividend_tvalid(A_valid),
              .s_axis_dividend_tdata(A_reg),
              .s_axis_divisor_tvalid(B_valid),
              .s_axis_divisor_tdata(B_reg),
              .m_axis_dout_tvalid(res_valid),
              .m_axis_dout_tdata(divres)
             );

  assign res_u = divres[63:32];

endmodule
