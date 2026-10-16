package ttpe_isa_pkg;
  localparam logic [3:0] REG_ANDI  = 4'd9;
  localparam logic [3:0] REG_ORI   = 4'd10;
  localparam logic [3:0] REG_ADDI  = 4'd11;

  function [3:0] f_op; input [31:0] i; f_op = i[31:28]; endfunction
  function [3:0] f_a;  input [31:0] i; f_a  = i[27:24]; endfunction
  function [7:0] f_p;  input [31:0] i; f_p  = i[23:16]; endfunction
  function [15:0] f_d; input [31:0] i; f_d  = i[15:0];  endfunction
endpackage
