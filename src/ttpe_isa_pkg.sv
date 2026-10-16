package ttpe_isa_pkg;
  localparam logic [3:0] REG_ANDI  = 4'd9;
  localparam logic [3:0] REG_ORI   = 4'd10;
  localparam logic [3:0] REG_ADDI  = 4'd11;

  function automatic [3:0]  f_op (input [31:0] i); return i[31:28]; endfunction
  function automatic [3:0]  f_a  (input [31:0] i); return i[27:24]; endfunction
  function automatic [7:0]  f_p  (input [31:0] i); return i[23:16]; endfunction
  function automatic [15:0] f_d  (input [31:0] i); return i[15:0];  endfunction
endpackage
