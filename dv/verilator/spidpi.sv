
module spidpi(
  input  logic sck,
  input  logic copi,
  output logic cipo
);

chandle ctx;

import "DPI-C" function
  chandle spidpi_create();

import "DPI-C" function
  void spidpi_sckEdge(input chandle ctx, input bit copi);

initial begin
  ctx = spidpi_create();
end

always @(posedge sck) begin
  spidpi_sckEdge(ctx, copi);
end

// CIPO not required at present
assign cipo = 1'b0;

endmodule
