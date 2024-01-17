//**********************************************************
//**************** XBAR TCDM BUS ***************************
//**********************************************************

interface XBAR_TCDM_BUS;

   // REQUEST CHANNEL
   //***************************************
   logic          req;
   logic [31:0]   add;
   logic          wen;
   logic [31:0]   wdata;
   logic [3:0]    be;
   logic          gnt;

   // RESPONSE CHANNEL
   logic          r_opc;
   logic [31:0]   r_rdata;
   logic          r_valid;

   // Master Side
   //***************************************
   modport Master
     (
      output req, output add, output wen, output wdata, output be,
      input  gnt, input r_rdata, input r_opc, input r_valid
      );

   // Slave Side
   //***************************************
   modport Slave
     (
      input  req, input add, input wen, input wdata, input be,
      output gnt, output r_rdata, output r_opc, output r_valid
      );

endinterface // XBAR_TCDM_BUS

interface XBAR_TCDM_BUS_DV(input logic clk_i);

   // REQUEST CHANNEL
   //***************************************
   logic     req;
   logic [31:0] add;
   logic        wen;
   logic [31:0] wdata;
   logic [3:0]  be;
   logic        gnt;

   // RESPONSE CHANNEL
   logic        r_opc;
   logic [31:0] r_rdata;
   logic        r_valid;

   // Master Side
   //***************************************
   modport Master
     (
      output req, output add, output wen, output wdata, output be,
      input  gnt, input r_rdata, input r_opc, input r_valid
      );

   // Slave Side
   //***************************************
   modport Slave
     (
      input  req, input add, input wen, input wdata, input be,
      output gnt, output r_rdata, output r_opc, output r_valid
      );

endinterface // XBAR_TCDM_BUS

