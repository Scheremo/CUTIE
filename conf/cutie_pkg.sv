`include "register_interface/typedef.svh"
`include "axi/typedef.svh"

package cutie_pkg;
	// regif channels declaration
	typedef logic [31:0] addr_t;
	typedef logic [31:0] data_t;
	typedef logic [32/8-1:0] strb_t;
	`REG_BUS_TYPEDEF_REQ(reg_req_t, addr_t, data_t, strb_t)
	`REG_BUS_TYPEDEF_RSP(reg_rsp_t, data_t)

	// AXI lite structure declaration
	typedef logic [7:0]      byte_t;
	typedef logic [31:0]     axi_addr_t;
	typedef logic [31:0]     axi_data_t;
	typedef logic [32/8-1:0] axi_strb_t;
	`AXI_LITE_TYPEDEF_ALL(axi_lite, axi_addr_t, axi_data_t, axi_strb_t)


	typedef struct packed {
		logic          req;
		logic [31:0]   add;
		logic          wen;
		logic [31:0]   wdata;
		logic [3:0]    be;
	} tcdm_req_t;
	typedef struct packed {
		logic         gnt;
		logic         r_opc;
		logic [31:0]  r_rdata;
		logic         r_valid;
	}tcdm_rsp_t;

endpackage
