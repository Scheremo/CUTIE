`include "mem_map.svh"

module tcdm_to_mem
  import cutie_params::*;

  #(
    parameter K = cutie_params::K,
    parameter N_I = cutie_params::N_I,
    parameter N_O = cutie_params::N_O,
    parameter IMAGEWIDTH = cutie_params::IMAGEWIDTH,
    parameter IMAGEHEIGHT = cutie_params::IMAGEHEIGHT,
    parameter WEIGHT_STAGGER = cutie_params::WEIGHT_STAGGER,
    parameter NUMACTMEMBANKSETS = cutie_params::NUMACTMEMBANKSETS,
    parameter WEIGHTBANKDEPTH = cutie_params::WEIGHTBANKDEPTH,
    parameter int unsigned NUM_LAYERS = cutie_params::NUM_LAYERS,
    parameter int unsigned BANKSETSBITWIDTH = NUMACTMEMBANKSETS > 1 ? $clog2(NUMACTMEMBANKSETS) : 1,
    parameter int unsigned NUMBANKS = K*WEIGHT_STAGGER,
    parameter int unsigned TOTNUMTRITS = IMAGEWIDTH*IMAGEHEIGHT*N_I,
    parameter int unsigned TRITSPERBANK = (TOTNUMTRITS+NUMBANKS-1)/NUMBANKS,
    parameter int unsigned EFFECTIVETRITSPERWORD = N_I/WEIGHT_STAGGER,
    parameter int unsigned PHYSICALTRITSPERWORD = ((EFFECTIVETRITSPERWORD + 4) / 5) * 5,
    parameter int unsigned PHYSICALBITSPERWORD = PHYSICALTRITSPERWORD / 5 * 8,
    parameter int unsigned ACTMEMBANKDEPTH = (TRITSPERBANK+EFFECTIVETRITSPERWORD-1)/EFFECTIVETRITSPERWORD,
    parameter int unsigned ACTMEMFULLADDRESSBITWIDTH = $clog2(NUMBANKS*ACTMEMBANKDEPTH),
    parameter int unsigned WEIGHTMEMFULLADDRESSBITWIDTH = $clog2(WEIGHTBANKDEPTH),
    parameter int unsigned NUMENCODERS = (N_I/WEIGHT_STAGGER+4)/5,
    parameter int unsigned NUMDECODERS = PHYSICALBITSPERWORD/8,
    parameter int unsigned BUFFER_WIDTH = (2*(N_I/WEIGHT_STAGGER))/`TCDM_BUS_WIDTH,
    localparam int unsigned ACTMEM_ADDR_WIDTH = $clog2(IMAGEWIDTH*IMAGEHEIGHT*WEIGHT_STAGGER),
    localparam int unsigned ACTMEM_SET_WIDTH = $clog2(NUMACTMEMBANKSETS),
    localparam int unsigned WEIGHTMEM_ADDR_WIDTH = $clog2(K*K*WEIGHT_STAGGER*NUM_LAYERS),
    localparam int unsigned WEIGHTMEM_BANK_WIDTH = $clog2(N_I)
    )(
      input                                           soc_clk_i,
      input                                           soc_rst_ni,
      input                                           cutie_clk_i,
      input                                           cutie_rst_ni,

      XBAR_TCDM_BUS.Slave                             tcdm_slave,

      input logic [PHYSICALBITSPERWORD-1:0]           actmem_rdata_i,
      input logic                                     actmem_valid_i,

      input logic [PHYSICALBITSPERWORD-1:0]           weightmem_rdata_i,
      input logic                                     weightmem_valid_i,

      output logic [BANKSETSBITWIDTH-1:0]             actmem_bank_set_o,
      output logic                                    actmem_we_o,
      output logic                                    actmem_req_o,
      output logic [ACTMEMFULLADDRESSBITWIDTH-1:0]    actmem_addr_o,
      output logic [PHYSICALBITSPERWORD-1:0]          actmem_wdata_o,

      output logic [$clog2(N_O)-1:0]                  weightmem_bank_o,
      output logic                                    weightmem_we_o,
      output logic                                    weightmem_req_o,
      output logic [WEIGHTMEMFULLADDRESSBITWIDTH-1:0] weightmem_addr_o,
      output logic [PHYSICALBITSPERWORD-1:0]          weightmem_wdata_o
    );

   typedef enum                                       logic                                      {ACTMEM, WEIGHTMEM} memory_t;

   typedef struct packed {
      logic [PHYSICALBITSPERWORD-1:0] data;
      logic [`TCDM_ADDR_WIDTH-1:0] addr;
      logic                        we;
      memory_t                     memory;
   } cdc_fifo_t;

  // Request/write FSM
  typedef enum                     {ACTIVE, WAIT_WR_FIFO} request_state_t;
  request_state_t req_st_d, req_st_q;
  logic [BUFFER_WIDTH-1:0][`TCDM_BUS_WIDTH-1:0] wr_buffer_data_q, wr_buffer_data_d;
  logic [$clog2(BUFFER_WIDTH)-1:0]              wr_buffer_cnt_q, wr_buffer_cnt_d;
  logic [$clog2(BUFFER_WIDTH):0]                req_rd_buffer_cnt_q, req_rd_buffer_cnt_d;
  logic [`TCDM_ADDR_WIDTH-1:0]                  wr_addr_q, wr_addr_d;
  logic [`TCDM_ADDR_WIDTH-1:0]                  req_rd_addr_q, req_rd_addr_d;
  logic                                         we_s, we_q, we_d;
  logic                                         req_gnt;

  // Response/read FSM
  typedef enum                                  {IDLE, WAIT_RD_FIFO, OUT_ACTIVE} resp_state_t;
  resp_state_t resp_st_d, resp_st_q;
  logic [BUFFER_WIDTH-1:0][`TCDM_BUS_WIDTH-1:0] rd_buffer_data_q, rd_buffer_data_d;
  logic [`TCDM_ADDR_WIDTH-1:0]                  resp_rd_addr_q, resp_rd_addr_d;
  logic [$clog2(BUFFER_WIDTH)-1:0]              resp_rd_buffer_cnt_q, resp_rd_buffer_cnt_d;;
  logic                                         resp_gnt;



  cdc_fifo_t write_cdc_fifo_input, write_cdc_fifo_output;
  //logic [$bits(cdc_fifo_t)-1:0]                 write_cdc_fifo_data_i, write_cdc_fifo_data_o;
  logic                                         write_cdc_fifo_valid_i, write_cdc_fifo_valid_o;
  logic                                         write_cdc_fifo_ready_i, write_cdc_fifo_ready_o;

  cdc_fifo_t read_cdc_fifo_input, read_cdc_fifo_output;
  //logic [$bits(cdc_fifo_t)-1:0]                 read_cdc_fifo_data_i, read_cdc_fifo_data_o;
  logic                                         read_cdc_fifo_valid_i, read_cdc_fifo_valid_o;
  logic                                         read_cdc_fifo_ready_i, read_cdc_fifo_ready_o;

  logic [NUMENCODERS-1:0][9:0]                  encoder_inputs;
  logic [NUMENCODERS-1:0][7:0]                  encoder_outputs;
  logic [NUMDECODERS-1:0][7:0]                  decoder_inputs;
  logic [NUMDECODERS-1:0][9:0]                  decoder_outputs;
  logic [PHYSICALTRITSPERWORD-1:0][1:0]         decoder_outputs_flat;

  // this can stay static

  // spare MSB of encoder inputs are padded with zeros
  assign encoder_inputs = {'0, {>>{wr_buffer_data_q}}};
  assign write_cdc_fifo_input.data = encoder_outputs;
  assign write_cdc_fifo_input.we = we_s;
  assign write_cdc_fifo_input.memory = (we_s) ? ((wr_addr_q[`MEM_REGION_DIFF_BIT]) ? WEIGHTMEM : ACTMEM) :
						((req_rd_addr_q[`MEM_REGION_DIFF_BIT]) ? WEIGHTMEM : ACTMEM);
  assign write_cdc_fifo_input.addr = (we_s) ? wr_addr_q : req_rd_addr_q;

  //assign decoder_inputs[2*PHYSICALTRITSPERWORD-1:2*EFFECTIVETRITSPERWORD] = '0;
  assign decoder_inputs = read_cdc_fifo_output.data;

  assign decoder_outputs_flat = decoder_outputs;

  assign tcdm_slave.r_rdata = rd_buffer_data_q[resp_rd_buffer_cnt_q];
  assign tcdm_slave.gnt = (tcdm_slave.wen) ? resp_gnt : req_gnt;
  assign tcdm_slave.r_opc = 1'b0;


  always_comb begin : req_fsm
    req_st_d               = req_st_q;
    wr_buffer_data_d       = wr_buffer_data_q;
    wr_buffer_cnt_d        = wr_buffer_cnt_q;
    wr_addr_d              = wr_addr_q;
    req_rd_buffer_cnt_d    = req_rd_buffer_cnt_q;
    req_rd_addr_d          = req_rd_addr_q;
    req_gnt                = 1'b0;
    we_d                   = we_q;
    we_s                   = we_q;
    write_cdc_fifo_valid_i = 1'b0;

    unique case (req_st_q)
      ACTIVE : begin
	if (tcdm_slave.req) begin
	  if (tcdm_slave.wen) begin
	    // read - grant is handled by response FSM;
	    if (tcdm_slave.add == req_rd_addr_q) begin
	      // if we get the same address as
	      // before, only place a read request on the FIFO
	      // if we have filled BUFFER_WIDTH read requests
	      // already from this
	      // address
	      if (req_rd_buffer_cnt_q == 'd0) begin
		write_cdc_fifo_valid_i = 1'b1;
		we_s                  = 1'b0;
		if (write_cdc_fifo_ready_o) begin
		  req_rd_buffer_cnt_d = BUFFER_WIDTH;
		end else begin
		  we_d     = 1'b0;
		  req_st_d = WAIT_WR_FIFO;
		end
	      end
	    end else begin // if (tcdm_slave.add == req_rd_addr_q)
	      req_rd_addr_d = tcdm_slave.add;
	      we_d = 1'b0;
	      req_st_d      = WAIT_WR_FIFO;
	      // if the response FSM grants the  read request, decrement the
	      // counter
	    end // else: !if(tcdm_slave.add == req_rd_addr_q)
	    if (tcdm_slave.gnt) begin
	      req_rd_buffer_cnt_d = req_rd_buffer_cnt_q-1;
	    end
	  end else begin // if (tcdm_slave.wen)
	    // write - grant is handled by this fsm
	    we_d = 1'b1;
	    if (wr_buffer_cnt_q == 'd0) begin
	      // "new" write -> we can always grant this
	      req_gnt             = 1'b1;
	      wr_buffer_data_d[0] = tcdm_slave.wdata;
	      for (int i=1; i<BUFFER_WIDTH; i++)
		wr_buffer_data_d[i] = 'h0;
	      wr_addr_d           = tcdm_slave.add;
	      if (BUFFER_WIDTH != 1)
		wr_buffer_cnt_d                   = wr_buffer_cnt_q + 1;
	      else
		req_st_d = WAIT_WR_FIFO;
	    end else if (tcdm_slave.add == wr_addr_q) begin
	      req_gnt                           = 1'b1;
	      wr_buffer_data_d[wr_buffer_cnt_q] = tcdm_slave.wdata;
	      if (wr_buffer_cnt_q != BUFFER_WIDTH-1)
		wr_buffer_cnt_d = wr_buffer_cnt_q + 1;
	      else begin
		wr_buffer_cnt_d = 'd0;
		// before writing into the fifo, we need to wait at least one cycle for
		// wr_buffer_data_q to get updated -> go into wait state
		req_st_d        = WAIT_WR_FIFO;
	      end
	    end else begin // if (tcdm_slave.add == wr_addr_q)
	      // the address we got is different from the last one -> we need to
	      // flush the write buffer
	      write_cdc_fifo_valid_i = 1'b1;
	      we_s = 1'b1;
	      // TODO fill fifo input
	      if (write_cdc_fifo_ready_o) begin
		wr_addr_d           = tcdm_slave.add;
		req_gnt             = 1'b1;
		wr_buffer_data_d[0] = tcdm_slave.wdata;
		for (int i=1; i<BUFFER_WIDTH; i++)
		  wr_buffer_data_d[i] = 'h0;
		wr_buffer_cnt_d = 'd1;
	      end else begin
		req_st_d = WAIT_WR_FIFO;
	      end
	    end // else: !if(tcdm_slave.add == wr_addr_q)
	  end // else: !if(tcdm_slave.wen)
	end // if (tcdm_slave.req)
      end // case: ACTIVE
      WAIT_WR_FIFO : begin
	write_cdc_fifo_valid_i = 1'b1;
	if (write_cdc_fifo_ready_o) begin
	  //req_gnt = 1'b1; // if write request, this is propagated - if read
			  // request, no effect
			  //no! we already grant all requests in the ACTIVE state
	  if (tcdm_slave.wen) begin
	    req_rd_buffer_cnt_d = BUFFER_WIDTH; // "refill" the read counter
	  end else begin
	    wr_buffer_cnt_d = 'd0;
	  end
	  req_st_d = ACTIVE;
	end
      end // case: WAIT_WR_FIFO
      default : begin
	req_st_d = ACTIVE;
      end
    endcase // block: req_fsm
  end // block: req_fsm

  always_comb begin : resp_fsm
    resp_st_d             = resp_st_q;
    resp_rd_addr_d        = resp_rd_addr_q;
    resp_rd_buffer_cnt_d  = resp_rd_buffer_cnt_q;
    resp_gnt              = 1'b0;
    rd_buffer_data_d      = rd_buffer_data_q;
    read_cdc_fifo_ready_i = 1'b0;

    unique case (resp_st_q)
      IDLE : begin
	resp_rd_buffer_cnt_d = 'd0;
	if (tcdm_slave.req && tcdm_slave.wen) begin
	  resp_rd_addr_d = tcdm_slave.add;
	  resp_st_d      = WAIT_RD_FIFO;
	end
      end
      WAIT_RD_FIFO : begin
	read_cdc_fifo_ready_i = 1'b1;
	if (read_cdc_fifo_valid_o) begin
	  resp_rd_buffer_cnt_d = 'd0;
	  rd_buffer_data_d     = decoder_outputs_flat[EFFECTIVETRITSPERWORD-1:0];
	  resp_st_d            = OUT_ACTIVE;
	  resp_gnt = 1'b1;
	end
      end
      OUT_ACTIVE : begin
	if (tcdm_slave.req && tcdm_slave.wen) begin
	  if (tcdm_slave.add == resp_rd_addr_q) begin
	    if (resp_rd_buffer_cnt_q == BUFFER_WIDTH-1)
	      resp_st_d = IDLE;
	    else begin
	      resp_gnt             = 1'b1;
	      resp_rd_buffer_cnt_d = resp_rd_buffer_cnt_q + 1;
	    end
	  end else begin
	    resp_rd_buffer_cnt_d = 'd0;
	    resp_st_d            = WAIT_RD_FIFO;
	    resp_rd_addr_d       = tcdm_slave.add;
	  end
	end
      end
    endcase // case (resp_st_q)
  end
   always_comb begin : cutie_clk_domain

      read_cdc_fifo_input = '0;

      write_cdc_fifo_ready_i = 1'b1; // memories are always ready for external input
      //write_cdc_fifo_output = cdc_fifo_t'(write_cdc_fifo_data_o);

      actmem_bank_set_o = 0;
      actmem_we_o = 1'b0;
      actmem_req_o = 1'b0;
      actmem_addr_o = '0;
      actmem_wdata_o = '0;

      weightmem_bank_o = '0;
      weightmem_we_o = 1'b0;
      weightmem_req_o = 1'b0;
      weightmem_addr_o = '0;
      weightmem_wdata_o = '0;


      // Process output of write fifo
      if (write_cdc_fifo_valid_o) begin

	 unique case (write_cdc_fifo_output.memory)

	   ACTMEM: begin
	      actmem_bank_set_o = write_cdc_fifo_output.addr[ACTMEM_ADDR_WIDTH+ACTMEM_SET_WIDTH-1+2:ACTMEM_ADDR_WIDTH+2];
	      actmem_we_o = write_cdc_fifo_output.we;
	      actmem_req_o = 1'b1;
	      actmem_addr_o = write_cdc_fifo_output.addr[ACTMEM_ADDR_WIDTH-1+2:2];
	      actmem_wdata_o = write_cdc_fifo_output.data;
	   end

	   WEIGHTMEM: begin
	      // WRITE operation to WEIGHTMEM
	      weightmem_bank_o = write_cdc_fifo_output.addr[WEIGHTMEM_ADDR_WIDTH+WEIGHTMEM_BANK_WIDTH-1+2:WEIGHTMEM_ADDR_WIDTH+2];
	      weightmem_we_o = write_cdc_fifo_output.we;
	      weightmem_req_o =1'b1;
	      weightmem_addr_o = write_cdc_fifo_output.addr[WEIGHTMEM_ADDR_WIDTH+2-1:2];
	      weightmem_wdata_o = write_cdc_fifo_output.data;
	   end
	   default: ;
	 endcase // unique case (write_cdc_fifo_output.memory)
      end

      if (actmem_valid_i) begin
	 read_cdc_fifo_input.data = actmem_rdata_i;
	 read_cdc_fifo_input.addr = '0; // Currently addr information is lost
	 read_cdc_fifo_input.memory = ACTMEM;
	 read_cdc_fifo_input.we = 1'b0;
      end else if (weightmem_valid_i) begin
	 read_cdc_fifo_input.data = weightmem_rdata_i;
	 read_cdc_fifo_input.addr = '0; // Currently addr information is lost
	 read_cdc_fifo_input.memory = WEIGHTMEM;
	 read_cdc_fifo_input.we = 1'b0;
      end

      read_cdc_fifo_valid_i = actmem_valid_i || weightmem_valid_i;
      //read_cdc_fifo_data_i = {>>{read_cdc_fifo_input}};
   end // block: signals2mem


   always_ff @(posedge soc_clk_i, negedge soc_rst_ni) begin
      if (~soc_rst_ni) begin
	resp_st_q            <= IDLE;
	resp_rd_addr_q       <= 'h0;
	resp_rd_buffer_cnt_q <= 'h0;
	rd_buffer_data_q     <= '0;
	req_st_q             <= ACTIVE;
	wr_buffer_data_q     <= '0;
	wr_addr_q            <= 'h0;
	wr_buffer_cnt_q      <= 'h0;
	req_rd_buffer_cnt_q  <= 'h0;
	req_rd_addr_q        <= 'h0;
	we_q                 <= 1'b0;
	tcdm_slave.r_valid   <= 1'b0;
      end else begin
	resp_st_q            <= resp_st_d;
	resp_rd_addr_q       <= resp_rd_addr_d;
	resp_rd_buffer_cnt_q <= resp_rd_buffer_cnt_d;
	rd_buffer_data_q     <= rd_buffer_data_d;
	req_st_q             <= req_st_d;
	wr_buffer_data_q     <= wr_buffer_data_d;
	wr_addr_q            <= wr_addr_d;
	wr_buffer_cnt_q      <= wr_buffer_cnt_d;
	req_rd_buffer_cnt_q  <= req_rd_buffer_cnt_d;
	req_rd_addr_q        <= req_rd_addr_d;
	we_q                 <= we_d;
	tcdm_slave.r_valid   <= tcdm_slave.gnt & tcdm_slave.req;
      end
   end // always_ff @ (posedge clk_i, negedge rst_ni)

   for (genvar ii = 0; ii < NUMENCODERS; ii++) begin : encoders
      encoder i_encoder (.encoder_i(encoder_inputs[ii]), .encoder_o(encoder_outputs[ii]));
   end

   for (genvar ii = 0; ii < NUMDECODERS; ii++) begin : decoders
      decoder i_decoder (.decoder_i(decoder_inputs[ii]), .decoder_o(decoder_outputs[ii]));
   end




   cdc_fifo_gray
     #(
       // Somehow, parametrizing width does not work. Use constant instead
       //.WIDTH       ( 114                  ),
       .T(cdc_fifo_t),
       .LOG_DEPTH   ( 3                    )
       ) i_write_cdc_fifo_gray
       (
	.src_rst_ni  ( soc_rst_ni                ),
	.src_clk_i   ( soc_clk_i                 ),
	.src_data_i  ( write_cdc_fifo_input     ),
	.src_valid_i ( write_cdc_fifo_valid_i    ),
	.src_ready_o ( write_cdc_fifo_ready_o    ),
	.dst_rst_ni  ( cutie_rst_ni              ),
	.dst_clk_i   ( cutie_clk_i               ),
	.dst_data_o  ( write_cdc_fifo_output     ),
	.dst_valid_o ( write_cdc_fifo_valid_o    ),
	.dst_ready_i ( write_cdc_fifo_ready_i    )
	);

   cdc_fifo_gray
     #(
       // Somehow, parametrizing width does not work. Use constant instead
       //.WIDTH       ( 114                  ),
       .T(cdc_fifo_t),
       .LOG_DEPTH   ( 3                    )
       ) i_read_cdc_fifo_gray
       (
	.src_rst_ni  ( cutie_rst_ni             ),
	.src_clk_i   ( cutie_clk_i              ),
	.src_data_i  ( read_cdc_fifo_input     ),
	.src_valid_i ( read_cdc_fifo_valid_i    ),
	.src_ready_o ( read_cdc_fifo_ready_o    ),
	.dst_rst_ni  ( soc_rst_ni               ),
	.dst_clk_i   ( soc_clk_i                ),
	.dst_data_o  ( read_cdc_fifo_output     ),
	.dst_valid_o ( read_cdc_fifo_valid_o    ),
	.dst_ready_i ( read_cdc_fifo_ready_i    )
	);

endmodule // tcdm_to_mem
