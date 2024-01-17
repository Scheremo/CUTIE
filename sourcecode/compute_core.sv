// ----------------------------------------------------------------------
//
// File: compute_core.sv
//
// Created: 26.07.2023
//
// Copyright (C) 2023, ETH Zurich and University of Bologna.
//
// Author: Moritz Scherer, ETH Zurich
//
// SPDX-License-Identifier: SHL-0.51
//
// Copyright and related rights are licensed under the Solderpad Hardware License,
// Version 0.51 (the "License"); you may not use this file except in compliance with
// the License. You may obtain a copy of the License at http://solderpad.org/licenses/SHL-0.51.
// Unless required by applicable law or agreed to in writing, software, hardware and materials
// distributed under this License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and limitations under the License.
//
// ----------------------------------------------------------------------

module compute_core #(
		       parameter K = 3,
		       parameter N_I = 96,
		       parameter N_O = 96,
		       parameter IMAGEWIDTH = 96,
		       parameter IMAGEHEIGHT = 96,
		       parameter ITERATIVE_DECOMP = 1,
		       parameter PIPELINEDEPTH = 4,
		       parameter PIPELINEWIDTH = 4,
		       parameter WEIGHT_STAGGER = 2,
		       parameter NUM_LAYERS = 1,
		       parameter WEIGHTBANKDEPTH = 64,
		       parameter PHYSICALBITSPERWORD = 16,
		       parameter EFFECTIVETRITSPERWORD = 10,
		       parameter POOLING_FIFODEPTH = 8,
		       parameter THRESHOLD_FIFODEPTH = 8,
		       parameter WEIGHTMEMFULLADDRESSBITWIDTH = 15,
		       parameter COMPUTEDELAY = 12

		      )(

			 input logic								    clk_i,
			 input logic								    rst_ni,

			 input logic [0:K-1][0:K-1][0:N_I-1][1:0]				    acts_i,
			 input logic								    acts_valid_i,
			 input logic signed [$clog2(K*K*N_I):0]					    ocu_thresh_pos_i, ocu_thresh_neg_i,
			 input logic [0:(N_O/ITERATIVE_DECOMP)-1]				    ocu_thresholds_save_enable_i,

			 input logic								    layer_compute_latch_new_layer_i,

			 input logic [$clog2(N_O):0]						    layer_compute_no_i,
			 input logic unsigned [$clog2(IMAGEWIDTH):0]				    layer_imagewidth_i,
			 input logic unsigned [$clog2(IMAGEHEIGHT):0]				    layer_imageheight_i,
			 input logic								    layer_pooling_enable_i,
			 input logic unsigned [$clog2(K)-1:0]					    layer_pooling_kernel_i,
			 input logic								    layer_pooling_pooling_type_i,
			 input logic								    layer_pooling_padding_type_i,
			 input logic								    layer_skip_in_i,
			 input logic								    layer_skip_out_i,

			 input logic [$clog2(N_O):0]						    layer_weights_no_i,
			 input logic [$clog2(N_I):0]						    layer_weights_ni_i,
			 input logic [$clog2(K):0]						    layer_weights_k_i,
			 input logic [0:PIPELINEDEPTH-1]					    layer_weights_latch_new_layer_i,
			 input logic [0:PIPELINEDEPTH-1]					    layer_weights_soft_reset_i,
			 input logic [0:PIPELINEDEPTH-1]					    layer_weights_toggle_banks_i,


			 input logic [0:(N_O/ITERATIVE_DECOMP)-1]				    weightmemory_external_we_vector_i,
			 input logic [0:(N_O/ITERATIVE_DECOMP)-1]				    weightmemory_external_req_vector_i,
			 input logic [0:(N_O/ITERATIVE_DECOMP)-1][WEIGHTMEMFULLADDRESSBITWIDTH-1:0] weightmemory_external_addr_vector_i,
			 input logic [0:(N_O/ITERATIVE_DECOMP)-1][PHYSICALBITSPERWORD-1:0]	    weightmemory_external_wdata_vector_i,


			 output logic [0:(N_O/ITERATIVE_DECOMP)-1][PHYSICALBITSPERWORD-1:0]	    weightmemory_external_weights_vector_o,
			 output logic [0:(N_O/ITERATIVE_DECOMP)-1]				    weightmemory_external_valid_vector_o,

			 output logic [0:PIPELINEDEPTH-1][0:(PIPELINEWIDTH)-1]			    weightmemory_controller_ready_o, // OCU is ready for inputs
			 output logic [0:PIPELINEDEPTH-1][0:(PIPELINEWIDTH)-1]			    weightmemory_controller_valid_o, // Activation memory outputs are valid

			 output logic [0:PIPELINEDEPTH-1][0:(PIPELINEWIDTH)-1][1:0]		    compute_output_o,
			 output logic								    compute_output_valid_o,
			 output logic [0:PIPELINEDEPTH-1][0:(PIPELINEWIDTH)-1][$clog2(K*K*N_I):0]   fp_output_o

			);

   ///////////////////////////////// WEIGHTMEMORY CONTROLLER HELPER SIGNALS /////////////////////////////////
   logic [0:(N_O/ITERATIVE_DECOMP)-1]					   weightmemory_valid_vector_o;

   logic [0:PIPELINEDEPTH-1][0:(PIPELINEWIDTH)-1]	   weightmemory_controller_rw_collision_in;
   logic [0:PIPELINEDEPTH-1][0:(PIPELINEWIDTH)-1]	   weightmemory_controller_ready_in; // OCU is ready for inputs
   logic [0:PIPELINEDEPTH-1][0:(PIPELINEWIDTH)-1]	   weightmemory_controller_valid_in; // Activation memory outputs are valid

   logic [0:PIPELINEDEPTH-1][0:(PIPELINEWIDTH)-1]	   weightmemory_controller_ready_out; // OCU is ready for inputs
   logic [0:PIPELINEDEPTH-1][0:(PIPELINEWIDTH)-1]	   weightmemory_controller_valid_out; // Activation memory outputs are valid

   assign weightmemory_controller_ready_o = weightmemory_controller_ready_out;
   assign weightmemory_controller_valid_o = weightmemory_controller_valid_out;

   ///////////////////////////////// END WEIGHTMEMORY CONTROLLER HELPER SIGNALS /////////////////////////////////


   logic [0:PIPELINEDEPTH-1][0:(PIPELINEWIDTH)-1][1:0]	   immediate_compute_output;
   logic [0:PIPELINEDEPTH-1][0:(PIPELINEWIDTH)-1][1:0]	   pipelined_compute_output;

   ///////////////////////////////// COMPUTE CORE /////////////////////////////////

   ///////////////////////////////// OCU CONTROLLER SIGNALS /////////////////////////////////

   logic [0:PIPELINEDEPTH-1]				   ocu_controller_compute_enable;

   logic						   ocu_controller_pooling_fifo_flush;

   logic						   ocu_controller_pooling_fifo_testmode;

   logic						   ocu_controller_pooling_store_to_fifo;
   // 1: Store, 0: Don't store
   logic						   ocu_controller_threshold_fifo_flush;

   logic						   ocu_controller_threshold_fifo_testmode;

   logic [0:(N_O/ITERATIVE_DECOMP)-1]			   ocu_controller_threshold_pop;
   // 1: Store, 0: Don't store
   enums_ocu_pool::alu_operand_sel ocu_controller_alu_operand_sel;
   // 01: FIFO, 10: previous result, 00: zero ATTENTION: Choosing 01 also pops from FIFO!
   enums_ocu_pool::multiplexer ocu_controller_multiplexer;
   // 1: Use ALU result, 0: Use current convolution result
   enums_ocu_pool::alu_op ocu_controller_alu_op;
   // 1: Sum, 0: Max
   logic                                                                         ocu_controller_ready;

   logic									 ocu_controller_valid;


   logic [0:COMPUTEDELAY-1]							 pipelined_ocu_controller_valid_q, pipelined_ocu_controller_valid_d;
   logic [0:PIPELINEDEPTH-1]							 pipelined_ocu_controller_pooling_store_to_fifo_q, pipelined_ocu_controller_pooling_store_to_fifo_d;
   logic [0:PIPELINEDEPTH-1][1:0]						 pipelined_ocu_controller_alu_operand_sel_q, pipelined_ocu_controller_alu_operand_sel_d;
   logic [0:PIPELINEDEPTH-1]							 pipelined_ocu_controller_alu_op_q, pipelined_ocu_controller_alu_op_d;

   ///////////////////////////////// END OCU CONTROLLER SIGNALS /////////////////////////////////


   logic [0:PIPELINEDEPTH-1][0:K-1][0:K-1][0:N_I-1][1:0]			 acts_pipeline;

   assign acts_pipeline[0] = acts_i;
   assign compute_output_o = pipelined_compute_output;
   assign compute_output_valid_o = pipelined_ocu_controller_valid_q[PIPELINEDEPTH-1];

   ///////////////////////////////// OCU GLUE LOGIC /////////////////////////////////

   always_comb begin
      pipelined_ocu_controller_valid_d[0] = ocu_controller_valid;
      if (PIPELINEDEPTH > 1) begin
	 pipelined_ocu_controller_valid_d[1:COMPUTEDELAY-1] = pipelined_ocu_controller_valid_q[0:COMPUTEDELAY-2];
      end
   end

   always_ff @(posedge clk_i, negedge rst_ni) begin
      if(~rst_ni) begin
	 pipelined_ocu_controller_valid_q <= '0;
      end else begin
	 pipelined_ocu_controller_valid_q <= pipelined_ocu_controller_valid_d;
      end
   end

   always_comb begin
      pipelined_ocu_controller_pooling_store_to_fifo_d[0] = ocu_controller_pooling_store_to_fifo;
      pipelined_ocu_controller_pooling_store_to_fifo_d[1:PIPELINEDEPTH-1] = pipelined_ocu_controller_pooling_store_to_fifo_q[0:PIPELINEDEPTH-2];
   end

   always_ff @(posedge clk_i, negedge rst_ni) begin
      if(~rst_ni) begin
	 pipelined_ocu_controller_pooling_store_to_fifo_q <= '0;
      end else begin
	 pipelined_ocu_controller_pooling_store_to_fifo_q <= pipelined_ocu_controller_pooling_store_to_fifo_d;
      end
   end

   always_comb begin
      pipelined_ocu_controller_alu_operand_sel_d[0] = ocu_controller_alu_operand_sel;
      pipelined_ocu_controller_alu_operand_sel_d[1:PIPELINEDEPTH-1] = pipelined_ocu_controller_alu_operand_sel_q[0:PIPELINEDEPTH-2];
   end

   always_ff @(posedge clk_i, negedge rst_ni) begin
      if(~rst_ni) begin
	 pipelined_ocu_controller_alu_operand_sel_q <= '0;
      end else begin
	 pipelined_ocu_controller_alu_operand_sel_q <= pipelined_ocu_controller_alu_operand_sel_d;
      end
   end


   ///////////////////////////////// END OCU GLUE LOGIC /////////////////////////////////



   ocu_controller
     #(
       .PIPELINEDEPTH(PIPELINEDEPTH),
       .N_O(N_O),
       .K(K),
       .IMAGEWIDTH(IMAGEWIDTH),
       .IMAGEHEIGHT(IMAGEHEIGHT),
       .ITERATIVE_DECOMP(ITERATIVE_DECOMP)
       )
   ocu_controller (
		   .clk_i(clk_i),
		   .rst_ni(rst_ni),

		   .latch_new_layer_i(layer_compute_latch_new_layer_i),
		   .layer_no_i(layer_compute_no_i),
		   .layer_imagewidth_i(layer_imagewidth_i),
		   .layer_imageheight_i(layer_imageheight_i),
		   .layer_pooling_enable_i(layer_pooling_enable_i),
		   .layer_pooling_kernel_i(layer_pooling_kernel_i),
		   .layer_pooling_pooling_type_i(layer_pooling_pooling_type_i),
		   .layer_pooling_padding_type_i(layer_pooling_padding_type_i),
		   .layer_skip_in_i(layer_skip_in_i),
		   .layer_skip_out_i(layer_skip_out_i),

		   .tilebuffer_valid_i(acts_valid_i),
		   .weightmemory_valid_i(weightmemory_valid_vector_o),

		   .compute_enable_o(ocu_controller_compute_enable),
		   .pooling_fifo_flush_o(ocu_controller_pooling_fifo_flush),
		   .pooling_fifo_testmode_o(ocu_controller_pooling_fifo_testmode),
		   .pooling_store_to_fifo_o(ocu_controller_pooling_store_to_fifo),
		   .threshold_fifo_flush_o(ocu_controller_threshold_fifo_flush),
		   .threshold_fifo_testmode_o(ocu_controller_threshold_fifo_testmode),
		   .threshold_pop_o(ocu_controller_threshold_pop),
		   .alu_operand_sel_o(ocu_controller_alu_operand_sel),
		   .multiplexer_o(ocu_controller_multiplexer),
		   .alu_op_o(ocu_controller_alu_op),
		   .ready_o(ocu_controller_ready),
		   .valid_o(ocu_controller_valid)
		   /*AUTOINST*/);


   ///////////////////////////////// PIPELINE STAGES /////////////////////////////////

   genvar                                                    pipelinestage;
   genvar						     localmodulenum;
   generate
      for(pipelinestage=0;pipelinestage<PIPELINEDEPTH;pipelinestage++) begin: pipeline

	 logic [0:(PIPELINEWIDTH)-1][1:0]                  pipeline_output;

	 logic [0:(PIPELINEDEPTH-2)-pipelinestage][0:(PIPELINEWIDTH)-1][1:0] output_pipeline_d, output_pipeline;
	 ///////////////////////////////// PIPELINE STAGE SIGNALS /////////////////////////////////

	 if(pipelinestage < PIPELINEDEPTH-1) begin : compute_output_pipeline

	    always_comb begin
	       output_pipeline_d[0] = immediate_compute_output[pipelinestage];
	       for (int i=1;i<PIPELINEDEPTH-pipelinestage-1;i++) begin
		  output_pipeline_d[i] = output_pipeline[i-1];
	       end
	    end

	    assign pipeline_output = output_pipeline[(PIPELINEDEPTH-2)-pipelinestage];

	    ///////////////////////////////// PIPELINE FOR OUTPUT /////////////////////////////////

	    always_ff @(posedge clk_i, negedge rst_ni) begin
	       if(~rst_ni) begin
		  for (int i=1;i<PIPELINEDEPTH-pipelinestage-1;i++) begin
		     output_pipeline[i] <= '0;
		  end
	       end else begin
		  output_pipeline <= output_pipeline_d;
	       end
	    end // always_ff @ (posedge clk_i, negedge rst_ni)

	 end else begin // if (pipelinestage < PIPELINEDEPTH-1)
	    assign pipeline_output = immediate_compute_output[pipelinestage];
	 end // else: !if(pipelinestage < PIPELINEDEPTH-2)

	 assign pipelined_compute_output[pipelinestage] = pipeline_output;

	 if(PIPELINEDEPTH-pipelinestage>0 && pipelinestage > 0) begin : activations_pipeline

	    ///////////////////////////////// PIPELINE FOR ACTIVATIONS /////////////////////////////////

	    always_ff @(posedge clk_i, negedge rst_ni) begin
	       if(~rst_ni) begin
		  acts_pipeline[pipelinestage] <= '0;
	       end else begin
		  if(pipelinestage < layer_compute_no_i/(PIPELINEWIDTH)) begin
		     acts_pipeline[pipelinestage] <= acts_pipeline[pipelinestage-1];
		  end
	       end
	    end
	 end

	 ///////////////////////////////// END PIPELINE STAGE SIGNALS /////////////////////////////////

	 ///////////////////////////////// WEIGHTMEMORY CONTROLLER SIGNALS /////////////////////////////////

	 logic weightmemory_controller_latch_new_layer_i;
	 logic unsigned [$clog2(K):0] weightmemory_controller_layer_k_i; // For MVP disregard kernel variance
	 logic unsigned [$clog2(N_I):0]	weightmemory_controller_layer_ni_i;
	 logic unsigned [$clog2(N_O):0]	weightmemory_controller_layer_no_i;
	 logic [0:(PIPELINEWIDTH)-1]	weightmemory_controller_rw_collision_i;
	 logic [0:(PIPELINEWIDTH)-1]	weightmemory_controller_ready_i; // OCU is ready for inputs
	 logic [0:(PIPELINEWIDTH)-1]	weightmemory_controller_valid_i; // Activation memory outputs are valid
	 logic				weightmemory_controller_soft_reset_i;
	 logic				weightmemory_controller_toggle_banks_i;

	 logic				weightmemory_controller_mem_read_enable_o;
	 logic [$clog2(WEIGHTBANKDEPTH)-1:0] weightmemory_controller_mem_read_addr_o;
	 logic				     weightmemory_controller_weights_read_bank_o;
	 logic				     weightmemory_controller_weights_save_bank_o;
	 logic [0:WEIGHT_STAGGER-1][0:K-1][0:K-1] weightmemory_controller_weights_save_enable_o;
	 logic [0:WEIGHT_STAGGER-1][0:K-1][0:K-1] weightmemory_controller_weights_test_enable_o;
	 logic [0:WEIGHT_STAGGER-1]		  weightmemory_controller_weights_flush_o;
	 logic					  weightmemory_controller_ready_o;
	 logic					  weightmemory_controller_valid_o;

	 ///////////////////////////////// END WEIGHTMEMORY CONTROLLER SIGNALS /////////////////////////////////

	 weightmemory_controller
	   #(
	     .K(K), .N_I(N_I), .WEIGHT_STAGGER(WEIGHT_STAGGER), .N_O(N_O),
	     .BANKDEPTH(WEIGHTBANKDEPTH), .PIPELINEDEPTH(PIPELINEDEPTH), .NUM_LAYERS(NUM_LAYERS),
	     .ITERATIVE_DECOMP(ITERATIVE_DECOMP))
	 weightmemory_controller
	   (
	    .clk_i(clk_i),
	    .rst_ni(rst_ni),
	    .rw_collision_i(weightmemory_controller_rw_collision_i),
	    .valid_i(weightmemory_controller_valid_i),
	    .ready_i(weightmemory_controller_ready_i),
	    .latch_new_layer_i(weightmemory_controller_latch_new_layer_i),
	    .layer_k_i(weightmemory_controller_layer_k_i),
	    .layer_ni_i(weightmemory_controller_layer_ni_i),
	    .layer_no_i(weightmemory_controller_layer_no_i),
	    .soft_reset_i(weightmemory_controller_soft_reset_i),
	    .toggle_banks_i(weightmemory_controller_toggle_banks_i),
	    .mem_read_enable_o(weightmemory_controller_mem_read_enable_o),
	    .mem_read_addr_o(weightmemory_controller_mem_read_addr_o),
	    .weights_read_bank_o(weightmemory_controller_weights_read_bank_o),
	    .weights_save_bank_o(weightmemory_controller_weights_save_bank_o),
	    .weights_save_enable_o(weightmemory_controller_weights_save_enable_o),
	    .weights_test_enable_o(weightmemory_controller_weights_test_enable_o),
	    .weights_flush_o(weightmemory_controller_weights_flush_o),
	    .ready_o(weightmemory_controller_ready_o),
	    .valid_o(weightmemory_controller_valid_o)
	    /*AUTOINST*/);

	 ///////////////////////////////// WEIGHTMEMORY CONTROLLER SIGNAL CONNECTIONS /////////////////////////////////

	 assign weightmemory_controller_latch_new_layer_i = layer_weights_latch_new_layer_i[pipelinestage];
	 assign weightmemory_controller_layer_k_i =  layer_weights_k_i; // For MVP disregard kernel variance
	 assign weightmemory_controller_layer_ni_i = layer_weights_ni_i;
	 assign weightmemory_controller_layer_no_i = layer_weights_no_i;
	 assign weightmemory_controller_soft_reset_i = layer_weights_soft_reset_i[pipelinestage];
	 assign weightmemory_controller_toggle_banks_i = layer_weights_toggle_banks_i[pipelinestage];

	 assign weightmemory_controller_rw_collision_i = weightmemory_controller_rw_collision_in[pipelinestage];
	 assign weightmemory_controller_ready_i =  weightmemory_controller_ready_in[pipelinestage]; // OCU is ready for inputs
	 assign weightmemory_controller_valid_i = weightmemory_controller_valid_in[pipelinestage]; // Activation memory outputs are valid

	 ///////////////////////////////// END WEIGHTMEMORY CONTROLLER SIGNAL CONNECTIONS /////////////////////////////////


	 for(localmodulenum=0;localmodulenum<(PIPELINEWIDTH);localmodulenum++) begin: compute_block

	    ///////////////////////////////// OCU SIGNALS /////////////////////////////////

	    logic [0:(N_I/WEIGHT_STAGGER)-1][1:0] ocu_weights_i;
	    logic [0:K-1][0:K-1][0:N_I-1][1:0]    ocu_acts_i;
	    enums_ocu_pool::alu_operand_sel ocu_alu_operand_sel_i;
	    enums_ocu_pool::multiplexer ocu_multiplexer_i;
	    enums_ocu_pool::alu_op ocu_alu_op_i;
	    logic                                 ocu_pooling_fifo_flush_i;
	    logic				  ocu_pooling_fifo_testmode_i;
	    logic				  ocu_pooling_store_to_fifo_i;
	    logic				  ocu_threshold_fifo_flush_i;
	    logic				  ocu_threshold_fifo_testmode_i;
	    logic				  ocu_threshold_store_to_fifo_i;
	    logic				  ocu_threshold_pop_i;
	    logic				  ocu_weights_read_bank_i;
	    logic				  ocu_weights_save_bank_i;
	    logic				  ocu_compute_enable_i;
	    logic [0:WEIGHT_STAGGER-1][0:K-1][0:K-1] ocu_weights_save_enable_i;
	    logic [0:WEIGHT_STAGGER-1][0:K-1][0:K-1] ocu_weights_test_enable_i;
	    logic [0:WEIGHT_STAGGER-1]		     ocu_weights_flush_i;
	    //logic                                    local_ocu_thresholds_save_enable_i;

	    logic [1:0]				     ocu_out_o;

	    ///////////////////////////////// END OCU SIGNALS /////////////////////////////////
	    ///////////////////////////////// WEIGHTMEMORY SIGNALS /////////////////////////////////

	    logic				     weightmemory_read_enable_i;
	    logic [PHYSICALBITSPERWORD-1:0]	     weightmemory_wdata_i; // Data for up to all OCUs at once
	    logic [$clog2(WEIGHTBANKDEPTH)-1:0]	     weightmemory_read_addr_i; // Addresses for all memories
	    logic [$clog2(WEIGHTBANKDEPTH)-1:0]	     weightmemory_write_addr_i; // Addresses for all memories
	    logic				     weightmemory_write_enable_i; // Write enable for all memories

	    logic				     weightmemory_ready_o;
	    logic				     weightmemory_rw_collision_o;
	    logic [0:EFFECTIVETRITSPERWORD-1][1:0]   weightmemory_weights_o;
	    ///////////////////////////////// END WEIGHTMEMORY SIGNALS /////////////////////////////////

	    ocu_pool_weights
	      #(.K(K),
		.N_I(N_I),
		.POOLING_FIFODEPTH(POOLING_FIFODEPTH),
		.WEIGHT_STAGGER(WEIGHT_STAGGER),
		.THRESHOLD_FIFODEPTH(THRESHOLD_FIFODEPTH)
		) OCU (
		       .clk_i(clk_i),
		       .rst_ni(rst_ni),
		       .weights_i(ocu_weights_i),
		       .acts_i(ocu_acts_i),
		       .thresh_pos_i(ocu_thresh_pos_i),
		       .thresh_neg_i(ocu_thresh_neg_i),
		       .pooling_fifo_flush_i(ocu_pooling_fifo_flush_i),
		       .pooling_fifo_testmode_i(ocu_pooling_fifo_testmode_i),
		       .pooling_store_to_fifo_i(ocu_pooling_store_to_fifo_i),
		       .threshold_fifo_flush_i(ocu_threshold_fifo_flush_i),
		       .threshold_fifo_testmode_i(ocu_threshold_fifo_testmode_i),
		       .threshold_store_to_fifo_i(ocu_threshold_store_to_fifo_i),
		       .threshold_pop_i(ocu_threshold_pop_i),
		       .alu_operand_sel_i(ocu_alu_operand_sel_i),
		       .multiplexer_i(ocu_multiplexer_i),
		       .alu_op_i(ocu_alu_op_i),
		       .compute_enable_i(ocu_compute_enable_i),
		       .weights_read_bank_i(ocu_weights_read_bank_i),
		       .weights_save_bank_i(ocu_weights_save_bank_i),
		       .weights_save_enable_i(ocu_weights_save_enable_i),
		       .weights_test_enable_i(ocu_weights_test_enable_i),
		       .weights_flush_i(ocu_weights_flush_i),
		       .out_o(ocu_out_o),
		       .fp_out_o(fp_output_o[pipelinestage][localmodulenum])
		       /*AUTOINST*/);

	    assign immediate_compute_output[pipelinestage][localmodulenum] = ocu_out_o;

	    weightmemory_external_wrapper
	      #(
		.N_I(N_I),
		.K(K),
		.WEIGHT_STAGGER(WEIGHT_STAGGER),
		.BANKDEPTH(WEIGHTBANKDEPTH)
		) weightmemory_internal_wrapper (
						 .clk_i(clk_i),
						 .rst_ni(rst_ni),
						 .external_we_i(weightmemory_external_we_vector_i[localmodulenum + pipelinestage*(PIPELINEWIDTH)]),
						 .external_req_i(weightmemory_external_req_vector_i[localmodulenum + pipelinestage*(PIPELINEWIDTH)]),
						 .external_addr_i(weightmemory_external_addr_vector_i[localmodulenum + pipelinestage*(PIPELINEWIDTH)]),
						 .external_wdata_i(weightmemory_external_wdata_vector_i[localmodulenum + pipelinestage*(PIPELINEWIDTH)]),
						 .read_enable_i(weightmemory_read_enable_i),
						 .wdata_i(weightmemory_wdata_i),
						 .read_addr_i(weightmemory_read_addr_i),
						 .write_addr_i(weightmemory_write_addr_i),
						 .write_enable_i(weightmemory_write_enable_i),
						 .valid_o(weightmemory_ready_o),
						 .rw_collision_o(weightmemory_rw_collision_o),
						 .weights_o(weightmemory_weights_o),
						 .external_weights_o(weightmemory_external_weights_vector_o[localmodulenum + pipelinestage*(PIPELINEWIDTH)]),
						 .external_valid_o(weightmemory_external_valid_vector_o[localmodulenum + pipelinestage*(PIPELINEWIDTH)])
						 /*AUTOINST*/);



	    ///////////////////////////////// WEIGHTMEMORY SIGNALS CONNECTIONS /////////////////////////////////

	    assign weightmemory_valid_vector_o[localmodulenum + (PIPELINEWIDTH)*pipelinestage] = weightmemory_ready_o & weightmemory_rw_collision_o;

	    assign weightmemory_controller_ready_in[pipelinestage][localmodulenum] = ocu_controller_ready;
	    assign weightmemory_controller_rw_collision_in[pipelinestage][localmodulenum] = weightmemory_rw_collision_o;
	    assign weightmemory_controller_valid_in[pipelinestage][localmodulenum] = weightmemory_ready_o;

	    assign weightmemory_controller_ready_out[pipelinestage][localmodulenum] = weightmemory_controller_ready_o;
	    assign weightmemory_controller_valid_out[pipelinestage][localmodulenum] = weightmemory_controller_valid_o;

	    assign weightmemory_read_enable_i = weightmemory_controller_mem_read_enable_o;
	    assign weightmemory_read_addr_i = weightmemory_controller_mem_read_addr_o; // Addresses for all memories

	    // ATTENTION: Zero wired
	    assign weightmemory_write_addr_i = 'X; // Addresses for all memories
	    assign weightmemory_wdata_i = 'X; // Data for up to all OCUs at once
	    assign weightmemory_write_enable_i = '0; // __weightmemory_write_enable[pipelinestage][localmodulenum]; // Write enable for all memories

	    ///////////////////////////////// WEIGHTMEMORY SIGNALS CONNECTIONS /////////////////////////////////

	    ///////////////////////////////// OCU SIGNALS CONNECTIONS /////////////////////////////////

	    assign ocu_weights_i = weightmemory_weights_o;
	    assign ocu_alu_operand_sel_i = enums_ocu_pool::alu_operand_sel'(pipelined_ocu_controller_alu_operand_sel_d[pipelinestage]);
	    assign ocu_multiplexer_i = ocu_controller_multiplexer;
	    assign ocu_alu_op_i = ocu_controller_alu_op;
	    assign ocu_pooling_fifo_flush_i = ocu_controller_pooling_fifo_flush;
	    assign ocu_pooling_fifo_testmode_i = ocu_controller_pooling_fifo_testmode;
	    assign ocu_pooling_store_to_fifo_i = pipelined_ocu_controller_pooling_store_to_fifo_d[pipelinestage];
	    assign ocu_threshold_fifo_flush_i = ocu_controller_threshold_fifo_flush;
	    assign ocu_threshold_fifo_testmode_i =  ocu_controller_threshold_fifo_testmode;
	    assign ocu_threshold_store_to_fifo_i = ocu_thresholds_save_enable_i[localmodulenum + (PIPELINEWIDTH)*pipelinestage];
	    assign ocu_threshold_pop_i = ocu_controller_threshold_pop[localmodulenum + (PIPELINEWIDTH)*pipelinestage];
	    assign ocu_weights_read_bank_i = weightmemory_controller_weights_read_bank_o;
	    assign ocu_weights_save_bank_i = weightmemory_controller_weights_save_bank_o;
	    assign ocu_compute_enable_i = ocu_controller_compute_enable[pipelinestage];
	    assign ocu_weights_save_enable_i = weightmemory_controller_weights_save_enable_o;
	    assign ocu_weights_test_enable_i = weightmemory_controller_weights_test_enable_o;
	    assign ocu_weights_flush_i = weightmemory_controller_weights_flush_o;

	    //assign ocu_thresholds_save_enable_i = ocu_thresholds_save_enable[localmodulenum + pipelinestage*PIPELINEWIDTH];

	    assign ocu_acts_i = acts_pipeline[pipelinestage];
	    //assign ocu_acts_i = tilebuffer_acts_out_o;
	    ///////////////////////////////// END OCU SIGNALS CONNECTIONS /////////////////////////////////

	 end // for (pipelinestage=0;pipelinestage<PIPELINEDEPTH;pipelinestage++)
      end
   endgenerate
   ///////////////////////////////// END COMPUTE CORE /////////////////////////////////
endmodule
