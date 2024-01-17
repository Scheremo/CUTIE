// ----------------------------------------------------------------------
//
// File: cutie_top.sv
//
// Created: 06.05.2022
//
// Copyright (C) 2022, ETH Zurich and University of Bologna.
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
// This module is the system level interconnection of all submodules.

module cutie_top
  import cutie_params::*;
   #(
      parameter		     N_I = cutie_params::N_I,
      parameter		     N_O = cutie_params::N_O,
      parameter		     IMAGEWIDTH = cutie_params::IMAGEWIDTH,
      parameter		     IMAGEHEIGHT = cutie_params::IMAGEHEIGHT,
      parameter		     TCN_WIDTH = cutie_params::TCN_WIDTH,
      parameter		     K = cutie_params::K,
      parameter		     LAYER_FIFODEPTH = cutie_params::LAYER_FIFODEPTH,
      parameter		     POOLING_FIFODEPTH = cutie_params::POOLING_FIFODEPTH,
      parameter		     THRESHOLD_FIFODEPTH = cutie_params::THRESHOLD_FIFODEPTH,
      parameter		     WEIGHT_STAGGER = cutie_params::WEIGHT_STAGGER,
      parameter		     PIPELINEDEPTH = cutie_params::PIPELINEDEPTH,
      parameter		     WEIGHTBANKDEPTH = cutie_params::WEIGHTBANKDEPTH,
      parameter		     NUMACTMEMBANKSETS = cutie_params::NUMACTMEMBANKSETS,
      parameter		     NUM_LAYERS = cutie_params::NUM_LAYERS,
      parameter		     ITERATIVE_DECOMP = cutie_params::ITERATIVE_DECOMP,
      localparam	     PIPELINEWIDTH = (N_O/PIPELINEDEPTH)/ITERATIVE_DECOMP,

      parameter int unsigned OCUDELAY = 1,
      parameter int unsigned COMPUTEDELAY = PIPELINEDEPTH - 1 + OCUDELAY,

      parameter int unsigned POOLING_USAGEWIDTH = POOLING_FIFODEPTH > 1 ? $clog2(POOLING_FIFODEPTH) : 1,
      parameter int unsigned THRESHOLD_USAGEWIDTH = THRESHOLD_FIFODEPTH > 1 ? $clog2(THRESHOLD_FIFODEPTH) : 1,

      parameter int unsigned EFFECTIVETRITSPERWORD = N_I/WEIGHT_STAGGER,
      parameter int unsigned PHYSICALTRITSPERWORD = ((EFFECTIVETRITSPERWORD + 4) / 5) * 5, // Round up number of trits per word, cut excess
      parameter int unsigned PHYSICALBITSPERWORD = PHYSICALTRITSPERWORD / 5 * 8,
      parameter int unsigned EXCESSBITS = (PHYSICALTRITSPERWORD - EFFECTIVETRITSPERWORD)*2,
      parameter int unsigned EFFECTIVEWORDWIDTH = PHYSICALBITSPERWORD - EXCESSBITS,
      parameter int unsigned NUMDECODERSPERBANK = PHYSICALBITSPERWORD/8,

      parameter int unsigned NUMWRITEBANKS = 1 > (N_O/ITERATIVE_DECOMP)/(N_I/WEIGHT_STAGGER) ? 1 : (N_O/ITERATIVE_DECOMP)/(N_I/WEIGHT_STAGGER),
      parameter int unsigned NUMENCODERSPERBANK = ((N_O/ITERATIVE_DECOMP)/NUMWRITEBANKS+4)/5,

      parameter int unsigned NUMBANKS = K*WEIGHT_STAGGER, // Need K*NI trits per cycle
      parameter int unsigned TOTNUMTRITS = IMAGEWIDTH*IMAGEHEIGHT*N_I,
      parameter int unsigned TRITSPERBANK = (TOTNUMTRITS+NUMBANKS-1)/NUMBANKS,
      parameter int unsigned ACTMEMBANKDEPTH = (TRITSPERBANK+EFFECTIVETRITSPERWORD-1)/EFFECTIVETRITSPERWORD,
      parameter int unsigned ACTMEMBANKADDRESSDEPTH = $clog2(ACTMEMBANKDEPTH),

      parameter int unsigned LEFTSHIFTBITWIDTH = NUMBANKS > 1 ? $clog2(NUMBANKS) : 1,
      parameter int unsigned BANKSETSBITWIDTH = NUMACTMEMBANKSETS > 1 ? $clog2(NUMACTMEMBANKSETS) : 1,
      parameter int unsigned SPLITBITWIDTH = $clog2(WEIGHT_STAGGER)+1,

      parameter int unsigned COLADDRESSWIDTH = $clog2(IMAGEWIDTH),
      parameter int unsigned ROWADDRESSWIDTH = $clog2(IMAGEHEIGHT),
      parameter int unsigned TBROWADDRESSWIDTH = $clog2(K),

      parameter int unsigned WEIGHTMEMFULLADDRESSBITWIDTH = $clog2(WEIGHTBANKDEPTH),
      parameter int unsigned ACTMEMFULLADDRESSBITWIDTH = $clog2(NUMBANKS*ACTMEMBANKDEPTH)

     )
   (
     input logic							      clk_i,
     input logic							      rst_ni,
    ///////////////////////////////// External actmem access signals /////////////////////////////////
     input logic [BANKSETSBITWIDTH-1:0]					      actmem_external_bank_set_i,
     input logic							      actmem_external_we_i,
     input logic							      actmem_external_req_i,
     input logic [ACTMEMFULLADDRESSBITWIDTH-1:0]			      actmem_external_addr_i,
     input logic [PHYSICALBITSPERWORD-1:0]				      actmem_external_wdata_i,

    ///////////////////////////////// External weightmem access signals /////////////////////////////////
     input logic [$clog2(N_O/ITERATIVE_DECOMP)-1:0]			      weightmem_external_bank_i,
     input logic							      weightmem_external_we_i,
     input logic							      weightmem_external_req_i,
     input logic [WEIGHTMEMFULLADDRESSBITWIDTH-1:0]			      weightmem_external_addr_i,
     input logic [PHYSICALBITSPERWORD-1:0]				      weightmem_external_wdata_i,

     input logic signed [$clog2(K*K*N_I):0]				      ocu_thresh_pos_i, ocu_thresh_neg_i,
    ///////////////////////////////// Signals with a leading underscore are not yet final, might still change /////////////////////////////////
     input logic [0:(N_O/ITERATIVE_DECOMP)-1]				      ocu_thresholds_save_enable_i,

     input logic							      LUCA_store_to_fifo_i,
     input logic							      LUCA_testmode_i,
     input logic unsigned [$clog2(IMAGEWIDTH):0]			      LUCA_layer_imagewidth_i,
     input logic unsigned [$clog2(IMAGEHEIGHT):0]			      LUCA_layer_imageheight_i,
     input logic unsigned [$clog2(K):0]					      LUCA_layer_k_i,
     input logic unsigned [$clog2(N_I):0]				      LUCA_layer_ni_i,
     input logic unsigned [$clog2(N_O):0]				      LUCA_layer_no_i,
     input logic unsigned [$clog2(K)-1:0]				      LUCA_layer_stride_width_i,
     input logic unsigned [$clog2(K)-1:0]				      LUCA_layer_stride_height_i,
     input logic							      LUCA_layer_padding_type_i,
     input logic							      LUCA_pooling_enable_i,
     input logic							      LUCA_pooling_pooling_type_i,
     input logic unsigned [$clog2(K)-1:0]				      LUCA_pooling_kernel_i,
     input logic							      LUCA_pooling_padding_type_i,
     input logic							      LUCA_layer_skip_in_i,
     input logic							      LUCA_layer_skip_out_i,
     input logic							      LUCA_layer_is_tcn_i,
     input logic [$clog2(TCN_WIDTH)-1:0]				      LUCA_layer_tcn_width_i,
     input logic [$clog2(TCN_WIDTH)-1:0]				      LUCA_layer_tcn_width_mod_dil_i,
     input logic [$clog2(K)-1:0]					      LUCA_layer_tcn_k_i,

     input logic							      LUCA_compute_disable_i,

     output logic [PHYSICALBITSPERWORD-1:0]				      actmem_external_acts_o,
     output logic							      actmem_external_valid_o,

     output logic [PHYSICALBITSPERWORD-1:0]				      weightmem_external_weights_o,
     output logic							      weightmem_external_valid_o,

     output logic [0:PIPELINEDEPTH-1][0:(PIPELINEWIDTH)-1][$clog2(K*K*N_I):0] fp_output_o,

     output logic							      compute_done_o
    /*AUTOARG*/);

   import enums_linebuffer::*;
   import enums_conv_layer::*;
   import cutie_params::*;

   logic                                                                         clk, rst;

   assign clk = clk_i;
   assign rst = rst_ni;

   ///////////////////////////////// ACTMEM WRITE CONTROLLER SIGNALS /////////////////////////////////

   logic                                                                         actmem_write_ctrl_latch_new_layer_i;
   logic [$clog2(N_O):0]							 actmem_write_ctrl_layer_no_i;
   logic [0:NUMWRITEBANKS-1][PHYSICALBITSPERWORD-1:0]				 actmem_write_ctrl_wdata_in_i;
   logic									 actmem_write_ctrl_valid_i;

   logic [0:NUMBANKS-1][PHYSICALBITSPERWORD-1:0]				 actmem_write_ctrl_wdata_out_o;
   logic [0:NUMBANKS-1]								 actmem_write_ctrl_write_enable_o;
   logic [0:NUMBANKS-1][ACTMEMBANKADDRESSDEPTH-1:0]				 actmem_write_ctrl_write_addr_o;

   ///////////////////////////////// END ACTMEM WRITE CONTROLLER SIGNALS /////////////////////////////////

   ///////////////////////////////// ACTMEM SIGNALS /////////////////////////////////

   logic [0:NUMBANKS-1]								 actmem_read_enable_i;
   logic [0:BANKSETSBITWIDTH-1]							 actmem_read_enable_bank_set_i;
   logic [0:NUMBANKS-1][$clog2(ACTMEMBANKDEPTH)-1:0]				 actmem_read_addr_i;

   logic [0:NUMBANKS-1]								 actmem_write_enable_i;
   logic [0:BANKSETSBITWIDTH-1]							 actmem_write_enable_bank_set_i;
   logic [0:NUMBANKS-1][$clog2(ACTMEMBANKDEPTH)-1:0]				 actmem_write_addr_i;

   logic [0:NUMBANKS-1][PHYSICALBITSPERWORD-1:0]				 actmem_wdata_i;

   logic [LEFTSHIFTBITWIDTH-1:0]						 actmem_left_shift_i;
   logic [SPLITBITWIDTH-1:0]							 actmem_scatter_coefficient_i;
   logic [$clog2(WEIGHT_STAGGER):0]						 actmem_pixelwidth_i;
   logic									 actmem_tcn_actmem_set_shift_i;
   logic [$clog2(TCN_WIDTH)-1:0]						 actmem_tcn_actmem_read_shift_i;
   logic [$clog2(TCN_WIDTH)-1:0]						 actmem_tcn_actmem_write_shift_i;


   logic [0:NUMBANKS-1]								 actmem_ready_o;
   logic [0:NUMBANKS-1]								 actmem_rw_collision_o;
   logic [0:K-1][0:N_I-1][1:0]							 actmem_acts_o;

   ///////////////////////////////// END ACTMEM SIGNALS /////////////////////////////////

   // logic [0:COMPUTEDELAY-1]                           pipelined_tilebuffer_controller_new_layer_ready_q, pipelined_tilebuffer_controller_new_layer_ready_d;

   ///////////////////////////////// LINEBUFFER SIGNALS /////////////////////////////////
   logic [0:K-1][0:N_I-1][1:0]							 linebuffer_acts_i;
   logic									 linebuffer_flush_i;
   logic									 linebuffer_valid_i;
   logic [COLADDRESSWIDTH:0]							 linebuffer_layer_imagewidth_i;
   logic [ROWADDRESSWIDTH:0]							 linebuffer_layer_imageheight_i;
   logic									 linebuffer_write_enable_i;
   logic									 linebuffer_wrap_around_save_enable_i;
   logic [COLADDRESSWIDTH-1:0]							 linebuffer_write_col_i;
   logic									 linebuffer_read_enable_i;
   logic [COLADDRESSWIDTH-1:0]							 linebuffer_read_col_i;
   logic [TBROWADDRESSWIDTH-1:0]						 linebuffer_read_row_i;
   logic [0:K-1][0:K-1][0:N_I-1][1:0]						 linebuffer_acts_o;
   ///////////////////////////////// LINEBUFFER SIGNALS END /////////////////////////////////


   //////////////////////////// LINEBUFFER MASTER CONTROLLER  SIGNALS ///////////////////////////////
   logic									 linebuffer_master_controller_new_layer_i;
   logic									 linebuffer_master_controller_valid_i;
   logic									 linebuffer_master_controller_ready_i;
   logic [$clog2(K)-1:0]							 linebuffer_master_controller_layer_stride_width_i;
   logic [$clog2(K)-1:0]							 linebuffer_master_controller_layer_stride_height_i;
   enums_conv_layer::padding_type                     linebuffer_master_controller_layer_padding_type_i;
   logic [COLADDRESSWIDTH:0]                                                     linebuffer_master_controller_layer_imagewidth_i;
   logic [ROWADDRESSWIDTH:0]							 linebuffer_master_controller_layer_imageheight_i;
   logic [$clog2(N_I):0]							 linebuffer_master_controller_layer_ni_i;
   logic									 linebuffer_master_controller_layer_is_tcn_i;
   logic [$clog2(TCN_WIDTH)-1:0]						 linebuffer_master_controller_layer_tcn_width_i;
   logic [$clog2(TCN_WIDTH)-1:0]						 linebuffer_master_controller_layer_tcn_width_mod_dil_i;
   logic [$clog2(K)-1:0]							 linebuffer_master_controller_layer_tcn_k_i;
   logic									 linebuffer_master_controller_ready_read_o;
   logic									 linebuffer_master_controller_ready_write_o;
   logic [COLADDRESSWIDTH-1:0]							 linebuffer_master_controller_read_col_o;
   logic [ROWADDRESSWIDTH-1:0]							 linebuffer_master_controller_read_row_o;
   logic [COLADDRESSWIDTH-1:0]							 linebuffer_master_controller_write_col_o;
   logic [ROWADDRESSWIDTH-1:0]							 linebuffer_master_controller_write_row_o;
   logic									 linebuffer_master_controller_wrap_around_save_enable_o;
   logic									 linebuffer_master_controller_done_o;
   logic									 linebuffer_master_controller_flush_o;
   ///////////////////////////// LINEBUFFER MASTER CONTROLLER  SIGNALS END ////////////////////////

   ///////////////////////////////// ACTMEM2LB CONTROLLER SIGNALS /////////////////////////////////
   logic									 actmem2lb_controller_new_layer_i;
   logic [$clog2(K)-1:0]							 actmem2lb_controller_layer_stride_width_i;
   logic [$clog2(K)-1:0]							 actmem2lb_controller_layer_stride_height_i;
   enums_conv_layer::padding_type                     actmem2lb_controller_layer_padding_type_i;
   logic [COLADDRESSWIDTH:0]                                                     actmem2lb_controller_layer_imagewidth_i;
   logic [ROWADDRESSWIDTH:0]							 actmem2lb_controller_layer_imageheight_i;
   logic [$clog2(N_I):0]							 actmem2lb_controller_layer_ni_i;
   logic									 actmem2lb_controller_ready_i;
   logic									 actmem2lb_controller_valid_i;
   logic [COLADDRESSWIDTH-1:0]							 actmem2lb_controller_write_col_i;
   logic [ROWADDRESSWIDTH-1:0]							 actmem2lb_controller_write_row_i;
   logic									 actmem2lb_controller_wrap_around_save_enable_i;
   logic [0:NUMBANKS-1]								 actmem2lb_controller_read_enable_vector_o;
   logic [0:NUMBANKS-1][ACTMEMBANKADDRESSDEPTH-1:0]				 actmem2lb_controller_read_addr_o;
   logic [LEFTSHIFTBITWIDTH-1:0]						 actmem2lb_controller_left_shift_o;
   logic [SPLITBITWIDTH-1:0]							 actmem2lb_controller_scatter_coefficient_o;
   ///////////////////////////////// ACTMEM2LB CONTROLLER SIGNALS END /////////////////////////////////

   ///////////////////////////////// LB2OCU CONTROLLER SIGNALS /////////////////////////////////
   logic									 lb2ocu_controller_new_layer_i;
   logic [$clog2(K)-1:0]							 lb2ocu_controller_layer_stride_width_i;
   logic [$clog2(K)-1:0]							 lb2ocu_controller_layer_stride_height_i;
   enums_conv_layer::padding_type                     lb2ocu_controller_layer_padding_type_i;
   logic [COLADDRESSWIDTH:0]                                                     lb2ocu_controller_layer_imagewidth_i;
   logic [ROWADDRESSWIDTH:0]							 lb2ocu_controller_layer_imageheight_i;
   logic [$clog2(N_I):0]							 lb2ocu_controller_layer_ni_i;
   logic									 lb2ocu_controller_ready_i;
   logic [COLADDRESSWIDTH-1:0]							 lb2ocu_controller_read_col_i;
   logic [ROWADDRESSWIDTH-1:0]							 lb2ocu_controller_read_row_i;
   logic [COLADDRESSWIDTH-1:0]							 lb2ocu_controller_read_col_o;
   logic [TBROWADDRESSWIDTH-1:0]						 lb2ocu_controller_read_row_o;
   ///////////////////////////////// LB2OCU CONTROLLER SIGNALS END /////////////////////////////////


   ///////////////////////////////// LUCA SIGNALS /////////////////////////////////

   logic									 LUCA_tilebuffer_done_i;
   logic [0:PIPELINEDEPTH-1]							 LUCA_weightload_done_i;

   logic									 LUCA_testmode_o;
   logic									 LUCA_compute_latch_new_layer_o;
   logic									 LUCA_compute_soft_reset_o;
   logic [$clog2(IMAGEWIDTH):0]							 LUCA_compute_imagewidth_o;
   logic [$clog2(IMAGEHEIGHT):0]						 LUCA_compute_imageheight_o;
   logic [$clog2(K):0]								 LUCA_compute_k_o;
   logic [$clog2(N_I):0]							 LUCA_compute_ni_o;
   logic [$clog2(N_O):0]							 LUCA_compute_no_o;
   logic [$clog2(K)-1:0]							 LUCA_stride_width_o;
   logic [$clog2(K)-1:0]							 LUCA_stride_height_o;
   logic									 LUCA_padding_type_o;
   logic									 LUCA_compute_is_tcn_o;
   logic [$clog2(TCN_WIDTH)-1:0]						 LUCA_compute_tcn_width_o;
   logic [$clog2(TCN_WIDTH)-1:0]						 LUCA_compute_tcn_width_mod_dil_o;
   logic [$clog2(K)-1:0]							 LUCA_compute_tcn_k_o;
   logic [$clog2(WEIGHT_STAGGER):0]						 LUCA_pixelwidth_o;
   logic									 LUCA_tcn_actmem_set_shift_o;
   logic [$clog2(TCN_WIDTH)-1:0]						 LUCA_tcn_actmem_read_shift_o;
   logic [$clog2(TCN_WIDTH)-1:0]						 LUCA_tcn_actmem_write_shift_o;

   logic [$clog2(ITERATIVE_DECOMP):0]						 LUCA_output_offset_o;
   logic [$clog2(ITERATIVE_DECOMP):0]						 LUCA_output_stride_o;

   logic									 LUCA_pooling_enable_o;
   logic									 LUCA_pooling_pooling_type_o;
   logic [$clog2(K)-1:0]							 LUCA_pooling_kernel_o;
   logic									 LUCA_pooling_padding_type_o;

   logic									 LUCA_layer_skip_in_o;
   logic									 LUCA_layer_skip_out_o;

   logic [$clog2(NUMACTMEMBANKSETS)-1:0]					 LUCA_readbank_o;
   logic [$clog2(NUMACTMEMBANKSETS)-1:0]					 LUCA_writebank_o;
   logic [0:PIPELINEDEPTH-1]							 LUCA_weights_latch_new_layer_o;
   logic [$clog2(K):0]								 LUCA_weights_k_o;
   logic [$clog2(N_I):0]							 LUCA_weights_ni_o;
   logic [$clog2(N_O):0]							 LUCA_weights_no_o;

   logic [0:PIPELINEDEPTH-1]							 LUCA_weights_soft_reset_o;
   logic [0:PIPELINEDEPTH-1]							 LUCA_weights_toggle_banks_o;
   logic									 LUCA_fifo_pop_o;
   logic									 LUCA_compute_done_o;

   ///////////////////////////////// END LUCA SIGNALS /////////////////////////////////

   ///////////////////////////////// WEIGHTMEMORY HELPER SIGNALS /////////////////////////////////

   logic [0:(N_O/ITERATIVE_DECOMP)-1][PHYSICALBITSPERWORD-1:0]			 weightmemory_external_weights_vector;
   logic [0:(N_O/ITERATIVE_DECOMP)-1]						 weightmemory_external_valid_vector;

   logic [0:(N_O/ITERATIVE_DECOMP)-1]						 weightmemory_external_we_vector;
   logic [0:(N_O/ITERATIVE_DECOMP)-1]						 weightmemory_external_req_vector;
   logic [0:(N_O/ITERATIVE_DECOMP)-1] [WEIGHTMEMFULLADDRESSBITWIDTH-1:0]	 weightmemory_external_addr_vector;
   logic [0:(N_O/ITERATIVE_DECOMP)-1] [PHYSICALBITSPERWORD-1:0]			 weightmemory_external_wdata_vector;

   logic [0:(N_O/ITERATIVE_DECOMP)-1]						 weightmemory_write_enable;
   logic [PIPELINEDEPTH-1:0][0:(PIPELINEWIDTH)-1]				 __weightmemory_write_enable; // Activation memory outputs are valid

   logic [$clog2(N_O)-1:0]							 weightmemory_external_bank_q;

   logic [$clog2(N_O)-1:0]							 weightmemory_bank_helper;
   logic [0:(N_O/ITERATIVE_DECOMP)-1]						 weightmemory_enable_helper;

   ///////////////////////////////// END WEIGHTMEMORY HELPER SIGNALS /////////////////////////////////

   logic [0:PIPELINEDEPTH-1][0:(PIPELINEWIDTH)-1]				 weightmemory_controller_ready_out; // OCU is ready for inputs
   logic [0:PIPELINEDEPTH-1][0:(PIPELINEWIDTH)-1]				 weightmemory_controller_valid_out; // Activation memory outputs are valid
   logic [0:PIPELINEDEPTH-1][0:(PIPELINEWIDTH)-1][1:0]				 pipelined_compute_output;

   ///////////////////////////////// INPUT REGISTER FOR PIPELINE /////////////////////////////////

   logic [0:K-1][0:K-1][0:N_I-1][1:0]						 acts_buffer_q;
   logic									 tilebuffer_valid_q;

   logic									 compute_output_valid_o;


   ///////////////////////////////// END INPUT REGISTER FOR PIPELINE /////////////////////////////////


   always_ff @(posedge clk_i, negedge rst_ni) begin
      if (~rst_ni) begin
	 acts_buffer_q <= '0;
	 tilebuffer_valid_q <= '0;
      end else begin
	 tilebuffer_valid_q <= linebuffer_master_controller_ready_read_o;
	 if(linebuffer_master_controller_ready_read_o == 1) begin
	    acts_buffer_q <= linebuffer_acts_o;
	 end
      end
   end // always_ff @ (posedge clk, negedge rst)



   actmem_write_controller
     #(
       .N_O(N_O),
       .N_I(N_I),
       .K(K),
       .WEIGHT_STAGGER(WEIGHT_STAGGER),
       .IMAGEWIDTH(IMAGEWIDTH),
       .IMAGEHEIGHT(IMAGEHEIGHT),
       .NUMACTMEMBANKSETS(NUMACTMEMBANKSETS),
       .ITERATIVE_DECOMP(ITERATIVE_DECOMP)
       ) actmem_write_ctrl (
			    .clk_i(clk),
			    .rst_ni(rst),
			    .latch_new_layer_i(actmem_write_ctrl_latch_new_layer_i),
			    .layer_offset_i(LUCA_output_offset_o),
			    .layer_stride_i(LUCA_output_stride_o),
			    .layer_no_i(actmem_write_ctrl_layer_no_i),
			    .pipeline_outputs_i(pipelined_compute_output),
			    .valid_i(actmem_write_ctrl_valid_i),

			    .wdata_o(actmem_write_ctrl_wdata_out_o),
			    .write_enable_o(actmem_write_ctrl_write_enable_o),
			    .write_addr_o(actmem_write_ctrl_write_addr_o)
			    /*AUTOINST*/);


   activationmemory_external_wrapper
     #(
       .N_I(N_I),
       .N_O(N_O),
       .K(K),
       .WEIGHT_STAGGER(WEIGHT_STAGGER),
       .IMAGEWIDTH(IMAGEWIDTH),
       .IMAGEHEIGHT(IMAGEHEIGHT),
       .TCN_WIDTH(TCN_WIDTH),
       .NUMBANKSETS(NUMACTMEMBANKSETS)
       ) actmem (
		 .clk_i(clk),
		 .rst_ni(rst),
		 .external_bank_set_i(actmem_external_bank_set_i),
		 .external_we_i(actmem_external_we_i),
		 .external_req_i(actmem_external_req_i),
		 .external_addr_i(actmem_external_addr_i),
		 .external_wdata_i(actmem_external_wdata_i),
		 .read_enable_i(actmem_read_enable_i),
		 .read_enable_bank_set_i(actmem_read_enable_bank_set_i),
		 .read_addr_i(actmem_read_addr_i),
		 .wdata_i(actmem_wdata_i),
		 .write_addr_i(actmem_write_addr_i),
		 .write_enable_i(actmem_write_enable_i),
		 .write_enable_bank_set_i(actmem_write_enable_bank_set_i),
		 .left_shift_i(actmem_left_shift_i),
		 .scatter_coefficient_i(actmem_scatter_coefficient_i),
		 .pixelwidth_i(actmem_pixelwidth_i),
		 .tcn_actmem_set_shift_i(actmem_tcn_actmem_set_shift_i),
		 .tcn_actmem_read_shift_i(actmem_tcn_actmem_read_shift_i),
		 .tcn_actmem_write_shift_i(actmem_tcn_actmem_write_shift_i),
		 .valid_o(actmem_ready_o),
		 .rw_collision_o(actmem_rw_collision_o),
		 .acts_o(actmem_acts_o),
		 .external_acts_o(actmem_external_acts_o),
		 .external_valid_o(actmem_external_valid_o)
		 /*AUTOINST*/);

   ///////////////////////////////// SYSTEM-LEVEL CONTROL /////////////////////////////////

   LUCA
     #(
       .K(K),
       .N_I(N_I),
       .N_O(N_O),
       .PIPELINEDEPTH(PIPELINEDEPTH),
       .IMAGEWIDTH(IMAGEWIDTH),
       .IMAGEHEIGHT(IMAGEHEIGHT),
       .TCN_WIDTH(TCN_WIDTH),
       .LAYER_FIFODEPTH(LAYER_FIFODEPTH),
       .NUMACTMEMBANKSETS(NUMACTMEMBANKSETS),
       .NUMBANKS(NUMBANKS),
       .ITERATIVE_DECOMP(ITERATIVE_DECOMP)
       ) LUCA (
	       .clk_i(clk),
	       .rst_ni(rst),
	       .store_to_fifo_i(LUCA_store_to_fifo_i),
	       .testmode_i(LUCA_testmode_i),
	       .layer_imagewidth_i(LUCA_layer_imagewidth_i),
	       .layer_imageheight_i(LUCA_layer_imageheight_i),
	       .layer_k_i(LUCA_layer_k_i),
	       .layer_ni_i(LUCA_layer_ni_i),
	       .layer_no_i(LUCA_layer_no_i),
	       .layer_stride_width_i(LUCA_layer_stride_width_i),
	       .layer_stride_height_i(LUCA_layer_stride_height_i),
	       .layer_padding_type_i(LUCA_layer_padding_type_i),
	       .layer_pooling_enable_i(LUCA_pooling_enable_i),
	       .layer_pooling_pooling_type_i(LUCA_pooling_pooling_type_i),
	       .layer_pooling_kernel_i(LUCA_pooling_kernel_i),
	       .layer_pooling_padding_type_i(LUCA_pooling_padding_type_i),
	       .layer_skip_out_i(LUCA_layer_skip_out_i),
	       .layer_skip_in_i(LUCA_layer_skip_in_i),
	       .layer_is_tcn_i(LUCA_layer_is_tcn_i),
	       .layer_tcn_width_i(LUCA_layer_tcn_width_i),
	       .layer_tcn_width_mod_dil_i(LUCA_layer_tcn_width_mod_dil_i),
	       .layer_tcn_k_i(LUCA_layer_tcn_k_i),
	       .compute_disable_i(LUCA_compute_disable_i),
	       .tilebuffer_done_i(LUCA_tilebuffer_done_i),
	       .weightload_done_i(LUCA_weightload_done_i),
	       .testmode_o(LUCA_testmode_o),
	       .compute_latch_new_layer_o(LUCA_compute_latch_new_layer_o),
	       .compute_soft_reset_o(LUCA_compute_soft_reset_o),
	       .compute_imagewidth_o(LUCA_compute_imagewidth_o),
	       .compute_imageheight_o(LUCA_compute_imageheight_o),
	       .compute_k_o(LUCA_compute_k_o),
	       .compute_ni_o(LUCA_compute_ni_o),
	       .compute_no_o(LUCA_compute_no_o),
	       .stride_width_o(LUCA_stride_width_o),
	       .stride_height_o(LUCA_stride_height_o),
	       .padding_type_o(LUCA_padding_type_o),
	       .pooling_enable_o(LUCA_pooling_enable_o),
	       .pooling_pooling_type_o(LUCA_pooling_pooling_type_o),
	       .pooling_kernel_o(LUCA_pooling_kernel_o),
	       .pooling_padding_type_o(LUCA_pooling_padding_type_o),
	       .skip_out_o(LUCA_layer_skip_out_o),
	       .skip_in_o(LUCA_layer_skip_in_o),
	       .compute_is_tcn_o(LUCA_compute_is_tcn_o),
	       .compute_tcn_width_o(LUCA_compute_tcn_width_o),
	       .compute_tcn_width_mod_dil_o(LUCA_compute_tcn_width_mod_dil_o),
	       .compute_tcn_k_o(LUCA_compute_tcn_k_o),
	       .output_offset_o(LUCA_output_offset_o),
	       .output_stride_o(LUCA_output_stride_o),
	       .readbank_o(LUCA_readbank_o),
	       .writebank_o(LUCA_writebank_o),
	       .pixelwidth_o(LUCA_pixelwidth_o),
	       .tcn_actmem_set_shift_o(LUCA_tcn_actmem_set_shift_o),
	       .tcn_actmem_read_shift_o(LUCA_tcn_actmem_read_shift_o),
	       .tcn_actmem_write_shift_o(LUCA_tcn_actmem_write_shift_o),
	       .weights_latch_new_layer_o(LUCA_weights_latch_new_layer_o),
	       .weights_k_o(LUCA_weights_k_o),
	       .weights_ni_o(LUCA_weights_ni_o),
	       .weights_no_o(LUCA_weights_no_o),
	       .weights_soft_reset_o(LUCA_weights_soft_reset_o),
	       .weights_toggle_banks_o(LUCA_weights_toggle_banks_o),
	       .fifo_pop_o(LUCA_fifo_pop_o),
	       .compute_done_o(LUCA_compute_done_o)
	       /*AUTOINST*/
	       );

   linebuffer
     #(/*AUTOINSTPARAM*/
       // Parameters
       .N_I                    (N_I),
       .K                      (K),
       .IMAGEWIDTH             (IMAGEWIDTH),
       .IMAGEHEIGHT            (IMAGEHEIGHT),
       .COLADDRESSWIDTH        (COLADDRESSWIDTH),
       .ROWADDRESSWIDTH        (ROWADDRESSWIDTH),
       .TBROWADDRESSWIDTH      (TBROWADDRESSWIDTH))
   linebuffer (/*AUTOINST*/
	       // Outputs
	       .acts_o                (linebuffer_acts_o),
	       // Inputs
	       .clk_i                 (clk_i),
	       .rst_ni                (rst_ni),
	       .acts_i                (linebuffer_acts_i),
	       .flush_i               (linebuffer_flush_i),
	       .valid_i               (linebuffer_valid_i),
	       .layer_imagewidth_i    (linebuffer_layer_imagewidth_i),
	       .layer_imageheight_i   (linebuffer_layer_imageheight_i),
	       .write_enable_i        (linebuffer_write_enable_i),
	       .wrap_around_save_enable_i(linebuffer_wrap_around_save_enable_i),
	       .write_col_i           (linebuffer_write_col_i),
	       .read_enable_i         (linebuffer_read_enable_i),
	       .read_col_i            (linebuffer_read_col_i),
	       .read_row_i            (linebuffer_read_row_i));

   linebuffer_master_controller
     #(/*AUTOINSTPARAM*/
       // Parameters
       .N_I                  (N_I),
       .K                    (K),
       .IMAGEWIDTH           (IMAGEWIDTH),
       .IMAGEHEIGHT          (IMAGEHEIGHT),
       .COLADDRESSWIDTH      (COLADDRESSWIDTH),
       .ROWADDRESSWIDTH      (ROWADDRESSWIDTH),
       .TCN_WIDTH            (TCN_WIDTH)
       )
   linebuffer_master_controller (/*AUTOINST*/
				 // Outputs
				 .ready_read_o          (linebuffer_master_controller_ready_read_o),
				 .ready_write_o         (linebuffer_master_controller_ready_write_o),
				 .read_col_o            (linebuffer_master_controller_read_col_o),
				 .read_row_o            (linebuffer_master_controller_read_row_o),
				 .write_col_o           (linebuffer_master_controller_write_col_o),
				 .write_row_o           (linebuffer_master_controller_write_row_o),
				 .wrap_around_save_enable_o(linebuffer_master_controller_wrap_around_save_enable_o),
				 .done_o                (linebuffer_master_controller_done_o),
				 .flush_o               (linebuffer_master_controller_flush_o),
				 // Inputs
				 .clk_i                 (clk_i),
				 .rst_ni                (rst_ni),
				 .new_layer_i           (linebuffer_master_controller_new_layer_i),
				 .valid_i               (linebuffer_master_controller_valid_i),
				 .ready_i               (linebuffer_master_controller_ready_i),
				 .layer_stride_width_i  (linebuffer_master_controller_layer_stride_width_i),
				 .layer_stride_height_i (linebuffer_master_controller_layer_stride_height_i),
				 .layer_padding_type_i  (linebuffer_master_controller_layer_padding_type_i),
				 .layer_imagewidth_i    (linebuffer_master_controller_layer_imagewidth_i),
				 .layer_imageheight_i   (linebuffer_master_controller_layer_imageheight_i),
				 .layer_ni_i            (linebuffer_master_controller_layer_ni_i),
				 .layer_is_tcn_i        (linebuffer_master_controller_layer_is_tcn_i),
				 .layer_tcn_width_mod_dil_i  (linebuffer_master_controller_layer_tcn_width_mod_dil_i),
				 .layer_tcn_k_i         (linebuffer_master_controller_layer_tcn_k_i));

   actmem2lb_controller
     #(/*AUTOINSTPARAM*/
       // Parameters
       .N_I                  (N_I),
       .K                    (K),
       .IMAGEWIDTH           (IMAGEWIDTH),
       .IMAGEHEIGHT          (IMAGEHEIGHT),
       .COLADDRESSWIDTH      (COLADDRESSWIDTH),
       .ROWADDRESSWIDTH      (ROWADDRESSWIDTH),
       .WEIGHT_STAGGER       (WEIGHT_STAGGER),
       .NUMACTMEMBANKSETS    (NUMACTMEMBANKSETS),
       .EFFECTIVETRITSPERWORD(EFFECTIVETRITSPERWORD),
       .PHYSICALTRITSPERWORD (PHYSICALTRITSPERWORD),
       .PHYSICALBITSPERWORD  (PHYSICALBITSPERWORD),
       .EXCESSBITS           (EXCESSBITS),
       .EFFECTIVEWORDWIDTH   (EFFECTIVEWORDWIDTH),
       .NUMDECODERSPERBANK   (NUMDECODERSPERBANK),
       .NUMBANKS             (NUMBANKS),
       .TOTNUMTRITS          (TOTNUMTRITS),
       .TRITSPERBANK         (TRITSPERBANK),
       .BANKDEPTH            (ACTMEMBANKDEPTH),
       .LEFTSHIFTBITWIDTH    (LEFTSHIFTBITWIDTH),
       .SPLITWIDTH           (SPLITBITWIDTH))
   actmem2lb_controller (/*AUTOINST*/
			 // Outputs
			 .read_enable_vector_o(actmem2lb_controller_read_enable_vector_o),
			 .read_addr_o        (actmem2lb_controller_read_addr_o),
			 .left_shift_o       (actmem2lb_controller_left_shift_o),
			 .scatter_coefficient_o(actmem2lb_controller_scatter_coefficient_o),
			 // Inputs
			 .clk_i              (clk_i),
			 .rst_ni             (rst_ni),
			 .new_layer_i        (actmem2lb_controller_new_layer_i),
			 .layer_imagewidth_i (actmem2lb_controller_layer_imagewidth_i),
			 .layer_imageheight_i(actmem2lb_controller_layer_imageheight_i),
			 .layer_ni_i         (actmem2lb_controller_layer_ni_i),
			 .ready_i            (actmem2lb_controller_ready_i),
			 .valid_i            (actmem2lb_controller_valid_i),
			 .write_col_i        (actmem2lb_controller_write_col_i),
			 .write_row_i        (actmem2lb_controller_write_row_i),
			 .wrap_around_save_enable_i(actmem2lb_controller_wrap_around_save_enable_i));

   lb2ocu_controller
     #(/*AUTOINSTPARAM*/
       // Parameters
       .N_I             (N_I),
       .K               (K),
       .IMAGEWIDTH      (IMAGEWIDTH),
       .IMAGEHEIGHT     (IMAGEHEIGHT),
       .COLADDRESSWIDTH (COLADDRESSWIDTH),
       .ROWADDRESSWIDTH (ROWADDRESSWIDTH),
       .TBROWADDRESSWIDTH(TBROWADDRESSWIDTH))
   lb2ocu_controller (/*AUTOINST*/
		      // Outputs
		      .read_col_o         (lb2ocu_controller_read_col_o),
		      .read_row_o         (lb2ocu_controller_read_row_o),
		      // Inputs
		      .clk_i                 (clk_i),
		      .rst_ni                (rst_ni),
		      .new_layer_i           (lb2ocu_controller_new_layer_i),
		      .layer_stride_width_i  (lb2ocu_controller_layer_stride_width_i),
		      .layer_stride_height_i (lb2ocu_controller_layer_stride_height_i),
		      .layer_padding_type_i  (lb2ocu_controller_layer_padding_type_i),
		      .layer_imagewidth_i    (lb2ocu_controller_layer_imagewidth_i),
		      .layer_imageheight_i   (lb2ocu_controller_layer_imageheight_i),
		      .layer_ni_i            (lb2ocu_controller_layer_ni_i),
		      .ready_i               (lb2ocu_controller_ready_i),
		      .read_col_i            (lb2ocu_controller_read_col_i),
		      .read_row_i            (lb2ocu_controller_read_row_i));

   ///////////////////////////////// END SYSTEM-LEVEL CONTROL /////////////////////////////////

   compute_core #(
		  .K(K),
		  .N_I(N_I),
		  .N_O(N_O),
		  .IMAGEWIDTH(IMAGEWIDTH),
		  .IMAGEHEIGHT(IMAGEHEIGHT),
		  .ITERATIVE_DECOMP(ITERATIVE_DECOMP),
		  .PIPELINEDEPTH(PIPELINEDEPTH),
		  .PIPELINEWIDTH(PIPELINEWIDTH),
		  .WEIGHT_STAGGER(WEIGHT_STAGGER),
		  .NUM_LAYERS(NUM_LAYERS),
		  .WEIGHTBANKDEPTH(WEIGHTBANKDEPTH),
		  .PHYSICALBITSPERWORD(PHYSICALBITSPERWORD),
		  .EFFECTIVETRITSPERWORD(EFFECTIVETRITSPERWORD),
		  .POOLING_FIFODEPTH(POOLING_FIFODEPTH),
		  .THRESHOLD_FIFODEPTH(THRESHOLD_FIFODEPTH),
		  .WEIGHTMEMFULLADDRESSBITWIDTH(WEIGHTMEMFULLADDRESSBITWIDTH),
		  .COMPUTEDELAY(COMPUTEDELAY)
		  ) compute_core_i (
				    .clk_i(clk),
				    .rst_ni(rst),

				    .acts_i(acts_buffer_q),
				    .acts_valid_i(tilebuffer_valid_q),

				    .ocu_thresh_pos_i(ocu_thresh_pos_i),
				    .ocu_thresh_neg_i(ocu_thresh_neg_i),
				    .ocu_thresholds_save_enable_i(ocu_thresholds_save_enable_i),

				    .layer_compute_no_i(LUCA_compute_no_o),
				    .layer_compute_latch_new_layer_i(LUCA_compute_latch_new_layer_o),
				    .layer_imagewidth_i(LUCA_layer_imagewidth_i),
				    .layer_imageheight_i(LUCA_layer_imageheight_i),
				    .layer_pooling_enable_i(LUCA_pooling_enable_i),
				    .layer_pooling_kernel_i(LUCA_pooling_kernel_i),
				    .layer_pooling_pooling_type_i(LUCA_pooling_pooling_type_i),
				    .layer_pooling_padding_type_i(LUCA_pooling_padding_type_i),
				    .layer_skip_in_i(LUCA_layer_skip_in_i),
				    .layer_skip_out_i(LUCA_layer_skip_out_i),

				    .layer_weights_no_i(LUCA_weights_no_o),
				    .layer_weights_ni_i(LUCA_weights_ni_o),
				    .layer_weights_k_i(LUCA_weights_k_o),
				    .layer_weights_latch_new_layer_i(LUCA_weights_latch_new_layer_o),
				    .layer_weights_soft_reset_i(LUCA_weights_soft_reset_o),
				    .layer_weights_toggle_banks_i(LUCA_weights_toggle_banks_o),

				    .weightmemory_external_we_vector_i(weightmemory_external_we_vector),
				    .weightmemory_external_req_vector_i(weightmemory_external_req_vector),
				    .weightmemory_external_addr_vector_i(weightmemory_external_addr_vector),
				    .weightmemory_external_wdata_vector_i(weightmemory_external_wdata_vector),


				    .weightmemory_external_weights_vector_o(weightmemory_external_weights_vector),
				    .weightmemory_external_valid_vector_o(weightmemory_external_valid_vector),

				    .weightmemory_controller_ready_o(weightmemory_controller_ready_out),
				    .weightmemory_controller_valid_o(weightmemory_controller_valid_out),

				    .compute_output_o(pipelined_compute_output),
				    .compute_output_valid_o(compute_output_valid_o),
				    .fp_output_o(fp_output_o)
				    );




   ///////////////////////////////// LUCA SIGNAL CONNECTIONS /////////////////////////////////

   assign LUCA_tilebuffer_done_i = linebuffer_master_controller_done_o;
   //assign LUCA_weightload_done_i = weightmemory_controller_ready_out[0];

   always_comb begin
      for(int i=0;i<PIPELINEDEPTH;i++) begin
	 LUCA_weightload_done_i[i] = weightmemory_controller_ready_out[i][0];
      end
   end

   ///////////////////////////////// END LUCA SIGNAL CONNECTIONS /////////////////////////////////

   ///////////////////////////////// LINEBUFFER SIGNAL CONNECTIONS /////////////////////////////////
   assign linebuffer_acts_i = actmem_acts_o;
   assign linebuffer_flush_i = linebuffer_master_controller_flush_o;
   assign linebuffer_valid_i = (actmem_ready_o>0)&&(actmem_rw_collision_o == '0);
   assign linebuffer_layer_imagewidth_i = LUCA_compute_imagewidth_o;
   assign linebuffer_layer_imageheight_i = LUCA_compute_imageheight_o;
   assign linebuffer_write_enable_i = linebuffer_master_controller_ready_write_o;
   assign linebuffer_wrap_around_save_enable_i = linebuffer_master_controller_wrap_around_save_enable_o;
   assign linebuffer_write_col_i = linebuffer_master_controller_write_col_o;
   assign linebuffer_read_enable_i = linebuffer_master_controller_ready_read_o;
   assign linebuffer_read_col_i = lb2ocu_controller_read_col_o;
   assign linebuffer_read_row_i = lb2ocu_controller_read_row_o;

   ///////////////////////////////// END LINEBUFFER SIGNAL CONNECTIONS /////////////////////////////////

   ///////////////////////////////// ACTMEM WRITE CTRL SIGNAL CONNECTIONS /////////////////////////////////

   assign actmem_write_ctrl_latch_new_layer_i = LUCA_compute_latch_new_layer_o;
   assign actmem_write_ctrl_layer_no_i = LUCA_compute_no_o;

   assign actmem_write_ctrl_valid_i = compute_output_valid_o;

   ///////////////////////////////// END ACTMEM WRITE CTRL SIGNAL CONNECTIONS /////////////////////////////////

   ///////////////////////////////// ACTMEM SIGNAL CONNECTIONS /////////////////////////////////


   assign actmem_read_enable_i = actmem2lb_controller_read_enable_vector_o;
   assign actmem_read_enable_bank_set_i = LUCA_readbank_o;
   assign actmem_read_addr_i = actmem2lb_controller_read_addr_o;
   assign actmem_left_shift_i = actmem2lb_controller_left_shift_o;
   assign actmem_scatter_coefficient_i = actmem2lb_controller_scatter_coefficient_o;
   assign actmem_pixelwidth_i = LUCA_pixelwidth_o;
   assign actmem_tcn_actmem_set_shift_i = LUCA_tcn_actmem_set_shift_o;
   assign actmem_tcn_actmem_read_shift_i = LUCA_tcn_actmem_read_shift_o;
   assign actmem_tcn_actmem_write_shift_i = LUCA_tcn_actmem_write_shift_o;
   assign actmem_wdata_i = actmem_write_ctrl_wdata_out_o;
   assign actmem_write_enable_i = actmem_write_ctrl_write_enable_o;
   assign actmem_write_addr_i = actmem_write_ctrl_write_addr_o;
   assign actmem_write_enable_bank_set_i = LUCA_writebank_o;

   ///////////////////////////////// END ACTMEM SIGNAL CONNECTIONS /////////////////////////////////

   ///////////////////////////////// LINEBUFFER MASTER CONTROLLER SIGNAL CONNECTIONS /////////////////////////////////

   // assign actmem_tilebuffer_controller_testmode_i = LUCA_testmode_o;
   // assign actmem_tilebuffer_controller_valid_i = (actmem_ready_o>0)&&(actmem_rw_collision_o == '0); // Actually valid, need to rename, TODO
   // assign actmem_tilebuffer_controller_ready_i = weightmemory_controller_valid_out[0][0]; // OCU is ready for s
   assign linebuffer_master_controller_new_layer_i = LUCA_compute_latch_new_layer_o;
   assign linebuffer_master_controller_layer_stride_width_i = LUCA_stride_width_o;
   assign linebuffer_master_controller_layer_stride_height_i = LUCA_stride_height_o;
   assign linebuffer_master_controller_layer_padding_type_i = enums_conv_layer::padding_type'(LUCA_padding_type_o);
   assign linebuffer_master_controller_layer_imagewidth_i = LUCA_compute_imagewidth_o;
   assign linebuffer_master_controller_layer_imageheight_i = LUCA_compute_imageheight_o;
   assign linebuffer_master_controller_layer_ni_i = LUCA_compute_ni_o;
   assign linebuffer_master_controller_layer_is_tcn_i = LUCA_compute_is_tcn_o;
   assign linebuffer_master_controller_layer_tcn_width_mod_dil_i = LUCA_compute_tcn_width_mod_dil_o;
   assign linebuffer_master_controller_layer_tcn_k_i = LUCA_compute_tcn_k_o;
   assign linebuffer_master_controller_valid_i = (actmem_ready_o>0)&&(actmem_rw_collision_o == '0);
   assign linebuffer_master_controller_ready_i = weightmemory_controller_valid_out[0][0];
   // TODO: valid_i = (actmem_ready_o>0)&&(actmem_rw_collision_o == '0)
   // TODO: ready_i = weightmemory_controller_valid_out[0][0];
   ////////////////////// END LINEBUFFER MASTER CONTROLLER SIGNAL CONNECTIONS /////////////////////////

   //////////////////////////// ACTMEM2LB CONTROLLER SIGNAL CONNECTIONS ///////////////////////////////
   assign actmem2lb_controller_new_layer_i = LUCA_compute_latch_new_layer_o;
   assign actmem2lb_controller_layer_stride_width_i = LUCA_stride_width_o;
   assign actmem2lb_controller_layer_stride_height_i = LUCA_stride_height_o;
   assign actmem2lb_controller_layer_padding_type_i = enums_conv_layer::padding_type'(LUCA_padding_type_o);
   assign actmem2lb_controller_layer_imagewidth_i = LUCA_compute_imagewidth_o;
   assign actmem2lb_controller_layer_imageheight_i = LUCA_compute_imageheight_o;
   assign actmem2lb_controller_layer_ni_i = LUCA_compute_ni_o;
   assign actmem2lb_controller_ready_i = linebuffer_master_controller_ready_write_o;
   assign actmem2lb_controller_write_col_i = linebuffer_master_controller_write_col_o;
   assign actmem2lb_controller_write_row_i = linebuffer_master_controller_write_row_o;
   assign actmem2lb_controller_wrap_around_save_enable_i = linebuffer_master_controller_wrap_around_save_enable_o;
   // TODO: assign actmem2lb_controller_valid_i =

   ////////////////////////// END ACTMEM2LB CONTROLLER SIGNAL CONNECTIONS //////////////////////////

   ////////////////////////// LB2OCU CONTROLLER SIGNAL CONNECTIONS /////////////////////////////
   assign lb2ocu_controller_new_layer_i = LUCA_compute_latch_new_layer_o;
   assign lb2ocu_controller_layer_stride_width_i = LUCA_stride_width_o;
   assign lb2ocu_controller_layer_stride_height_i = LUCA_stride_height_o;
   assign lb2ocu_controller_layer_padding_type_i = enums_conv_layer::padding_type'(LUCA_padding_type_o);
   assign lb2ocu_controller_layer_imagewidth_i = LUCA_compute_imagewidth_o;
   assign lb2ocu_controller_layer_imageheight_i = LUCA_compute_imageheight_o;
   assign lb2ocu_controller_layer_ni_i = LUCA_compute_ni_o;
   assign lb2ocu_controller_ready_i = linebuffer_master_controller_ready_read_o;
   assign lb2ocu_controller_read_col_i = linebuffer_master_controller_read_col_o;
   assign lb2ocu_controller_read_row_i = linebuffer_master_controller_read_row_o;
   ///////////////////////// END LB2OCU CONTROLLER SIGNAL CONNECTIONS //////////////////////////

   ///////////////////////////////// WEIGHTMEMORY HELPER SIGNAL CONNECTIONS /////////////////////////////////

   always_comb begin : weightmem_external_demultiplex
      weightmemory_external_req_vector = '0;
      weightmemory_external_we_vector = '0;
      weightmemory_external_addr_vector = '0;
      weightmemory_external_wdata_vector = '0;

      weightmemory_external_req_vector[weightmem_external_bank_i] = weightmem_external_req_i;
      weightmemory_external_we_vector[weightmem_external_bank_i] = weightmem_external_we_i;
      weightmemory_external_addr_vector[weightmem_external_bank_i] = weightmem_external_addr_i;
      weightmemory_external_wdata_vector[weightmem_external_bank_i] = weightmem_external_wdata_i;

   end // block: weightmem_external_demultiplex

   ///////////////////////////////// END WEIGHTMEMORY HELPER SIGNAL CONNECTIONS  /////////////////////////////////


   ///////////////////////////////// WEIGHTMEMORY GLUE LOGIC /////////////////////////////////

   always_ff @(posedge clk, negedge rst) begin: external_bank_register
      if(~rst) begin
	 weightmemory_external_bank_q <= '0;
      end else begin
	 weightmemory_external_bank_q <= weightmem_external_bank_i;
      end
   end

   always_comb begin
      weightmemory_bank_helper = weightmem_external_bank_i;
   end

   always_comb begin
      if(weightmem_external_we_i == '1) begin
	 weightmemory_enable_helper = '1;
      end else begin
	 weightmemory_enable_helper = '0;
      end
   end
   assign __weightmemory_write_enable = (weightmemory_write_enable & weightmemory_enable_helper);

   ///////////////////////////////// END WEIGHTMEMORY GLUE LOGIC /////////////////////////////////

   ///////////////////////////////// OUTPUT ASSIGNMENT /////////////////////////////////

   assign weightmem_external_weights_o = weightmemory_external_weights_vector[weightmemory_external_bank_q];
   assign weightmem_external_valid_o = weightmemory_external_valid_vector[weightmemory_external_bank_q];
   assign compute_done_o = LUCA_compute_done_o;

   ///////////////////////////////// END OUTPUT ASSIGNMENTS /////////////////////////////////

endmodule
