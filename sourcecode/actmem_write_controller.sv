// ----------------------------------------------------------------------
//
// File: actmem_write_controller.sv
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
// The actmem_write_controller manages the writeback of valid computed outputs.
// In essence, it calculates how many words need to be written back and counts addresses

module actmem_write_controller
  #(
     parameter int unsigned N_O = 128,
     parameter int unsigned N_I = 128,
     parameter int unsigned WEIGHT_STAGGER = 8,
     parameter int unsigned K = 3,
     parameter int unsigned IMAGEWIDTH = 224,
     parameter int unsigned IMAGEHEIGHT = 224,
     parameter int unsigned NUMACTMEMBANKSETS = 2,
     parameter int unsigned ITERATIVE_DECOMP = 1,

     parameter int unsigned NUMBANKS = K*WEIGHT_STAGGER,
     parameter int unsigned EFFECTIVETRITSPERWORD = N_I/WEIGHT_STAGGER,
     parameter int unsigned PHYSICALTRITSPERWORD = ((EFFECTIVETRITSPERWORD + 4) / 5) * 5, // Round up number of trits per word, cut excess
     parameter int unsigned PHYSICALBITSPERWORD = PHYSICALTRITSPERWORD / 5 * 8,
     parameter int unsigned EXCESSBITS = (PHYSICALTRITSPERWORD - EFFECTIVETRITSPERWORD)*2,
     parameter int unsigned EFFECTIVEWORDWIDTH = PHYSICALBITSPERWORD - EXCESSBITS,

     parameter int unsigned TOTNUMTRITS = IMAGEWIDTH*IMAGEHEIGHT*N_I,
     parameter int unsigned TRITSPERBANK = (TOTNUMTRITS+NUMBANKS-1)/NUMBANKS,

      parameter int unsigned NUMWRITEBANKS = 1 > (N_O/ITERATIVE_DECOMP)/(N_I/WEIGHT_STAGGER) ? 1 : (N_O/ITERATIVE_DECOMP)/(N_I/WEIGHT_STAGGER),
     parameter int unsigned BANKSETSBITWIDTH = NUMACTMEMBANKSETS > 1 ? $clog2(NUMACTMEMBANKSETS) : 1,
     parameter int unsigned ACTMEMBANKDEPTH = (TRITSPERBANK+EFFECTIVETRITSPERWORD-1)/EFFECTIVETRITSPERWORD,
     parameter int unsigned NUMENCODERSPERBANK = ((N_O/ITERATIVE_DECOMP)/NUMWRITEBANKS+4)/5,

     parameter int unsigned ACTMEMBANKADDRESSDEPTH = $clog2(ACTMEMBANKDEPTH)
    )
   (
     input logic						       clk_i,
     input logic						       rst_ni,

     input logic						       latch_new_layer_i,
     input logic [$clog2(ITERATIVE_DECOMP):0]			       layer_offset_i,
     input logic [$clog2(ITERATIVE_DECOMP):0]			       layer_stride_i,
     input logic [$clog2(N_O):0]				       layer_no_i,
     input logic [0:NUMWRITEBANKS-1][(EFFECTIVETRITSPERWORD)-1:0][1:0] pipeline_outputs_i,
     input logic						       valid_i,

     output logic [0:NUMBANKS-1][PHYSICALBITSPERWORD-1:0]	       wdata_o,
     output logic [0:NUMBANKS-1]				       write_enable_o,
     output logic [0:NUMBANKS-1][ACTMEMBANKADDRESSDEPTH-1:0]	       write_addr_o
    );

   logic                                                     init_q;
   logic [ACTMEMBANKADDRESSDEPTH-1:0]			     addresscounter_q, addresscounter_d;

   logic [$clog2(NUMBANKS)-1:0]				     bankcounter_q, bankcounter_d;
   logic [$clog2(N_O):0]				     layer_no_q;
   logic [$clog2((N_O/ITERATIVE_DECOMP)/(N_I/WEIGHT_STAGGER)):0] numwrites;

   logic [$clog2(ITERATIVE_DECOMP):0]							       stride_q;

   assign numwrites = ITERATIVE_DECOMP <= WEIGHT_STAGGER ? (layer_no_q/ITERATIVE_DECOMP)/(N_I/WEIGHT_STAGGER) : 1;


   ///////////////////////////////// ENCODER BANK SIGNALS /////////////////////////////////

   logic [0:NUMWRITEBANKS-1][PHYSICALTRITSPERWORD-1:0][1:0]			 _encoder_inputs;
   logic [0:NUMWRITEBANKS-1][NUMENCODERSPERBANK-1:0][9:0]			 encoder_inputs;

   logic [0:NUMWRITEBANKS-1][NUMENCODERSPERBANK-1:0][7:0]			 encoder_outputs;
   logic [0:NUMWRITEBANKS-1][PHYSICALBITSPERWORD-1:0]				 actmem_write_input;

   ///////////////////////////////// END ENCODER BANK SIGNALS /////////////////////////////////

   ///////////////////////////////// ENCODER BANK /////////////////////////////////

   always_comb begin
      encoder_inputs = '0;
      _encoder_inputs = '0;
      for (int i=0;i<NUMWRITEBANKS;i++) begin
	 for (int j=0;j<EFFECTIVETRITSPERWORD;j++) begin
	    _encoder_inputs[i][j] = pipeline_outputs_i[i][j];
	 end
	 encoder_inputs[i] = _encoder_inputs[i];
      end
   end
   genvar m;
   genvar n;
   generate
      for (m=0;m<NUMWRITEBANKS;m++) begin: encoderbank
	 for (n=0;n<NUMENCODERSPERBANK;n++) begin : encodermodule
	    encoder enc (
			 .encoder_i(encoder_inputs[m][n]),
			 .encoder_o(encoder_outputs[m][n])
			 );
	 end
	 assign actmem_write_input[m] = {>>{encoder_outputs[m]}};
      end
   endgenerate

   ///////////////////////////////// END ENCODER BANK /////////////////////////////////


   always_comb begin
      wdata_o = '0;
      write_enable_o = '0;
      write_addr_o = '0;

      addresscounter_d = addresscounter_q;
      bankcounter_d = bankcounter_q;

      if(init_q == 1 && valid_i == 1'b1) begin

	 bankcounter_d = (bankcounter_q + (numwrites*stride_q))%NUMBANKS;
	 if(bankcounter_d < bankcounter_q) begin
	    addresscounter_d = addresscounter_q + 1;
	 end
	 for(int i=0;i<2*NUMBANKS;i++) begin
	    if(i>=bankcounter_q && i<(bankcounter_q+numwrites)) begin

	       if(i>NUMBANKS) begin
		  write_addr_o[((i - bankcounter_q)*stride_q + bankcounter_q)%NUMBANKS] = addresscounter_q + 1;
	       end else begin
		  write_addr_o[((i - bankcounter_q)*stride_q + bankcounter_q)%NUMBANKS] = addresscounter_q;
	       end
	       wdata_o[((i - bankcounter_q)*stride_q + bankcounter_q)%NUMBANKS] = actmem_write_input[i-bankcounter_q];
	       write_enable_o[((i - bankcounter_q)*stride_q + bankcounter_q)%NUMBANKS] = 1;
	    end
	 end

      end

   end

   always_ff @(posedge clk_i, negedge rst_ni) begin
      if(~rst_ni) begin
	 init_q <= 0;
	 stride_q <= 1;
	 layer_no_q <= N_O;
	 addresscounter_q <= '0;
	 bankcounter_q <= '0;
      end else begin
	 if(latch_new_layer_i == 1) begin
	    init_q <= 1;
	    stride_q <= layer_stride_i;
	    layer_no_q <= layer_no_i;
	    addresscounter_q <= '0;
	    bankcounter_q <= layer_offset_i;
	 end else begin
	    addresscounter_q <= addresscounter_d;
	    bankcounter_q <= bankcounter_d;
	 end
      end
   end

endmodule
