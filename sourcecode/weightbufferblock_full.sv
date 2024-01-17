// ----------------------------------------------------------------------
//
// File: weightbufferblock.sv
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
// Make sure input is stable for clock high

module weightbufferblock_full
  #(
    parameter int unsigned N_I = 512,
    parameter int unsigned WEIGHT_STAGGER = 2,
    parameter int unsigned K = 3
    )
   (
     input logic [0:N_I/WEIGHT_STAGGER-1][1:0]		 data_i,
     input logic					 clk_i,
     input logic					 rst_ni,
     input logic [0:1][0:WEIGHT_STAGGER-1][0:K-1][0:K-1] save_enable_i,
     input logic [0:1][0:WEIGHT_STAGGER-1][0:K-1][0:K-1] test_enable_i,
     input logic [0:1] [0:WEIGHT_STAGGER-1]	    flush_i,
     output logic [0:1][0:K-1][0:K-1][0:N_I-1][1:0] data_o
    );


   logic [0:1][0:K-1][0:K-1][0:WEIGHT_STAGGER-1][0:N_I/WEIGHT_STAGGER-1][1:0] mem_reg_d, mem_reg_q;

   assign data_o = mem_reg_q;

   always_comb begin : save_reshuffle

      mem_reg_d = mem_reg_q;

      for(int set=0;set<2;set++) begin
	 for(int stagger=0;stagger<WEIGHT_STAGGER;stagger++) begin
	    for(int kernel1=0;kernel1<K;kernel1++) begin
	       for(int kernel2=0;kernel2<K;kernel2++) begin
		  mem_reg_d[set][kernel1][kernel2][stagger] = save_enable_i[set][stagger][kernel1][kernel2] ? data_i : '0;

	       end
	    end
	 end
      end

      for(int set=0;set<2;set++) begin
	 for(int stagger=0;stagger<WEIGHT_STAGGER;stagger++) begin
	    for(int kernel1=0;kernel1<K;kernel1++) begin
	       for(int kernel2=0;kernel2<K;kernel2++) begin
		  if (flush_i[set][stagger] == 1'b1) begin
		     mem_reg_d[set][kernel1][kernel2][stagger] = '0;
		  end
	       end
	    end
	 end
      end

   end // block: save_reshuffle

   always_ff @(posedge clk_i, negedge rst_ni) begin
      if(~rst_ni) begin
	 mem_reg_q <= '0;
      end else begin
	 mem_reg_q <= mem_reg_d;
      end
   end


endmodule
