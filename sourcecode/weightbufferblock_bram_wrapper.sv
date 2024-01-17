// ----------------------------------------------------------------------
//
// File: weightbufferblock_bram_wrapper.sv
//
// Created: 13.07.2023
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

module weightbufferblock_bram_wrapper
  #(
    parameter int unsigned N_I = 512,
    parameter int unsigned K = 3
    )
   (
    input logic [0:K-1][0:K-1][0:N_I-1][1:0]  data_i,
    input logic				      clk_i,
    input logic				      save_enable_i,
    input logic				      flush_i,
    input logic				      read_set_i,
    output logic [0:K-1][0:K-1][0:N_I-1][1:0] data_o
    );

   logic [0:K-1][0:K-1][0:N_I-1][1:0]	      flush_data, input_data;
   logic				      write_enable;


   assign flush_data = '0;
   assign input_data = flush_i ? flush_data : data_i;
   assign write_enable = flush_i ? 1'b1 : save_enable_i;

   weightbufferblock_bram
     weightbufferblock (
			.clka(clk_i),
			.ena(write_enable),
			.wea(write_enable),
			.addra(~read_set_i),
			.dina(input_data),
			.clkb(clk_i),
			.enb('1),
			.addrb(read_set_i),
			.doutb(data_o)
			);
endmodule
