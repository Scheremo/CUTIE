package tcdm_test;

   class tcdm_driver #(
                       parameter int  AW = 32 ,
                       parameter int  DW = 32 ,
                       parameter time TA = 0ns , // stimuli application time
                       parameter time TT = 0ns   // stimuli test time
                       );

      typedef logic [AW-1:0]          addr_t;
      typedef logic [DW-1:0]          data_t;
      typedef logic [DW/8-1:0]        be_t;

      virtual                         XBAR_TCDM_BUS_DV tcdm;

      function new( virtual XBAR_TCDM_BUS_DV tcdm );
         this.tcdm = tcdm;
      endfunction // new

      function void reset_master();
         tcdm.req <= '0;
         tcdm.add <= '0;
         tcdm.wen <= '1;
         tcdm.wdata <= '0;
         tcdm.be <= '0;
      endfunction // reset_master

      function void reset_slave();
         tcdm.gnt <= '0;
         tcdm.r_opc <= '0;
         tcdm.r_rdata <= '0;
         tcdm.r_valid <= '0;
      endfunction // reset_slave


      task cycle_start;
         #TT;
      endtask

      task cycle_end;
         @(posedge tcdm.clk_i);
      endtask // cycle_end

      /// Issue a write request signal
      task send_w (
                   input addr_t addr,
                   input be_t be,
                   input data_t data
                   );

         tcdm.req <= #TA 1;
         tcdm.add <= #TA addr;
         tcdm.wen <= #TA 0;
         tcdm.be <= #TA be;
         tcdm.wdata <= #TA data;
         cycle_start();
         while (tcdm.gnt != 1) begin cycle_end(); cycle_start(); end
         cycle_end();
         tcdm.req <= #TA 0;
         tcdm.add <= #TA 0;
         tcdm.wen <= #TA 1;
         tcdm.be <= #TA 0;
         tcdm.wdata <= #TA 0;
      endtask // send_w

      // Issue a read request signal
      task send_r (
                   input addr_t addr,
                   input be_t be
                   );
         tcdm.req <= #TA 1;
         tcdm.add <= #TA addr;
         tcdm.wen <= #TA 1;
         tcdm.be <= #TA be;
         tcdm.wdata <= #TA 0;
         cycle_start();
         while (tcdm.gnt != 1) begin cycle_end(); cycle_start(); end
         cycle_end();
         tcdm.req <= #TA 0;
         tcdm.add <= #TA 0;
         tcdm.wen <= #TA 1;
         tcdm.be <= #TA 0;
         tcdm.wdata <= #TA 0;
      endtask // send_r

      // Wait for the r_valid after a write request
      task recv_w (
                   output logic valid
                   );
         cycle_start();
         while (tcdm.r_valid != 1) begin cycle_end(); cycle_start(); end
         valid = tcdm.r_valid;
         cycle_end();
      endtask // recv_r

      // Wait for the data after a read request
      task recv_r (
                   output data_t data,
                   output logic valid
                   );
         cycle_start();
         while (tcdm.r_valid != 1) begin cycle_end(); cycle_start(); end
         data = tcdm.r_rdata;
         valid = tcdm.r_valid;
         cycle_end();
      endtask // recv_r

   endclass // tcdm_driver

   class tcdm_rand_master #(
                            parameter int unsigned AW = 0,
                            parameter int unsigned DW = 0,
                            // Stimuli application and test time
                            parameter time         TA = 2ns,
                            parameter time         TT = 8ns);

      typedef tcdm_test::tcdm_driver #(
                                       .AW(AW), .DW(DW), .TA(TA), .TT(TT)
                                       ) tcdm_driver_t;

      typedef logic [AW-1:0]                       addr_t;
      typedef logic [DW-1:0]                       data_t;
      typedef logic [DW/8-1:0]                     be_t;

      string                                       name;
      tcdm_driver_t   drv;

      data_t write_data_queue[$];
      addr_t addr_queue[$];
      data_t read_data_queue[$];

      function new(
                   virtual      XBAR_TCDM_BUS_DV tcdm,
                   input string name
                   );
         this.drv  = new(tcdm);
         this.name = name;
         assert(AW != 0) else $fatal(1, "Address width must be non-zero!");
         assert(DW != 0) else $fatal(1, "Data width must be non-zero!");
      endfunction // new

      function void reset();
         drv.reset_master();
      endfunction // reset

      task automatic send_ws(input addr_t addr[$],
                             input be_t be[$],
                             input data_t data[$]);
         foreach(data[i]) begin
            // $display("%0t %s> Send WRITE with DATA: %h ADDR: %h BE: %h", $time(), this.name, data[i], addr[i], be[i]);
            this.drv.send_w(addr[i], be[i], data[i]);
         end
      endtask // send_ws

      task automatic recv_ws(input int unsigned n_writes,
                             ref logic valid[$]);

         logic                            resp;
         repeat(n_writes) begin
            this.drv.recv_w(resp);
            // $display("%0t %s> Recv WRITE with RESP: %0h", $time(), this.name, resp);
            valid.push_back(resp);
         end
      endtask // recv_ws

      task automatic send_rs(input addr_t addr[$],
                             input be_t be[$]);
         foreach(addr[i]) begin
            // $display("%0t %s> Send READ with ADDR: %h BE: %b", $time(), this.name, addr[i], be[i]);
            this.drv.send_r(addr[i], be[i]);
         end
      endtask // send_rs

      task automatic recv_rs(input int unsigned n_reads,
                             ref data_t data[$],
                             ref logic valid[$]
                             );
         data_t r_data;
         logic                            r_valid;
         repeat(n_reads) begin
            this.drv.recv_r(r_data, r_valid);
            // $display("%0t %s> Recv READ with DATA: %h RESP: %0h", $time(), this.name, r_data, r_valid);
            data.push_back(r_data);
            valid.push_back(r_valid);
         end
      endtask // recv_rs

      task automatic write_stream(input addr_t addr[$],
                                  input data_t data[$],
                                  input be_t be[$]);
         automatic logic valid[$];

         fork
            send_ws(addr, be, data);
            recv_ws(addr.size(), valid);
         join

      endtask // write_stream

      task automatic read_stream(input addr_t addr[$],
                                 input  be_t be[$],
                                 output data_t data[$]);
         automatic logic valid[$];
         fork
            send_rs(addr, be);
            recv_rs(addr.size(), data, valid);
            repeat($urandom%20) this.drv.cycle_end();
         join

      endtask

      task automatic write_region(input int unsigned n_writes,
                                  input              addr_t start_addr);

         automatic logic  rand_success;
         automatic addr_t addr[$];
         automatic logic [1:0]                          data_trits[0:DW/2-1];
         automatic data_t data[$];
         automatic be_t be[$];
         automatic logic valid[$];

         for (int i = 0; i < n_writes; i++) begin
            repeat(3) begin
               foreach(data_trits[i]) begin
                  rand_success = std::randomize(data_trits[i]) with {
                     data_trits[i] inside {2'b11, 2'b01, 2'b00};
                  }; assert(rand_success);
               end
               data.push_back({>>{data_trits}});
               addr.push_back(start_addr + (i<<2));
               be.push_back('1);
            end // for (int j = 0; j < 3; j++)
            fork
               send_ws(addr, be, data);
               recv_ws(3, valid);
            join
            data.delete();
            addr.delete();
            be.delete();
         end

      endtask // rand_incr_write

      task automatic read_region (
                                  int unsigned n_reads,
                                  input addr_t start_addr
                                  );
         automatic logic  rand_success;
         automatic addr_t addr[$];
         automatic data_t data[$];
         automatic be_t be[$];
         automatic logic valid[$];

         for (int i = 0; i < n_reads; i++) begin
            repeat(3) begin
               addr.push_back(start_addr + (i<<2));
               be.push_back('1);
            end
            fork
               send_rs(addr, be);
               recv_rs(3, data, valid);
            join
            addr.delete();
            be.delete();
         end
      endtask // read_region

      task automatic rand_run (input int unsigned n_read_writes,
                               input addr_t start_addr,
                               input int unsigned addr_size);

         automatic logic  rand_success;
         automatic addr_t addr, tmp_addr_queue[$];
         automatic logic [1:0]                          data_trits[0:DW/2-1];
         automatic data_t data, tmp_data_queue[$];
         automatic be_t be, tmp_be_queue[$];
         automatic logic tmp_valid_queue[$];

         int unsigned                             addr_offsets[$];
         for (int i = 0; i < addr_size; i+=4) addr_offsets.push_back(i);
         addr_offsets.shuffle();

         repeat(n_read_writes) begin
            int unsigned addr_offset;
            addr_offset = addr_offsets.pop_front();
            addr = start_addr + addr_offset;
            repeat(3) begin
               foreach(data_trits[i]) begin
                  rand_success = std::randomize(data_trits[i]) with {
                     data_trits[i] inside {2'b11, 2'b01, 2'b00};
                  }; assert(rand_success);
               end
               data = {>>{data_trits}};

               // Temporary queues
               tmp_addr_queue.push_back(addr);
               tmp_be_queue.push_back('1);
               tmp_data_queue.push_back(data);

               // Global queues
               addr_queue.push_back(addr);
               write_data_queue.push_back(data);

               // Issue writes

            end // repeat (3)
            fork
               send_ws(tmp_addr_queue, tmp_be_queue, tmp_data_queue);
               recv_ws(3, tmp_valid_queue);
            join
            tmp_addr_queue.delete();
            tmp_be_queue.delete();
            tmp_data_queue.delete();
            tmp_valid_queue.delete();
         end // repeat (n_read_writes)

         // Wait for a moment before reading
         repeat(20) this.drv.cycle_end();

         repeat(n_read_writes) begin
            repeat(3) begin
               tmp_addr_queue.push_back(addr_queue.pop_front());
               tmp_be_queue.push_back('1);
            end

            fork
               send_rs(tmp_addr_queue, tmp_be_queue);
               recv_rs(3, tmp_data_queue, tmp_valid_queue);
            join
            read_data_queue = {read_data_queue, tmp_data_queue};
            tmp_addr_queue.delete();
            tmp_be_queue.delete();
            tmp_data_queue.delete();
            tmp_valid_queue.delete();
         end
      endtask // rand_run

      task automatic check_data();
         int unsigned n_errors = 0;
         int unsigned n_total = 0;
         foreach(write_data_queue[i]) begin
            if(write_data_queue[i] == read_data_queue[i]) begin
               $display("Correct! W_DATA: %h, R_DATA: %h", write_data_queue[i], read_data_queue[i]);
            end else begin
               $display("Mismatch! W_DATA: %h, R_DATA: %h", write_data_queue[i], read_data_queue[i]);
               n_errors++;
            end
            n_total++;
         end
         write_data_queue.delete();
         read_data_queue.delete();
         $display("Tested %d write/read ops", n_total/3);
         $display("%d errors in testbench", n_errors/3);
      endtask; // check_data

   endclass

endpackage
