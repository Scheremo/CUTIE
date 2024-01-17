`define TCDM_BUS_WIDTH 32
`define TCDM_ADDR_WIDTH 32

`define ACTMEM_START_ADDR 32'h1EC0_0000
`define ACTMEM_ADDR_SIZE 32'h0003_FFFF

`define TCN_ACTMEM_START_ADDR 32'h1EC1_0000

`define WEIGHTMEM_START_ADDR 32'h1EC4_0000
`define WEIGHTMEM_ADDR_SIZE 32'h0003_FFFF

// actmem and weightmem addresses differ in that weightmem addresses have bit 18 set
`define MEM_REGION_DIFF_BIT 18
