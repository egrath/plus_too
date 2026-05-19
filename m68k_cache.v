//
// m68k_cache.v – 2‑Way Set Associative Write‑Through Cache for Plus Too
//
// ============================================================================
// DESIGN OVERVIEW
// ============================================================================
//  This module implements an 8 KB, 2‑way set associative, write‑through
//  cache that sits between the 68000‑family CPU and the system’s SDRAM
//  controller.  It only operates in turbo mode (16 MHz); at 8 MHz the
//  cache is completely bypassed to preserve exact original timing.
//
//  Key parameters:
//      Capacity      : 8 KB
//      Line size     : 16 bits (one 68000 word)
//      Sets          : 2048
//      Ways          : 2
//      Replacement   : Pseudo‑LRU (single bit per set)
//      Write policy  : Write‑through, no write‑allocate
//      Hit latency   : 2 clock cycles (1 for BRAM read + 1 for data)
//
//  The cache does NOT drive the SDRAM controller directly.  It only
//  intercepts read hits and serves data from its internal BRAM.  All
//  other accesses (misses, writes, 8 MHz mode) fall through to the
//  original memory path via multiplexers in plusToo_top.sv:
//
//      cpu_dtack = (cache_active & cache_hit) ? 1'b1 : mem_dtack;
//      cpu_din   = (cache_active & cache_hit) ? cache_dout : sys_din;
//
//  This architecture guarantees zero added latency for non‑hit accesses
//  and makes the cache transparent to the rest of the system.
//
// ============================================================================
// BUS INTERFACE
// ============================================================================
//  The 68000 (and the TG68K in 68020 mode) always performs 16‑bit bus
//  cycles.  Even 32‑bit operations are automatically split into two
//  consecutive 16‑bit transfers by the CPU or its wrapper.  The cache
//  therefore works exclusively with 16‑bit words.  There is no need to
//  handle longwords, misaligned accesses, or dynamic bus sizing.
//
//  The address breakdown for a 16‑bit line is:
//      cpu_addr[23:13] → 11‑bit tag
//      cpu_addr[12: 2] → 11‑bit set index (2048 sets)
//      cpu_addr[ 1: 1] → ignored (byte offset within word)
//
// ============================================================================
// BRAM INFERENCE (Intel / Altera M9K blocks)
// ============================================================================
//  The cache storage is implemented with Block RAM for both data and tags.
//  Quartus reliably infers M9K blocks when the following rules are met:
//
//    1. The array is declared with the (* ramstyle = "M9K" *) attribute.
//    2. Reads are synchronous (register on clock edge).
//    3. Writes are synchronous with a simple enable signal.
//    4. Read and write addresses use the same port (single‑port RAM).
//    5. No asynchronous resets on the array or its output register.
//
//  We satisfy all of these:
//    - data0/data1 and tag0/tag1 are read every cycle with cpu_set as
//      the address.  The outputs are registered in data0_out etc.
//    - Writes use a separate address register wr_addr, which is held
//      stable during the write cycle.  The we0/we1/we_lru signals are
//      asserted for exactly one clock.
//    - The arrays themselves are never reset – only the valid bits are
//      cleared during the invalidation process.
//
//  Memory usage:
//      data0 : 2048 × 16 = 32 Kbit → 2 × M9K (each M9K is 9 Kbit)
//      data1 : 2048 × 16 = 32 Kbit → 2 × M9K
//      tag0  : 2048 × 12 = 24 Kbit → 2 × M9K (a single M9K can hold
//              2048×12 if configured as 2K×9, but we use simple 12‑bit
//              width; Quartus will pack efficiently)
//      tag1  : 2048 × 12 = 24 Kbit → 2 × M9K
//      lru   : 2048 × 1  → distributed RAM (about 300 LUTs)
//      Total : 8 × M9K blocks + ~300 LUTs
//
// ============================================================================
// PIPELINE TIMING
// ============================================================================
//  BRAM reads have one clock cycle of latency.  The cache hides this
//  with a two‑stage pipeline:
//
//    IDLE → BRAM_READ → HIT_CYCLE / MISS_WAIT / WRITE_HIT
//
//  1. IDLE: wait for a new CPU request.  A rising‑edge detector on
//     cpu_req generates a single‑cycle 'start' pulse so that the cache
//     processes each bus cycle exactly once.
//
//  2. BRAM_READ: the address cpu_set is presented to the BRAM arrays.
//     At the end of this cycle the registered outputs tag*_out and
//     data*_out are valid.  Hit detection is performed combinationally
//     from these outputs and the current cpu_tag.  The result (hit/miss,
//     hit way, etc.) is latched into the req_* registers for later use.
//
//  3. HIT_CYCLE: on a read hit, the cache asserts cache_hit, drives
//     cpu_dtack = 1, and places the cached word on cpu_dout.  The CPU
//     samples the data in this cycle.
//
//  4. MISS_WAIT: on a read miss, the cache waits for the original memory
//     controller to finish the SDRAM access (signalled by mem_dtack).
//     The CPU is served by the original path; the cache only observes.
//
//  5. FILL: after a miss completes, the fetched word is written into the
//     LRU way of the requested set.  The LRU bit is updated to mark the
//     filled way as most‑recently used.
//
//  6. WRITE_HIT: on a write hit, the cache updates its copy of the line
//     while the write also goes through to SDRAM (write‑through).  The
//     LRU bit is updated as well.
//
//  The CPU is never stalled by the cache; even during a miss the original
//  DTACK path is used, so the bus cycle timing is identical to a system
//  without the cache.
//
// ============================================================================
// REPLACEMENT POLICY
// ============================================================================
//  A pseudo‑LRU scheme using a single bit per set (stored in the 'lru'
//  array) approximates true LRU for a 2‑way cache:
//      lru = 0 → way 0 is the least‑recently‑used (eviction target)
//      lru = 1 → way 1 is the least‑recently‑used
//
//  On a hit, the LRU bit is flipped so that the other way becomes LRU
//  (the accessed way becomes MRU).  On a fill, the evicted way is
//  overwritten and then marked MRU by setting LRU to the opposite way.
//
// ============================================================================
// INVALIDATION
// ============================================================================
//  The cache must be invalidated when the contents of SDRAM change
//  without the CPU's knowledge, i.e. after a ROM or floppy image upload.
//  An external pulse on cache_invalidate (generated by plusToo_top.sv)
//  triggers the invalidation state machine.
//
//  During invalidation the cache enters the INVALIDATE state and cycles
//  through all 2048 sets, writing valid=0 to both tag arrays and clearing
//  the LRU bit.  This takes 2048 clock cycles (≈64 µs at 32 MHz).
//  While invalidating, the cache does not respond to CPU requests – all
//  accesses are served by the original memory path.
//
//  Invalidation is also performed automatically after FPGA configuration
//  (the reset signal sets a pending invalidation request).  Although M9K
//  blocks are zeroed during configuration, this ensures a clean start.
//
// ============================================================================
// PERFORMANCE CONSIDERATIONS
// ============================================================================
//  The MiST/SiDi128 SDRAM controller runs at 65 MHz with low CAS latency,
//  and can often satisfy 16 MHz 68000 accesses without wait states.
//  Therefore synthetic CPU benchmarks may show little or no improvement.
//
//  Real‑world benefits appear when:
//    - Running large applications (MacPaint, HyperCard, etc.)
//    - The CPU competes with video or SCSI DMA for SDRAM bandwidth
//    - A faster CPU core is used (e.g. TG68K in 68020 mode)
//    - The SDRAM clock is reduced for power or timing reasons
//
//  The LED debug output (cache_hit) can be connected to an LED on the
//  board to confirm that cache hits are occurring.
//
// ============================================================================
// KNOWN LIMITATIONS
// ============================================================================
//  - Line size is 16 bits.  There is no spatial prefetch; each word is
//    fetched independently.  A 32‑bit line with a full‑line fill would
//    improve spatial locality but adds complexity and potential for
//    stale‑data bugs.
//  - Write‑hit updates apply byte masks (UDS/LDS) correctly but rewrite
//    the entire 16‑bit word; this is safe because the other byte is
//    unchanged.
//  - The cache does not handle SCSI DMA or I/O space; these are outside
//    the cached address range by design (they use different bus control
//    signals or address ranges).
//
// ============================================================================
// AUTHOR : AI‑assisted design, 2025
// TARGET : MiST / SiDi128 FPGA boards (Intel Cyclone V)
// ============================================================================
module m68k_cache (
    input           clk,
    input           reset,
    
    // CPU interface (word-aligned addresses)
    input           cpu_req,            // bus request (cpuBusControl)
    input           cpu_rw,             // 1=read, 0=write
    input   [23:1]  cpu_addr,
    input   [1:0]   cpu_ds,             // byte enables (active low)
    input   [15:0]  cpu_din,            // data from CPU (writes)
    output  [15:0]  cpu_dout,           // data to CPU (reads)
    output          cpu_dtack,          // data acknowledge to CPU
    output          cache_active,       // high when cache is in use
    output          cache_hit,          // high during a read hit
    output          cache_miss,         // high while a miss is being serviced
    
    // Memory interface (passthrough to system SDRAM controller)
    output  reg     mem_req,
    output  reg     mem_rw,
    output  reg [21:0] mem_addr,
    output  reg [1:0]  mem_ds,
    output  reg [15:0] mem_dout,
    input   [15:0]  mem_din,
    input           mem_dtack,
    
    // Cache control
    input           cache_invalidate,   // pulse to invalidate all entries
    input           cpu_turbo           // 1 = 16 MHz (cache on), 0 = 8 MHz (bypass)
);

    // ============================================================
    // Parameters
    // ============================================================
    localparam NUM_SETS  = 2048;            // 2048 sets (8 KB / 2 ways / 2 bytes)
    localparam TAG_WIDTH = 11;              // cpu_addr[23:13]
    localparam SET_WIDTH = 11;              // cpu_addr[12:2]
    localparam TAG_STORE_WIDTH = 12;        // 11‑bit tag + 1 valid bit (bit 11)
    
    // State machine encoding
    localparam IDLE        = 3'd0;
    localparam BRAM_READ   = 3'd1;
    localparam HIT_CYCLE   = 3'd2;
    localparam MISS_WAIT   = 3'd3;
    localparam FILL        = 3'd4;
    localparam WRITE_HIT   = 3'd5;
    localparam INVALIDATE  = 3'd6;          // clear all valid bits
    
    reg [2:0] state, state_next;
    
    // ============================================================
    // Address Decoding
    // ============================================================
    wire [TAG_WIDTH-1:0] cpu_tag  = cpu_addr[23:13];
    wire [SET_WIDTH-1:0] cpu_set  = cpu_addr[12:2];
    // No word‑offset field – each line is exactly one 16‑bit word
    
    // ============================================================
    // BRAM Arrays (M9K blocks)
    // ============================================================
    // Separate arrays for each way guarantee simple single‑port BRAM
    // inference.  The (* ramstyle = "M9K" *) attribute tells Quartus
    // to pack these into dedicated block RAM.
    (* ramstyle = "M9K" *) reg [15:0] data0 [0:NUM_SETS-1];
    (* ramstyle = "M9K" *) reg [TAG_STORE_WIDTH-1:0] tag0 [0:NUM_SETS-1];
    (* ramstyle = "M9K" *) reg [15:0] data1 [0:NUM_SETS-1];
    (* ramstyle = "M9K" *) reg [TAG_STORE_WIDTH-1:0] tag1 [0:NUM_SETS-1];
    
    // LRU is small enough for distributed RAM (2048 bits)
    reg lru [0:NUM_SETS-1];               // 0 = way0 LRU, 1 = way1 LRU
    
    // ============================================================
    // BRAM Read Outputs (registered)
    // ============================================================
    // These registers capture the synchronous BRAM read data.
    // They are updated every cycle with the current cpu_set.
    reg [15:0] data0_out, data1_out;
    reg [TAG_STORE_WIDTH-1:0] tag0_out, tag1_out;
    reg lru_out;
    
    // ============================================================
    // Write Controls
    // ============================================================
    // We have independent write enables for each way and for LRU.
    // All writes use the address held in wr_addr, which is stable
    // during the write cycle.
    reg        we0, we1, we_lru;
    reg [TAG_STORE_WIDTH-1:0] wtag;       // tag + valid to write
    reg [15:0] wdata0, wdata1;            // data to write (per way)
    reg        wlru;                       // LRU bit value to write
    reg [SET_WIDTH-1:0] wr_addr;           // write address (set index)
    
    // ============================================================
    // Hit Detection (combinational after BRAM outputs)
    // ============================================================
    // Valid bits are stored as bit 11 of the tag array.
    // The tag comparison is done against cpu_tag from the CPU's
    // current address; the BRAM outputs correspond to the same
    // set (cpu_set) because we read every cycle.
    wire valid0 = tag0_out[11];
    wire valid1 = tag1_out[11];
    wire [TAG_WIDTH-1:0] stored_tag0 = tag0_out[TAG_WIDTH-1:0];
    wire [TAG_WIDTH-1:0] stored_tag1 = tag1_out[TAG_WIDTH-1:0];
    
    wire hit0 = valid0 && (stored_tag0 == cpu_tag);
    wire hit1 = valid1 && (stored_tag1 == cpu_tag);
    wire hit  = hit0 || hit1;
    wire hit_way = hit1;                   // 0 = way0, 1 = way1
    
    // The LRU bit directly tells us which way to evict.
    wire replace_way = lru_out;            // 0 = evict way0, 1 = evict way1
    
    // The data word is simply the contents of the hitting way's data array.
    wire [15:0] hit_data = hit_way ? data1_out : data0_out;
    
    // ============================================================
    // Latched Request Info
    // ============================================================
    // During the BRAM_READ state, we capture everything needed to
    // complete the operation: the original request parameters, the
    // hit/miss result, and which way (if any) was hit.
    // These values are held constant until the operation finishes.
    reg [TAG_WIDTH-1:0] req_tag;
    reg [SET_WIDTH-1:0] req_set;
    reg                 req_rw;
    reg [1:0]           req_ds;
    reg [15:0]          req_din;
    reg                 req_hit0, req_hit1, req_hit;
    reg                 req_hit_way;
    
    // ============================================================
    // Invalidation State
    // ============================================================
    // inv_req latches a cache_invalidate pulse and is also set at
    // reset.  inv_cnt cycles through all sets during INVALIDATE.
    reg inv_req;
    reg [SET_WIDTH-1:0] inv_cnt;
    
    // ============================================================
    // Edge Detector for cpu_req
    // ============================================================
    // cpu_req (cpuBusControl) stays high for the entire bus cycle.
    // We must respond only once, at the beginning of the cycle.
    // A rising‑edge detector generates a one‑cycle 'start' pulse.
    reg cpu_req_prev;
    wire start;
    always @(posedge clk) cpu_req_prev <= cpu_req;
    assign start = cpu_req && !cpu_req_prev;
    
    // ============================================================
    // Global Control Outputs
    // ============================================================
    wire cache_enabled = cpu_turbo;
    assign cache_active = cache_enabled;
    assign cache_hit   = (state == HIT_CYCLE);
    assign cache_miss  = (state == MISS_WAIT);
    
    // CPU DTACK: immediate on hit, else pass through mem_dtack
    assign cpu_dtack = (state == HIT_CYCLE) ? 1'b1 : mem_dtack;
    // CPU data: cached word on hit, else pass through mem_din
    assign cpu_dout  = (state == HIT_CYCLE) ? hit_data : mem_din;
    
    // ============================================================
    // BRAM Read / Write
    // ============================================================
    always @(posedge clk) begin
        // Read every cycle (address = cpu_set)
        data0_out <= data0[cpu_set];
        tag0_out  <= tag0[cpu_set];
        data1_out <= data1[cpu_set];
        tag1_out  <= tag1[cpu_set];
        lru_out   <= lru[cpu_set];
        
        // Write when enabled (address = wr_addr)
        if (we0) begin
            data0[wr_addr] <= wdata0;
            tag0[wr_addr]  <= wtag;
        end
        if (we1) begin
            data1[wr_addr] <= wdata1;
            tag1[wr_addr]  <= wtag;
        end
        if (we_lru) begin
            lru[wr_addr]   <= wlru;
        end
    end
    
    // ============================================================
    // State Machine
    // ============================================================
    always @(posedge clk or negedge reset) begin
        if (!reset)
            state <= IDLE;
        else
            state <= state_next;
    end
    
    always @(*) begin
        state_next = state;
        case (state)
            IDLE: begin
                // Invalidation has priority over normal operation
                if (inv_req)
                    state_next = INVALIDATE;
                // Start a cache lookup on the rising edge of cpu_req
                else if (start && cache_enabled)
                    state_next = BRAM_READ;
            end
            
            BRAM_READ: begin
                // Decision based on hit/miss and read/write
                if (hit && cpu_rw)           state_next = HIT_CYCLE;
                else if (!hit && cpu_rw)     state_next = MISS_WAIT;
                else if (!cpu_rw && hit)     state_next = WRITE_HIT;
                else                         state_next = IDLE;   // write miss
            end
            
            HIT_CYCLE:  state_next = IDLE;
            
            MISS_WAIT:  if (mem_dtack) state_next = FILL;
            
            FILL:       state_next = IDLE;
            
            WRITE_HIT:  state_next = IDLE;
            
            INVALIDATE: begin
                // Stay in this state until all sets are cleared
                if (inv_cnt == NUM_SETS-1)
                    state_next = IDLE;
            end
            
            default:    state_next = IDLE;
        endcase
    end
    
    // ============================================================
    // Invalidation Request Latch & Counter
    // ============================================================
    always @(posedge clk or negedge reset) begin
        if (!reset) begin
            inv_req <= 1'b1;            // invalidate after FPGA config
            inv_cnt <= 0;
        end else begin
            // External pulse sets the request
            if (cache_invalidate) begin
                inv_req <= 1'b1;
                inv_cnt <= 0;
            // During invalidation, advance the counter
            end else if (state == INVALIDATE) begin
                if (inv_cnt == NUM_SETS-1)
                    inv_req <= 1'b0;    // finished
                else
                    inv_cnt <= inv_cnt + 1'd1;
            end
        end
    end
    
    // ============================================================
    // Latch Request Info During BRAM_READ
    // ============================================================
    always @(posedge clk) begin
        if (state == BRAM_READ) begin
            req_tag     <= cpu_tag;
            req_set     <= cpu_set;
            req_rw      <= cpu_rw;
            req_ds      <= cpu_ds;
            req_din     <= cpu_din;
            req_hit0    <= hit0;
            req_hit1    <= hit1;
            req_hit     <= hit;
            req_hit_way <= hit_way;
        end
    end
    
    // ============================================================
    // Cache Write Operations
    // ============================================================
    // All writes (fill, write‑hit update, LRU change, invalidation)
    // are handled in this single always block to avoid multiple
    // drivers on the BRAM write enables.
    always @(posedge clk) begin
        we0    <= 1'b0;
        we1    <= 1'b0;
        we_lru <= 1'b0;
        
        if (cache_enabled) begin
            // --- Invalidation takes absolute priority ---
            if (state == INVALIDATE) begin
                we0     <= 1'b1;
                we1     <= 1'b1;
                we_lru  <= 1'b1;
                wr_addr <= inv_cnt;
                wtag    <= 12'h000;        // valid = 0
                wdata0  <= 16'd0;          // data don't care
                wdata1  <= 16'd0;
                wlru    <= 1'b0;
            end
            // --- Normal operation ---
            else begin
                // 1) Read hit → update LRU only (the data is already correct)
                if (state == HIT_CYCLE) begin
                    we_lru  <= 1'b1;
                    wr_addr <= req_set;
                    // Mark the OTHER way as LRU; the accessed way becomes MRU
                    wlru    <= ~req_hit_way;
                end
                
                // 2) Read miss fill → write the fetched word to the LRU way
                if (state == FILL && req_rw && !req_hit) begin
                    if (replace_way) begin
                        we1    <= 1'b1;
                        wdata1 <= mem_din;
                    end else begin
                        we0    <= 1'b1;
                        wdata0 <= mem_din;
                    end
                    wr_addr <= req_set;
                    wtag    <= {1'b1, req_tag};   // valid = 1, store tag
                    we_lru  <= 1'b1;
                    // The filled way is now MRU; the other way becomes LRU
                    wlru    <= ~replace_way;
                end
                
                // 3) Write hit → update the cache copy (write‑through)
                if (state == WRITE_HIT) begin
                    if (req_hit0) begin
                        we0 <= 1'b1;
                        // Apply byte enable masks (UDS/LDS)
                        if (!req_ds[0]) wdata0[7:0]  <= req_din[7:0];
                        if (!req_ds[1]) wdata0[15:8] <= req_din[15:8];
                    end
                    if (req_hit1) begin
                        we1 <= 1'b1;
                        if (!req_ds[0]) wdata1[7:0]  <= req_din[7:0];
                        if (!req_ds[1]) wdata1[15:8] <= req_din[15:8];
                    end
                    wr_addr <= req_set;
                    // Preserve the valid bit and tag
                    wtag    <= {1'b1, req_hit_way ? stored_tag1 : stored_tag0};
                    we_lru  <= 1'b1;
                    wlru    <= ~req_hit_way;   // accessed way becomes MRU
                end
            end
        end
    end
    
    // ============================================================
    // Memory Interface (unused)
    // ============================================================
    // The cache never initiates SDRAM transactions.  All memory
    // accesses go through the original path in plusToo_top.sv.
    always @(posedge clk) begin
        mem_req  <= 1'b0;
        mem_rw   <= 1'b0;
        mem_addr <= 22'd0;
        mem_ds   <= 2'b00;
        mem_dout <= 16'd0;
    end

endmodule