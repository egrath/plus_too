//
// m68k_cache.v - 2-Way Set Associative Write-Through Cache for Plus Too
//
// Only active in turbo mode (16MHz). In 8MHz mode, the cache is completely
// bypassed with zero overhead - CPU connects directly to SDRAM.
//
// 8 KB total cache (4 KB per way)
// - 1024 sets × 2 ways
// - 32-bit (4 byte) line size
// - Write-through with write-no-allocate
// - Pseudo-LRU replacement
// - BRAM-optimized for Quartus (M9K blocks)
//

module m68k_cache (
    input           clk,                // System clock (clk32)
    input           reset,              // CPU reset (active low, _cpuReset)
    
    // CPU interface (word-aligned addresses only)
    input           cpu_req,            // CPU requests access
    input           cpu_rw,             // 1 = read, 0 = write
    input   [23:1]  cpu_addr,           // Word address from CPU
    input   [1:0]   cpu_ds,             // Data strobes (UDS/LDS active low)
    input   [15:0]  cpu_din,            // Data from CPU (for writes)
    output  reg [15:0] cpu_dout,        // Data to CPU (for reads)
    output          cpu_dtack,          // Data transfer acknowledge
    output          cache_active,       // Debug: high when cache is active
    
    // Memory interface
    output  reg     mem_req,            // Request to SDRAM
    output  reg     mem_rw,             // 1 = read, 0 = write
    output  reg [21:0] mem_addr,        // Word address to SDRAM
    output  reg [1:0]  mem_ds,          // Data strobes
    output  reg [15:0] mem_dout,        // Data to SDRAM (writes)
    input   [15:0]  mem_din,            // Data from SDRAM (reads)
    input           mem_dtack,          // SDRAM acknowledge
    
    // Cache control
    input           cache_invalidate,   // Invalidate entire cache (floppy/ROM load)
    input           cpu_turbo           // Turbo mode: 1 = 16MHz (cache on), 0 = 8MHz (cache off)
);

    // ============================================================
    // Parameters
    // ============================================================
    localparam NUM_SETS     = 1024;         // 1024 sets
    localparam WAYS         = 2;            // 2-way set associative
    localparam LINE_SIZE    = 32;           // 32 bits per line (2 words)
    localparam TAG_WIDTH    = 11;           // Tag width: 23 - 10(set) - 1(word) - 1(byte) = 11
    localparam SET_WIDTH    = 10;           // 10 bits for 1024 sets
    
    // ============================================================
    // State Machine
    // ============================================================
    localparam IDLE         = 2'd0;
    localparam CACHE_READ   = 2'd1;
    localparam MEM_READ     = 2'd2;
    localparam MEM_WRITE    = 2'd3;
    
    reg [1:0] state, state_next;
    
    // ============================================================
    // Address Decoding
    // ============================================================
    wire [TAG_WIDTH-1:0] cpu_tag   = cpu_addr[23:13];      // Upper 11 bits
    wire [SET_WIDTH-1:0] cpu_set   = cpu_addr[12:3];       // Middle 10 bits
    wire                 cpu_word  = cpu_addr[2];           // Which word in line (0 or 1)
    
    // ============================================================
    // Cache Storage - Explicit BRAM Inference for Quartus
    // ============================================================

    // Way 0 - Data memory (1024 × 32 bits)
    (* ramstyle = "M9K" *) reg [31:0] cache_data0_ram [0:NUM_SETS-1];
    reg [31:0] cache_data0_read;
    reg cache_data0_we;
    reg [31:0] cache_data0_write;

    // Way 0 - Tag + Valid memory (1024 × 12 bits: 11-bit tag + 1-bit valid)
    (* ramstyle = "M9K" *) reg [11:0] cache_tag0_ram [0:NUM_SETS-1];
    reg [11:0] cache_tag0_read;
    reg cache_tag0_we;
    reg [11:0] cache_tag0_write;

    // Way 1 - Data memory (1024 × 32 bits)
    (* ramstyle = "M9K" *) reg [31:0] cache_data1_ram [0:NUM_SETS-1];
    reg [31:0] cache_data1_read;
    reg cache_data1_we;
    reg [31:0] cache_data1_write;

    // Way 1 - Tag + Valid memory (1024 × 12 bits: 11-bit tag + 1-bit valid)
    (* ramstyle = "M9K" *) reg [11:0] cache_tag1_ram [0:NUM_SETS-1];
    reg [11:0] cache_tag1_read;
    reg cache_tag1_we;
    reg [11:0] cache_tag1_write;

    // LRU array (1024 × 1 bit, distributed logic)
    reg cache_lru_ram [0:NUM_SETS-1];
    reg cache_lru_read;
    reg cache_lru_we;
    reg cache_lru_write;

    // ============================================================
    // Invalidation State Machine
    // ============================================================
    reg [SET_WIDTH-1:0] invalidate_counter;
    reg invalidate_active;

    // ============================================================
    // Synchronous Reads (BRAM requirement: read on clock edge)
    // ============================================================
    always @(posedge clk) begin
        cache_data0_read <= cache_data0_ram[cpu_set];
        cache_data1_read <= cache_data1_ram[cpu_set];
        cache_tag0_read  <= cache_tag0_ram[cpu_set];
        cache_tag1_read  <= cache_tag1_ram[cpu_set];
        cache_lru_read   <= cache_lru_ram[cpu_set];
    end

    // Extract valid bits from tag reads (bit 11)
    wire valid0 = cache_tag0_read[11];
    wire valid1 = cache_tag1_read[11];
    
    // Pure tag values (without valid bit)
    wire [TAG_WIDTH-1:0] tag0 = cache_tag0_read[TAG_WIDTH-1:0];
    wire [TAG_WIDTH-1:0] tag1 = cache_tag1_read[TAG_WIDTH-1:0];

    // ============================================================
    // Cache Lookup
    // ============================================================
    wire hit0 = valid0 && (tag0 == cpu_tag);
    wire hit1 = valid1 && (tag1 == cpu_tag);
    wire hit  = hit0 || hit1;
    wire hit_way = hit1;  // 0 = way 0, 1 = way 1

    // Replacement (replace LRU way)
    wire replace_way = cache_lru_read;

    // ============================================================
    // Cache Output (word select based on cpu_word)
    // ============================================================
    wire [15:0] hit_data = cpu_word ? 
        (hit_way ? cache_data1_read[31:16] : cache_data0_read[31:16]) :
        (hit_way ? cache_data1_read[15:0]  : cache_data0_read[15:0]);

    // ============================================================
    // Turbo Mode Gating
    // ============================================================
    wire cache_enabled = cpu_turbo;
    assign cache_active = cache_enabled;

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
                if (cpu_req && cache_enabled && !invalidate_active) begin
                    if (cpu_rw) begin
                        // CPU Read
                        if (hit)
                            state_next = CACHE_READ;  // Cache hit
                        else
                            state_next = MEM_READ;    // Cache miss
                    end else begin
                        // CPU Write (write-through, no allocate)
                        state_next = MEM_WRITE;
                    end
                end else if (cpu_req && (!cache_enabled || invalidate_active)) begin
                    // Cache disabled or invalidating - direct passthrough
                    if (cpu_rw)
                        state_next = MEM_READ;
                    else
                        state_next = MEM_WRITE;
                end
            end
            
            CACHE_READ: begin
                // Single cycle cache read
                state_next = IDLE;
            end
            
            MEM_READ: begin
                if (mem_dtack)
                    state_next = IDLE;
            end
            
            MEM_WRITE: begin
                if (mem_dtack)
                    state_next = IDLE;
            end
        endcase
    end

    // ============================================================
    // DTACK Generation
    // ============================================================
    assign cpu_dtack = (state == CACHE_READ) || 
                       ((state == MEM_READ || state == MEM_WRITE) && mem_dtack);

    // ============================================================
    // Cache Control Logic
    // ============================================================
    always @(posedge clk or negedge reset) begin
        if (!reset) begin
            cache_data0_we <= 1'b0;
            cache_data1_we <= 1'b0;
            cache_tag0_we <= 1'b0;
            cache_tag1_we <= 1'b0;
            cache_lru_we <= 1'b0;
        end else begin
            // Default: no writes
            cache_data0_we <= 1'b0;
            cache_data1_we <= 1'b0;
            cache_tag0_we <= 1'b0;
            cache_tag1_we <= 1'b0;
            cache_lru_we <= 1'b0;
            
            if (cache_enabled && !invalidate_active) begin
                case (state)
                    CACHE_READ: begin
                        if (hit) begin
                            // Update LRU
                            cache_lru_we <= 1'b1;
                            cache_lru_write <= ~hit_way;
                        end
                    end
                    
                    MEM_READ: begin
                        if (mem_dtack) begin
                            // Fill cache line on read miss
                            if (replace_way) begin
                                cache_data1_we <= 1'b1;
                                cache_data1_write <= cpu_word ? 
                                    {mem_din, cache_data1_read[15:0]} :
                                    {cache_data1_read[31:16], mem_din};
                                cache_tag1_we <= 1'b1;
                                cache_tag1_write <= {1'b1, cpu_tag};
                            end else begin
                                cache_data0_we <= 1'b1;
                                cache_data0_write <= cpu_word ? 
                                    {mem_din, cache_data0_read[15:0]} :
                                    {cache_data0_read[31:16], mem_din};
                                cache_tag0_we <= 1'b1;
                                cache_tag0_write <= {1'b1, cpu_tag};
                            end
                            cache_lru_we <= 1'b1;
                            cache_lru_write <= ~replace_way;
                        end
                    end
                    
                    MEM_WRITE: begin
                        if (mem_dtack) begin
                            // Write-through: update cache on hit
                            if (hit0) begin
                                cache_data0_we <= 1'b1;
                                cache_data0_write <= cpu_word ?
                                    {cpu_din, cache_data0_read[15:0]} :
                                    {cache_data0_read[31:16], cpu_din};
                                cache_lru_we <= 1'b1;
                                cache_lru_write <= 1'b1;
                            end
                            if (hit1) begin
                                cache_data1_we <= 1'b1;
                                cache_data1_write <= cpu_word ?
                                    {cpu_din, cache_data1_read[15:0]} :
                                    {cache_data1_read[31:16], cpu_din};
                                cache_lru_we <= 1'b1;
                                cache_lru_write <= 1'b0;
                            end
                        end
                    end
                endcase
            end
        end
    end

    // ============================================================
    // Cache Invalidation Logic
    // ============================================================
    always @(posedge clk or negedge reset) begin
        if (!reset) begin
            invalidate_counter <= 0;
            invalidate_active <= 1'b1;  // Start invalidation on reset
        end else if (cache_invalidate && !invalidate_active) begin
            invalidate_counter <= 0;
            invalidate_active <= 1'b1;
        end else if (invalidate_active) begin
            if (invalidate_counter == NUM_SETS-1)
                invalidate_active <= 1'b0;
            else
                invalidate_counter <= invalidate_counter + 1'b1;
        end
    end
	 
	// ============================================================
	// Combined Write Block (normal + invalidation)
	// ============================================================
	always @(posedge clk) begin
		 if (invalidate_active) begin
			  // Invalidation: clear valid bits and LRU cycle by cycle
			  cache_tag0_ram[invalidate_counter][11] <= 1'b0;
			  cache_tag1_ram[invalidate_counter][11] <= 1'b0;
			  cache_lru_ram[invalidate_counter] <= 1'b0;
		 end else begin
			  // Normal cache writes
			  if (cache_data0_we)
					cache_data0_ram[cpu_set] <= cache_data0_write;
			  if (cache_data1_we)
					cache_data1_ram[cpu_set] <= cache_data1_write;
			  if (cache_tag0_we)
					cache_tag0_ram[cpu_set] <= cache_tag0_write;
			  if (cache_tag1_we)
					cache_tag1_ram[cpu_set] <= cache_tag1_write;
			  if (cache_lru_we)
					cache_lru_ram[cpu_set] <= cache_lru_write;
		 end
	end	 

    // ============================================================
    // Memory Interface Control
    // ============================================================
    always @(posedge clk or negedge reset) begin
        if (!reset) begin
            mem_req <= 0;
            mem_rw <= 0;
            mem_addr <= 0;
            mem_ds <= 2'b00;
            mem_dout <= 0;
        end else begin
            if (state == IDLE && cpu_req) begin
                if (cpu_rw) begin
                    // Read request (cache miss or cache disabled)
                    if (!cache_enabled || !hit || invalidate_active) begin
                        mem_req <= 1;
                        mem_rw <= 1;  // Read
                        mem_addr <= cpu_addr[23:2];
                        mem_ds <= 2'b00;
                    end
                end else begin
                    // Write request (always goes to memory)
                    mem_req <= 1;
                    mem_rw <= 0;  // Write
                    mem_addr <= cpu_addr[23:2];
                    mem_ds <= cpu_ds;
                    mem_dout <= cpu_din;
                end
            end else if ((state == MEM_READ || state == MEM_WRITE) && mem_dtack) begin
                // Request complete
                mem_req <= 0;
            end
        end
    end

    // ============================================================
    // CPU Data Output
    // ============================================================
    always @(posedge clk or negedge reset) begin
        if (!reset) begin
            cpu_dout <= 0;
        end else begin
            case (state)
                CACHE_READ: begin
                    cpu_dout <= hit_data;
                end
                
                MEM_READ: begin
                    if (mem_dtack)
                        cpu_dout <= mem_din;
                end
                
                default: begin
                    cpu_dout <= mem_din;
                end
            endcase
        end
    end

endmodule