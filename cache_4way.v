// 4-Way Set-Associative Cache with True LRU Logic
// Parameters: 16 sets, 4 ways, 32-bit address, 32-bit data
module cache_4way #(
parameter ADDR_WIDTH = 32,
parameter DATA_WIDTH = 32,
parameter SET_BITS = 4,        // 16 sets
parameter BLOCK_OFFSET = 2,    // 4 bytes per block
parameter TAG_BITS = ADDR_WIDTH - SET_BITS - BLOCK_OFFSET
)(
input wire clk,
input wire rst_n,
input wire [ADDR_WIDTH-1:0] addr,
input wire [DATA_WIDTH-1:0] write_data,
input wire read_en,
input wire write_en,
output reg [DATA_WIDTH-1:0] read_data,
output reg hit,
output reg valid_out
);

localparam NUM_SETS = 1 << SET_BITS;  
localparam NUM_WAYS = 4;  
  
// Extract address fields  
wire [TAG_BITS-1:0] tag = addr[ADDR_WIDTH-1:ADDR_WIDTH-TAG_BITS];  
wire [SET_BITS-1:0] set_index = addr[ADDR_WIDTH-TAG_BITS-1:BLOCK_OFFSET];  
  
// Cache storage  
reg [TAG_BITS-1:0] tag_array [0:NUM_SETS-1][0:NUM_WAYS-1];  
reg [DATA_WIDTH-1:0] data_array [0:NUM_SETS-1][0:NUM_WAYS-1];  
reg valid_array [0:NUM_SETS-1][0:NUM_WAYS-1];  
  
// LRU counter - 0 = MRU, 3 = LRU  
reg [1:0] lru_counter [0:NUM_SETS-1][0:NUM_WAYS-1];  
  
// Internal signals  
reg [1:0] hit_way;  
reg [1:0] lru_way; // The victim way  
integer i, j;  
  
// Hit detection - combinational  
always @(*) begin  
    hit = 1'b0;  
    hit_way = 2'b0;  
      
    for (i = 0; i < NUM_WAYS; i = i + 1) begin  
        if (valid_array[set_index][i] && (tag_array[set_index][i] == tag)) begin  
            hit = 1'b1;  
            hit_way = i[1:0];  
        end  
    end  
end  
  
// Find victim (Invalid way OR Oldest way) - combinational  
always @(*) begin  
    lru_way = 2'b0;  
      
    // Priority 1: Check for any invalid way (empty slot)  
    // We optimize by checking loops, but for 4-way, explicit check is readable  
    if (!valid_array[set_index][0]) lru_way = 2'd0;  
    else if (!valid_array[set_index][1]) lru_way = 2'd1;  
    else if (!valid_array[set_index][2]) lru_way = 2'd2;  
    else if (!valid_array[set_index][3]) lru_way = 2'd3;  
    else begin  
        // Priority 2: If all valid, find the way with max counter (3)  
        // In a perfect LRU system, exactly one way has counter == 3.  
        for (i = 0; i < NUM_WAYS; i = i + 1) begin  
            if (lru_counter[set_index][i] == 2'b11) begin  
                lru_way = i[1:0];  
            end  
        end  
    end  
end  
  
// Sequential logic  
always @(posedge clk or negedge rst_n) begin  
    if (!rst_n) begin  
        // Reset everything  
        for (i = 0; i < NUM_SETS; i = i + 1) begin  
            for (j = 0; j < NUM_WAYS; j = j + 1) begin  
                valid_array[i][j] <= 1'b0;  
                lru_counter[i][j] <= 2'b00; // Initialize to 0  
                tag_array[i][j] <= {TAG_BITS{1'b0}};  
                data_array[i][j] <= {DATA_WIDTH{1'b0}};  
            end  
        end  
        read_data <= {DATA_WIDTH{1'b0}};  
        valid_out <= 1'b0;  
    end else begin  
        valid_out <= 1'b0;  
          
        // Read operation  
        if (read_en && !write_en) begin  
            if (hit) begin  
                read_data <= data_array[set_index][hit_way];  
                valid_out <= 1'b1;  
                  
                // --- TRUE LRU UPDATE ON HIT ---  
                // 1. Set Hit Way to MRU (0)  
                // 2. Increment ONLY ways that were YOUNGER (count < hit_way_count)  
                //    Ways older than hit_way (count > hit_way_count) stay the same.  
                for (j = 0; j < NUM_WAYS; j = j + 1) begin  
                    if (j == hit_way) begin  
                        lru_counter[set_index][j] <= 2'b00;  
                    end else begin  
                        // Compare with the CURRENT value of the hit way (before update)  
                        if (lru_counter[set_index][j] < lru_counter[set_index][hit_way]) begin  
                            lru_counter[set_index][j] <= lru_counter[set_index][j] + 1'b1;  
                        end  
                    end  
                end  
            end else begin  
                read_data <= {DATA_WIDTH{1'b0}};  
                valid_out <= 1'b0;  
            end  
        end  
          
        // Write operation  
        if (write_en) begin  
            if (hit) begin  
                // Write Hit: Update Data and update LRU just like a read hit  
                data_array[set_index][hit_way] <= write_data;  
                  
                // --- TRUE LRU UPDATE ON HIT ---  
                for (j = 0; j < NUM_WAYS; j = j + 1) begin  
                    if (j == hit_way) begin  
                        lru_counter[set_index][j] <= 2'b00;  
                    end else begin  
                        if (lru_counter[set_index][j] < lru_counter[set_index][hit_way]) begin  
                            lru_counter[set_index][j] <= lru_counter[set_index][j] + 1'b1;  
                        end  
                    end  
                end  
            end else begin  
                // Write Miss: Allocate new line at lru_way (victim)  
                tag_array[set_index][lru_way] <= tag;  
                data_array[set_index][lru_way] <= write_data;  
                valid_array[set_index][lru_way] <= 1'b1;  
                  
                // --- TRUE LRU UPDATE ON MISS ---  
                // 1. New Way becomes MRU (0)  
                // 2. ALL other ways increment (because the new one effectively pushed everyone down)  
                for (j = 0; j < NUM_WAYS; j = j + 1) begin  
                    if (j == lru_way) begin  
                        lru_counter[set_index][j] <= 2'b00;  
                    end else begin  
                        // Saturate at 3 just in case, though logically shouldn't overflow if logic is perfect  
                        if (lru_counter[set_index][j] < 2'b11) begin  
                            lru_counter[set_index][j] <= lru_counter[set_index][j] + 1'b1;  
                        end  
                    end  
                end  
            end  
        end  
    end  
end
