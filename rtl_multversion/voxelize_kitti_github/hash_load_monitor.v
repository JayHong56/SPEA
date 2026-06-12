module hash_load_monitor #(
    parameter BUCKETS    = 64,
    parameter BUCKET_AW  = 6,
    parameter MAX_SLOTS  = 4,
    parameter ADDR_WIDTH = 8  
) (
    input  wire                  clk,
    input  wire                  rst_n,
    input  wire                  clear,

    
    input  wire                  alloc_valid,
    input  wire [BUCKET_AW-1:0]  alloc_bucket,

    
    input  wire                  kill_valid,
    input  wire [ADDR_WIDTH-1:0] kill_addr,

    
    output reg  [BUCKET_AW-1:0]  debug_addr,         
    output reg  [7:0]            debug_load,         
    output reg  [31:0]           total_full_buckets, 
    output reg  [31:0]           total_load          
);

    
    reg [$clog2(MAX_SLOTS+1)-1:0] bucket_load [0:BUCKETS-1];

    
    wire [BUCKET_AW-1:0] kill_bucket = kill_addr[ADDR_WIDTH-1 -: BUCKET_AW];

    
    wire alloc_makes_full    = alloc_valid && (bucket_load[alloc_bucket] == MAX_SLOTS - 1);
    wire kill_makes_not_full = kill_valid  && (bucket_load[kill_bucket] == MAX_SLOTS);
    wire same_bucket_conflict = (alloc_valid && kill_valid && (alloc_bucket == kill_bucket));

    integer i;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n || clear) begin
            total_load         <= 0;
            total_full_buckets <= 0;
            debug_addr         <= 0;
            debug_load         <= 0;
            for (i = 0; i < BUCKETS; i = i + 1) begin
                bucket_load[i] <= 0;
            end
        end else begin
            
            
            
            
            if (alloc_valid && !kill_valid) begin
                if (total_load < BUCKETS * MAX_SLOTS) begin
                    total_load <= total_load + 1'b1;
                end
            end else if (!alloc_valid && kill_valid) begin
                if (total_load != 0) begin
                    total_load <= total_load - 1'b1;
                end
            end
            

            
            
            
            if (same_bucket_conflict) begin
                
                debug_addr <= alloc_bucket;
                debug_load <= bucket_load[alloc_bucket]; 
            end else begin
                
                if (alloc_valid) begin
                    if (bucket_load[alloc_bucket] < MAX_SLOTS) begin
                        bucket_load[alloc_bucket] <= bucket_load[alloc_bucket] + 1'b1;
                    end
                    debug_addr <= alloc_bucket;
                    debug_load <= (bucket_load[alloc_bucket] < MAX_SLOTS) ? bucket_load[alloc_bucket] + 1'b1 : bucket_load[alloc_bucket];
                end
                
                
                if (kill_valid) begin
                    if (bucket_load[kill_bucket] != 0) begin
                        bucket_load[kill_bucket] <= bucket_load[kill_bucket] - 1'b1;
                    end
                    if (!alloc_valid) begin 
                        debug_addr <= kill_bucket;
                        debug_load <= (bucket_load[kill_bucket] != 0) ? bucket_load[kill_bucket] - 1'b1 : bucket_load[kill_bucket];
                    end
                end

                
                if (alloc_makes_full && kill_makes_not_full) begin
                    
                    total_full_buckets <= total_full_buckets;
                end else if (alloc_makes_full) begin
                    total_full_buckets <= total_full_buckets + 1'b1;
                end else if (kill_makes_not_full) begin
                    if (total_full_buckets != 0) begin
                        total_full_buckets <= total_full_buckets - 1'b1;
                    end
                end
            end
        end
    end

endmodule
