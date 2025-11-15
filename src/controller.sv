`default_nettype none
`timescale 1ns/1ns

module controller #(
    parameter ADDR_BITS     = 8,
    parameter DATA_BITS     = 16,
    parameter NUM_CONSUMERS = 4, // The number of consumers accessing memory through this controller
    parameter NUM_CHANNELS  = 1, // The number of concurrent channels available to send requests to global memory
    parameter WRITE_ENABLE  = 1  // Whether this memory controller can write to memory (program memory is read-only)
) (
    input  wire clk,
    input  wire reset,

    // Consumer Interface (Fetchers / LSUs)
    input  wire [NUM_CONSUMERS-1:0]                      consumer_read_valid,
    input  wire [ADDR_BITS-1:0]                          consumer_read_address [NUM_CONSUMERS-1:0],
    output reg  [NUM_CONSUMERS-1:0]                      consumer_read_ready,
    output reg  [DATA_BITS-1:0]                          consumer_read_data   [NUM_CONSUMERS-1:0],
    input  wire [NUM_CONSUMERS-1:0]                      consumer_write_valid,
    input  wire [ADDR_BITS-1:0]                          consumer_write_address[NUM_CONSUMERS-1:0],
    input  wire [DATA_BITS-1:0]                          consumer_write_data   [NUM_CONSUMERS-1:0],
    output reg  [NUM_CONSUMERS-1:0]                      consumer_write_ready,

    // Memory Interface (Data / Program)
    output reg  [NUM_CHANNELS-1:0]                       mem_read_valid,
    output reg  [ADDR_BITS-1:0]                          mem_read_address [NUM_CHANNELS-1:0],
    input  wire [NUM_CHANNELS-1:0]                       mem_read_ready,
    input  wire [DATA_BITS-1:0]                          mem_read_data   [NUM_CHANNELS-1:0],
    output reg  [NUM_CHANNELS-1:0]                       mem_write_valid,
    output reg  [ADDR_BITS-1:0]                          mem_write_address[NUM_CHANNELS-1:0],
    output reg  [DATA_BITS-1:0]                          mem_write_data   [NUM_CHANNELS-1:0],
    input  wire [NUM_CHANNELS-1:0]                       mem_write_ready
);
    localparam [2:0]
        IDLE          = 3'b000, 
        READ_WAITING  = 3'b010, 
        WRITE_WAITING = 3'b011,
        READ_RELAYING = 3'b100,
        WRITE_RELAYING= 3'b101;

    // Estado por canal y tracking de qué consumidor sirve cada canal
    reg [2:0]                       controller_state   [NUM_CHANNELS-1:0];
    reg [$clog2(NUM_CONSUMERS)-1:0] current_consumer   [NUM_CHANNELS-1:0];
    reg [NUM_CONSUMERS-1:0]         channel_serving_consumer;

    integer i, j;
    reg found;

    always @(posedge clk) begin
        if (reset) begin 
            // Reset por canal
            for (i = 0; i < NUM_CHANNELS; i = i + 1) begin
                mem_read_valid[i]     <= 1'b0;
                mem_read_address[i]   <= {ADDR_BITS{1'b0}};
                mem_write_valid[i]    <= 1'b0;
                mem_write_address[i]  <= {ADDR_BITS{1'b0}};
                mem_write_data[i]     <= {DATA_BITS{1'b0}};
                controller_state[i]   <= IDLE;
                current_consumer[i]   <= {($clog2(NUM_CONSUMERS)){1'b0}};
            end

            // Reset por consumidor
            for (j = 0; j < NUM_CONSUMERS; j = j + 1) begin
                consumer_read_ready[j]  <= 1'b0;
                consumer_read_data[j]   <= {DATA_BITS{1'b0}};
                consumer_write_ready[j] <= 1'b0;
            end

            channel_serving_consumer <= {NUM_CONSUMERS{1'b0}};

        end else begin 
            // Por defecto, no mantener ready en alto más de un ciclo
            for (j = 0; j < NUM_CONSUMERS; j = j + 1) begin
                consumer_read_ready[j]  <= 1'b0;
                consumer_write_ready[j] <= 1'b0;
            end

            // Manejo por canal
            for (i = 0; i < NUM_CHANNELS; i = i + 1) begin 
                case (controller_state[i])
                    IDLE: begin
                        // Buscar un consumidor con pedido pendiente
                        found = 1'b0;
                        for (j = 0; j < NUM_CONSUMERS; j = j + 1) begin 
                            // READ
                            if (!found && consumer_read_valid[j] && !channel_serving_consumer[j]) begin 
                                found                       = 1'b1;
                                channel_serving_consumer[j] <= 1'b1;
                                current_consumer[i]         <= j[$clog2(NUM_CONSUMERS)-1:0];

                                mem_read_valid[i]           <= 1'b1;
                                mem_read_address[i]         <= consumer_read_address[j];
                                controller_state[i]         <= READ_WAITING;
                            end
                            // WRITE
                            else if (!found && consumer_write_valid[j] && !channel_serving_consumer[j]) begin 
                                found                       = 1'b1;
                                channel_serving_consumer[j] <= 1'b1;
                                current_consumer[i]         <= j[$clog2(NUM_CONSUMERS)-1:0];

                                if (WRITE_ENABLE) begin
                                    mem_write_valid[i]      <= 1'b1;
                                    mem_write_address[i]    <= consumer_write_address[j];
                                    mem_write_data[i]       <= consumer_write_data[j];
                                    controller_state[i]     <= WRITE_WAITING;
                                end else begin
                                    // Si WRITE no está habilitado, soltar enseguida
                                    controller_state[i]         <= IDLE;
                                    channel_serving_consumer[j] <= 1'b0;
                                end
                            end
                        end
                    end

                    READ_WAITING: begin
                        // Espera respuesta de memoria
                        if (mem_read_ready[i]) begin 
                            mem_read_valid[i]                         <= 1'b0;
                            consumer_read_ready[current_consumer[i]]  <= 1'b1;
                            consumer_read_data[current_consumer[i]]   <= mem_read_data[i];
                            controller_state[i]                       <= READ_RELAYING;
                        end
                    end

                    WRITE_WAITING: begin 
                        // Espera respuesta de memoria para write
                        if (mem_write_ready[i]) begin 
                            mem_write_valid[i]                        <= 1'b0;
                            consumer_write_ready[current_consumer[i]] <= 1'b1;
                            controller_state[i]                       <= WRITE_RELAYING;
                        end
                    end

                    // Esperar a que el consumer baje su valid, luego liberar
                    READ_RELAYING: begin
                        if (!consumer_read_valid[current_consumer[i]]) begin 
                            channel_serving_consumer[current_consumer[i]] <= 1'b0;
                            controller_state[i]                           <= IDLE;
                        end
                    end

                    WRITE_RELAYING: begin 
                        if (!consumer_write_valid[current_consumer[i]]) begin 
                            channel_serving_consumer[current_consumer[i]] <= 1'b0;
                            controller_state[i]                           <= IDLE;
                        end
                    end
                endcase
            end
        end
    end
endmodule
