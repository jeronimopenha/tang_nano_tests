module usb_top (
    input  clk_27mhz,
    input  button_s1,
    input uart_rx,
    output [5:0] led,
    output uart_tx
    );
    
    wire rst;
    wire clk;
    assign rst = ~button_s1;
    assign clk = clk_27mhz;

    wire rx_bsy;
    wire tx_bsy;
    wire rx_timeout;
    wire rx_valid;
    wire [7:0] rx_data;

    assign led[0] = uart_rx;
    assign led[1] = ~rx_bsy;
    assign led[2] = ~rx_timeout;
    assign led[3] = uart_tx;
    assign led[4] = ~tx_bsy;
    
    uart_rx 
    u_uart_rx( 
    .clk(clk), //input
    .rst(rst), //input
    .rx(uart_rx), //input
    .rx_bsy(rx_bsy), // output
    .block_timeout(rx_timeout), // output
    .data_valid(rx_valid), // output
    .data_out(rx_data) // output
    );

    uart_tx 
    u_uart_tx
    (.clk(clk),// input
     .rst(rst),// input
     .send_trig(rx_valid),// input
     .send_data(rx_data[7:0]),// input [7:0]
     .tx(uart_tx),// output
     .tx_bsy(tx_bsy)// output
    );

endmodule

module uart_tx (
    input clk,
    input rst,  
    input send_trig,
    input [7:0] send_data,
    output reg tx,
    output reg tx_bsy
    );

    localparam SYSCLOCK = 27.0; // MHz
    localparam BAUDRATE = 3.0; // Mbps

    localparam CLKPERFRM = (SYSCLOCK/BAUDRATE)*10;
    // bit order is lsb-msb
    localparam TBITAT    = 1; // START bit
    localparam BIT0AT    = (SYSCLOCK/BAUDRATE*1)+1;
    localparam BIT1AT    = (SYSCLOCK/BAUDRATE*2)+1;
    localparam BIT2AT    = (SYSCLOCK/BAUDRATE*3)+1;
    localparam BIT3AT    = (SYSCLOCK/BAUDRATE*4)+1;
    localparam BIT4AT    = (SYSCLOCK/BAUDRATE*5)+1;
    localparam BIT5AT    = (SYSCLOCK/BAUDRATE*6)+1;
    localparam BIT6AT    = (SYSCLOCK/BAUDRATE*7)+1;
    localparam BIT7AT    = (SYSCLOCK/BAUDRATE*8)+1;
    localparam PBITAT    = (SYSCLOCK/BAUDRATE*9)+1;// STOP bit

    reg [9-1:0] tx_cnt; // tx flow control 
    //[$clog2(CLKPERFRM+1)-1:0] tx_cnt; // tx flow control
    
    reg [7:0] data2send; // buffer
    wire frame_begin;
    wire frame_end;

    assign frame_begin = send_trig & (~tx_bsy);
    assign frame_end = tx_bsy && (tx_cnt == CLKPERFRM);
    
    always@(posedge clk) begin
        if(rst) begin
            tx_bsy <= 1'b0;
        end else if (frame_begin) begin
            tx_bsy <= 1'b1;
        end else if (frame_end) begin
            tx_bsy <= 1'b0;
        end
    end
 
    always@(posedge clk) begin
        if (rst) begin
            tx_cnt <= 'd0;
        end else if (frame_end) begin
            tx_cnt <= 'd0;
        end else if (tx_bsy) begin
            tx_cnt <= tx_cnt + 1'b1;
        end
    end

    always@(posedge clk) begin
        if (rst) begin
            data2send <= 8'd0;
        end else if (frame_begin) begin
            data2send <= send_data;
        end
    end

    always@(posedge clk) begin 
        if (rst) begin
            tx <= 1'b1; // init val should be 1
        end else if (tx_bsy) begin
            case(tx_cnt) 
                TBITAT: tx <= 1'b0;
                BIT0AT: tx <= data2send[0];
                BIT1AT: tx <= data2send[1];
                BIT2AT: tx <= data2send[2];
                BIT3AT: tx <= data2send[3];
                BIT4AT: tx <= data2send[4];
                BIT5AT: tx <= data2send[5];
                BIT6AT: tx <= data2send[6];
                BIT7AT: tx <= data2send[7];
                PBITAT: tx <= 1'b1;
            endcase
        end else begin
            tx <= 1'b1;
        end
    end
endmodule


module uart_rx ( 
    input clk,
    input rst,
    input rx,
    output reg rx_bsy,        // high when rx is receiving data
    output reg block_timeout, // pulses high when timeout is reached
    output reg data_valid,    // pulses high when data_out is valid
    output reg [7:0] data_out       // byte frame data
    );

    localparam SYSCLOCK = 27.0; // MHz
    localparam BAUDRATE = 3.0; // Mbits
    
    localparam SYNC_DELAY = 2;
    localparam CLKPERFRM = (SYSCLOCK/BAUDRATE*9.8)-SYNC_DELAY;
    // bit order is lsb-msb
    localparam TBITAT    = (SYSCLOCK/BAUDRATE*0.8)-SYNC_DELAY; // START BIT
    localparam BIT0AT    = (SYSCLOCK/BAUDRATE*1.5)-SYNC_DELAY;
    localparam BIT1AT    = (SYSCLOCK/BAUDRATE*2.5)-SYNC_DELAY;
    localparam BIT2AT    = (SYSCLOCK/BAUDRATE*3.5)-SYNC_DELAY;
    localparam BIT3AT    = (SYSCLOCK/BAUDRATE*4.5)-SYNC_DELAY;
    localparam BIT4AT    = (SYSCLOCK/BAUDRATE*5.5)-SYNC_DELAY;
    localparam BIT5AT    = (SYSCLOCK/BAUDRATE*6.5)-SYNC_DELAY;
    localparam BIT6AT    = (SYSCLOCK/BAUDRATE*7.5)-SYNC_DELAY;
    localparam BIT7AT    = (SYSCLOCK/BAUDRATE*8.5)-SYNC_DELAY;
    localparam PBITAT    = (SYSCLOCK/BAUDRATE*9.2)-SYNC_DELAY; // STOP bit
    localparam BLK_TIMEOUT = BIT1AT; // this depends on your USB UART chip 

    reg [8:0] rx_cnt;      // rx flow control
    //[$clog2(CLKPERFRM):0] rx_cnt;      // rx flow control

    //logic rx_sync;
    reg rx_hold;
    wire frame_begin;
    wire frame_end;
    wire start_invalid;
    wire stop_invalid;
    reg timeout;
    
    /*synchronizer u_synchronizer
    ( .clk      (clk),    // input
      .rst_n    (rst_n),  // input
      .data_in  (rx),     // input
      .data_out (rx_sync) // output
     );*/

    always@(posedge clk) begin
        if (rst) begin
            rx_hold <= 1'b0; // this needs to match synchronizer reset val
        end else begin
            rx_hold <= rx;
        end
    end
        
    assign frame_begin = (!rx_bsy) && (!rx) && rx_hold; // negative edge detect
    assign frame_end   =   rx_bsy  && (rx_cnt == CLKPERFRM);      // final count

    // START bit must be low  for 80% of the bit duration
    assign start_invalid = rx_bsy && (rx_cnt < TBITAT) &&   rx;

    // STOP  bit must be high for 80% of the bit duration
    assign stop_invalid  = rx_bsy && (rx_cnt > PBITAT) && (!rx);

    always@(posedge clk) begin
        if (rst) begin
            rx_bsy <= 1'b0;
        end else if (frame_begin) begin
            rx_bsy <= 1'b1;
        end else if (start_invalid || stop_invalid) begin
            rx_bsy <= 1'b0;
        end else if (frame_end) begin
            rx_bsy <= 1'b0;
        end 
    end

    // count if frame is valid or until the timeout
    always@(posedge clk) begin
        if (rst) begin
            rx_cnt <= 'd0;
        end else if (frame_begin) begin 
            rx_cnt <= 'd0;
        end else if (start_invalid || stop_invalid || frame_end) begin 
            rx_cnt <= 'd0;
        end else if (!timeout) begin
            rx_cnt <= rx_cnt + 1'b1; 
        end else begin  
            rx_cnt <= 'd0;
        end 
    end

    // this just stops the rx_cnt
    always@(posedge clk) begin
        if (rst) begin 
            timeout <= 1'b0;
        end else if (frame_begin) begin
            timeout <= 1'b0;
        end else if ((!rx_bsy) && (rx_cnt == BLK_TIMEOUT)) begin 
            timeout <= 1'b1;
        end 
    end 

    // this signals the end of block uart transfer
    always@(posedge clk) begin
        if (rst) begin
            block_timeout <= 1'b0;
        end else if ((!rx_bsy) && (rx_cnt == BLK_TIMEOUT)) begin
            block_timeout <= 1'b1;
        end else begin
            block_timeout <= 1'b0;
        end
    end

    // this pulses upon completion of a clean frame
    always@(posedge clk) begin
        if (rst) begin
            data_valid <= 1'b0;
        end else if (frame_end) begin
            data_valid <= 1'b1;
        end else begin
            data_valid <= 1'b0;
        end
    end

    // rx data control
    always@(posedge clk) begin
        if (rst) begin
            data_out[7:0] <= 8'd0;
        end else if (rx_bsy) begin
            case(rx_cnt)
                BIT0AT: data_out[0] <= rx;
                BIT1AT: data_out[1] <= rx;
                BIT2AT: data_out[2] <= rx;
                BIT3AT: data_out[3] <= rx;
                BIT4AT: data_out[4] <= rx;
                BIT5AT: data_out[5] <= rx;
                BIT6AT: data_out[6] <= rx;
                BIT7AT: data_out[7] <= rx;
            endcase
        end
    end

endmodule