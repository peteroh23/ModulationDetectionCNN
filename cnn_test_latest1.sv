
`timescale 1ns/1ns

module testbench_iteration1();
	reg clk_50, clk_25, reset;
	
	reg [31:0] index;
	
	//Initialize clocks and index
	initial begin
		clk_50 = 1'b0;
		clk_25 = 1'b0;
		index  = 32'd0;
	end
	
	//Toggle the clocks
	always begin
		#10
		clk_50  = !clk_50;
	end
	
	always begin
		#20
		clk_25  = !clk_25;
	end
	
	//Intialize and drive signals
	initial begin
		reset  = 1'b0;
		#10 
		reset  = 1'b1;
		#30
		reset  = 1'b0;
	end
	
	//Increment index
	always @ (posedge clk_50) begin
		index  <= index + 32'd1;
		
	end

	wire signed [31:0] final_output [2:0];
	//signed [17:0] out_data [1:0][128:0][9:0]
    	wire nn_done;
    	wire signed [1:0][17:0] test_data [127:0];

	wire input_done;
	wire start;
	

    	// Use rom table to assign test_data


    	test_input TOP ( .clock(clk_50),
                    	 .reset(reset),
                    	 .test_data(test_data),
			            .done(input_done)
   	 );


	
	 layers_top TOP1(.clock(clk_50),
                    .start(input_done),
                    .reset(reset), 
                    .in_data(test_data), 
                    .out_data(final_output), 
                    .done(nn_done));

endmodule

module test_input(clock,reset,test_data, done);
	input clock;
	input reset;
	output reg signed [1:0][17:0] test_data [127:0];
	
	reg [17:0] i_addr;
	reg [17:0] q_addr;
	reg signed [17:0] i_data;
	reg signed [17:0] q_data;
	//reg i_q_index;
	reg [7:0] sample_index;

	output reg done;
	

	always @ (posedge clock) begin
		if(reset)begin
			i_addr <= 18'd0;
			q_addr <= 18'd128;
			sample_index <= 8'd0;
			done <= 1'b0;
			//test_data[sample_index]<={i_data, q_data};
		end
		else if(sample_index == 8'd128) begin
			done <= 1'b1;
            sample_index <= sample_index + 18'd1;
		end
        else if(sample_index == 8'd129) begin //Make sure done is a pulse
			done <= 1'b0;
		end 
		else begin
			test_data[sample_index]<={q_data,i_data}; //Makes q=1, i=0

			sample_index <= sample_index + 18'd1;
			i_addr <= i_addr + 18'd1;
			q_addr <= q_addr + 18'd1;
		end
	end
	
	test_table test1(	.address(i_addr),
				.data(i_data)
	);

	test_table test2(	.address(q_addr),
				.data(q_data)
	);
endmodule


////////////////////////////////////////////////
// provides [1:0][127:0] test input
module test_table (address, data);
input [17:0] address;
output reg signed [17:0] data;
//reg signed [17:0] data;

always@(address)
begin
    case(address)
   	18'd000: data = 18'b000000000000001011;
	//18'd001: data = 18'b000000000000010111;
	//18'd002: data = 18'b000000000000010111;
	18'd128: data = 18'b111111111111100000;
	//18'd129: data = 18'b111111111111110000;
	//18'd130: data = 18'b111111111111110000;
	
	default: data = 18'b000000000000000000;


    endcase
 end
endmodule

module layers_top(clock, start,reset, in_data, out_data, done);
    input clock;
    input reset;
    input start;
    input signed [1:0][17:0] in_data [127:0];
    output reg signed [31:0] out_data [2:0];

    //reg reset2;
    reg signed [31:0] pre_out_data [2:0];


    output reg done;

    wire signed [128:0][1:0][17:0]conv_output [2:0];
    wire conv_done;
    wire dense_done;
	 
	 reg started;
	

    always @(posedge clock) begin 
         if (reset) begin
             done <= 0;
				 started <= 0;
             //reset2<=1'b0;
         end else if (start && !started) begin
		done <= 0;
		started <= 1;
			end
         else if (dense_done)begin
            done <= 1'b1;
            out_data[0] <= (pre_out_data[0][31] == 0)? pre_out_data[0]:32'sd0;
            out_data[1] <= ((pre_out_data[1][31] == 0)? pre_out_data[1]:32'sd0); 
            out_data[2] <= ((pre_out_data[2][31] == 0)? pre_out_data[2]:32'sd0);
         end
        //  else if(conv_done == 1'b1) begin
        //      reset2<=1'b1;
        //  end
         else begin
			
         end


    end

    // INPUT: input signed [17:0] in_data [1:0][127:0]
    // OUTPUT: output signed [17:0] out_data [1:0][128:0][9:0]
    conv_top CON ( .clock(clock),
                  .reset(reset),
		            .start(start),
                  .in_data(in_data),
                  .out_data(conv_output),
                  .done(conv_done)
    );

//    // INPUT: input signed [17:0] out_data [1:0][128:0][9:0]
//    // OUTPUT: output signed [17:0] out_data [2:0]
    dense_layer DEN ( .clock(clock),
                    .reset(reset),
                    .in_data(conv_output),
                    .dense_out(pre_out_data),
                    .done(dense_done),
		              .start(conv_done)
    );

endmodule

// module dense_top(clock, reset, in_data, out_data, done, start);
//     input clock;
// 	input reset;
// 	input start;
//     input signed [128:0][1:0][17:0] in_data [9:0];
//     output signed [31:0] out_data [2:0]; // depends on how many classifications we want
//     output done;

// 	wire done_dense[2:0];
// 	assign done = ((done_dense[0]==1)&&(done_dense[1]==1)&&(done_dense[2]==1));

//     generate
//         genvar k;
//         for (k=0; k <= 2; k = k+1) begin: ClassGen
//             wire [1:0] class_index;
//             assign class_index = k;

//             dense_layer layer(  .clock(clock),
//                                 .reset(reset),
//                                 .dense_out(out_data[k]), // apply ReLU activation here or very top top module.
//                                 .class_index(class_index),
//                                 .done(done_dense[k]),
// 				                .in_data(in_data),
// 				                .start(start)
//             );
//         end
//     endgenerate
// endmodule

module dense_layer(clock, reset, done, dense_out, in_data, start);
    input clock;
    input reset;
    input start;
    // input [1:0] class_index;
    input signed [128:0][1:0][17:0] in_data [2:0];

	//wire [9:0][128:0][1:0][17:0]in_data_wire; // flatten the layer data
	//assign  in_data_wire = {in_data[9], in_data[8], in_data[7], in_data[6], in_data[5],in_data[4], in_data[3], in_data[2], in_data[1], in_data[0]};

    output reg done;
    output signed [31:0] dense_out [2:0];

    reg signed [31:0] dense_out_reg [2:0];
    assign dense_out = dense_out_reg;

        // reg signed [17:0] current_output;
    wire signed [17:0] current_output [2:0];

    reg signed [17:0] dense_bias [2:0];
    assign dense_bias [0] = 18'b000000001100100111; // hardcode now
    assign dense_bias [1] = 18'b000000000100111101; // hardcode now
    assign dense_bias [2] = 18'b111111111100000010; // hardcode now

    reg [17:0] weight_address;
    // reg [17:0] q_weight_address [2:0];

    // WE need to format the rom table of weights to be 
    //0 - 1290: First Class I Weights
    // 1290-2580: First Class Q Weights
    // 2580 - 3870: Second Class I Weights
    // 3870 - 5160: Second Class Q Weights
    // 5160 - 6450: Third Class I Weights
    // 6450 - 7740: Third Class Q Weights
    
    assign weight_address = 18'd0;
    // assign weight_address[0] = 18'd1290;
    // assign weight_address[1] = 18'd2580;
    // assign weight_address[1] = 18'd3870;
    // assign weight_address[2] = 18'd5160;
    // assign weight_address[2] = 18'd6450;

    reg [17:0] weight_address_reg;
    // reg [17:0] weight_address_reg;

    wire signed [1:0][17:0] weight [2:0];

    reg [7:0] sample_index;
    reg [3:0] kernel_index;

    reg started;

    always @(posedge clock) begin
        if (reset || (start && !started)) begin
            sample_index <= 8'd0;
            kernel_index <= 4'd0;
            weight_address_reg <= weight_address;
            dense_out_reg[0] <= {dense_bias[0][17],dense_bias[0][17],dense_bias[0][17],
                                dense_bias[0][17],dense_bias[0][17],dense_bias[0][17],
                                dense_bias[0][17],dense_bias[0][17],dense_bias[0][17],
                                dense_bias[0][17],dense_bias[0][17],dense_bias[0][17],
                                dense_bias[0][17],dense_bias[0][17],dense_bias[0]};
            dense_out_reg[1] <= {dense_bias[1][17],dense_bias[1][17],dense_bias[1][17],
                                dense_bias[1][17],dense_bias[1][17],dense_bias[1][17],
                                dense_bias[1][17],dense_bias[1][17],dense_bias[1][17],
                                dense_bias[1][17],dense_bias[1][17],dense_bias[1][17],
                                dense_bias[1][17],dense_bias[1][17],dense_bias[1]};
            dense_out_reg[2] <= {dense_bias[2][17],dense_bias[2][17],dense_bias[2][17],
                                dense_bias[2][17],dense_bias[2][17],dense_bias[2][17],
                                dense_bias[2][17],dense_bias[2][17],dense_bias[2][17],
                                dense_bias[2][17],dense_bias[2][17],dense_bias[2][17],
                                dense_bias[2][17],dense_bias[2][17],dense_bias[2]};
            done <= 1'd0;
            if (start) begin
                started <= 1;
            end else begin
                started <= 0;
            end
        end
        // finished the last sample
        else if (sample_index == 8'd128) begin
            // on the last kernel
            if (kernel_index == 4'd2) begin
		
                done  <= 1'd1;
            end
            else begin
             	 sample_index <= 8'd0;
            	 kernel_index <= kernel_index + 4'd1;
             end
            
        end
        // keep adding one to indexes
        else if (started) begin
            sample_index <= sample_index + 8'd1;
            // i_weight_address_reg <= i_weight_address_reg + 18'd1;
            weight_address_reg <= weight_address_reg + 18'd1;
            dense_out_reg[0] <= dense_out_reg[0] + {current_output[0][17],current_output[0][17],current_output[0][17],current_output[0][17],current_output[0][17]
						,current_output[0][17],current_output[0][17],current_output[0][17],current_output[0][17],current_output[0][17],current_output[0][17],
						current_output[0][17],current_output[0][17],current_output[0][17],current_output[0]};
            dense_out_reg[1]<= dense_out_reg[1] + {current_output[1][17],current_output[1][17],current_output[1][17],current_output[1][17],current_output[1][17]
						,current_output[1][17],current_output[1][17],current_output[1][17],current_output[1][17],current_output[1][17],current_output[1][17],
						current_output[1][17],current_output[1][17],current_output[1][17],current_output[1]};
            dense_out_reg[2] <= dense_out_reg[2] + {current_output[2][17],current_output[2][17],current_output[2][17],current_output[2][17],current_output[2][17]
						,current_output[2][17],current_output[2][17],current_output[2][17],current_output[2][17],current_output[2][17],current_output[2][17],
						current_output[2][17],current_output[2][17],current_output[2][17],current_output[2]};
        end
        
    end
    
    //get i weight
    rom_dense_i_class1 read_weight1_1(.clock(clock),
                        .address(weight_address_reg),
                        .data(weight[0][0]));

    // get q weight
    rom_dense_q_class1 read_weight2_1(.clock(clock),
                        .address(weight_address_reg),
                        .data(weight[0][1]));

                        //get i weight
    rom_dense_i_class2 read_weight1_2(.clock(clock),
                        .address(weight_address_reg),
                        .data(weight[1][0]));

    // get q weight
    rom_dense_q_class2 read_weight2_2(.clock(clock),
                        .address(weight_address_reg),
                        .data(weight[1][1]));

                        //get i weight
    rom_dense_i_class3 read_weight1_3(.clock(clock),
                        .address(weight_address_reg),
                        .data(weight[2][0]));

    // get q weight
    rom_dense_q_class3 read_weight2_3(.clock(clock),
                        .address(weight_address_reg),
                        .data(weight[2][1]));

    signed_mult mul3( .out(current_output[0]),
                      .a(in_data[kernel_index][sample_index][0]), // I Data
                      .b(weight[0][0]), // I Weight
                      .c(in_data[kernel_index][sample_index][1]), // Q Data
                      .d(weight[0][1]) // Q Weight
    );
    signed_mult mul4( .out(current_output[1]),
                      .a(in_data[kernel_index][sample_index][0]), // I Data
                      .b(weight[1][0]), // I Weight
                      .c(in_data[kernel_index][sample_index][1]), // Q Data
                      .d(weight[1][1]) // Q Weight
    );
    signed_mult mul5( .out(current_output[2]),
                      .a(in_data[kernel_index][sample_index][0]), // I Data
                      .b(weight[2][0]), // I Weight
                      .c(in_data[kernel_index][sample_index][1]), // Q Data
                      .d(weight[2][1]) // Q Weight
    );

endmodule

module conv_top(clock, reset,in_data, out_data, done, start);
    input clock;
    input reset;
    input start;
    input signed [1:0][17:0] in_data [127:0];
    output wire signed [128:0][1:0][17:0] out_data [2:0];

   
    output done;
	wire done_wire[2:0];
    assign done = done_wire[0];
	// wire done_wire[2:0];
    //wire [3:0] kernel_index;
    generate
        genvar j;
        for (j=0; j<=2; j=j+1) begin:  kernelGen
            	wire [3:0] kernel_index;
		        assign kernel_index = j;
		

            	conv_layer kernel( .clock(clock), 
                            	  .reset(reset),
                            	  .cov_out(out_data[j]),
                            	  .kernel_index(kernel_index),
                                .in_data(in_data),
                            	  .done(done_wire[j]),
										  .start(start)           
                );              
 
        end
    endgenerate
endmodule

// small state machine for 129 compute one by one!
module conv_layer(clock, reset, kernel_index, in_data ,done, cov_out, start);
    input clock;
    input reset;
    input start;
    input [3:0] kernel_index;
    
    input signed [1:0][17:0] in_data [127:0];

    //reg [17:0] zero_padding [1:0];
	
    reg signed [1:0][17:0] ad_in_data [129:0];
    reg started;

    output reg done;
    // state machine variables
    // reg [2:0] state;
    // parameter init = 0, start=1, s1 = 2, s2 = 3;
    output signed [128:0][1:0][17:0] cov_out;
    reg signed [1:0][17:0] out_cov [128:0];
    wire signed [1:0][17:0] cov_out_wire;
    
    assign cov_out = {out_cov[0],out_cov[1],out_cov[2],out_cov[3],out_cov[4],out_cov[5],out_cov[6],out_cov[7],out_cov[8],out_cov[9],out_cov[10],
		     out_cov[11],out_cov[12],out_cov[13],out_cov[14],out_cov[15],out_cov[16],out_cov[17],out_cov[18],out_cov[19],out_cov[20],
		     out_cov[21],out_cov[22],out_cov[23],out_cov[24],out_cov[25],out_cov[26],out_cov[27],out_cov[28],out_cov[29],out_cov[30],
		     out_cov[31],out_cov[32],out_cov[33],out_cov[34],out_cov[35],out_cov[36],out_cov[37],out_cov[38],out_cov[39],out_cov[40],
		     out_cov[41],out_cov[42],out_cov[43],out_cov[44],out_cov[45],out_cov[46],out_cov[47],out_cov[48],out_cov[49],out_cov[50],
		     out_cov[51],out_cov[52],out_cov[53],out_cov[54],out_cov[55],out_cov[56],out_cov[57],out_cov[58],out_cov[59],out_cov[60],
		     out_cov[61],out_cov[62],out_cov[63],out_cov[64],out_cov[65],out_cov[66],out_cov[67],out_cov[68],out_cov[69],out_cov[70],
		     out_cov[71],out_cov[72],out_cov[73],out_cov[74],out_cov[75],out_cov[76],out_cov[77],out_cov[78],out_cov[79],out_cov[80],
		     out_cov[81],out_cov[82],out_cov[83],out_cov[84],out_cov[85],out_cov[86],out_cov[87],out_cov[88],out_cov[89],out_cov[90],
		     out_cov[91],out_cov[92],out_cov[93],out_cov[94],out_cov[95],out_cov[96],out_cov[97],out_cov[98],out_cov[99],out_cov[100],
		     out_cov[101],out_cov[102],out_cov[103],out_cov[104],out_cov[105],out_cov[106],out_cov[107],out_cov[108],out_cov[109],out_cov[110],
		     out_cov[111],out_cov[112],out_cov[113],out_cov[114],out_cov[115],out_cov[116],out_cov[117],out_cov[118],out_cov[119],out_cov[120],
		     out_cov[121],out_cov[122],out_cov[123],out_cov[124],out_cov[125],out_cov[126],out_cov[127], out_cov[128]};

    //output [17:0] cov_out [1:0][128:0];

    reg [7:0] sample_index;
    integer i;


    always @(posedge clock) begin
        if(start || reset || !started) begin
            // state <= init;
                sample_index <= 8'd0;
                done <= 1'b0;
                for(i = 1; i<129; i=i+1) begin
                    ad_in_data[i][0] <= in_data[i-1][0];
						  ad_in_data[i][1] <= in_data[i-1][1];
                end
                ad_in_data[0] <= {18'd0,18'd0};
                ad_in_data[129] <= {18'd0,18'd0};

            if(start) begin
                started <= 1;
            end
            else begin
                started <= 0;
            end
    //	    ad_in_data[1][128:1]<= in_data[1][127:0];
    //	    ad_in_data[0][0]<=18'd0;
    //	    ad_in_data[1][0]<=18'd0;
    //	    ad_in_data[0][129]<=18'd0;
    //	    ad_in_data[1][129]<=18'd0;

        end
        else if (sample_index == 8'd129) begin
            done <= 1'b1;
            //reset <= 1'b1;
        end
        else begin
            out_cov[8'd128 - sample_index] <= cov_out_wire;
            sample_index <= sample_index + 8'd1;
        end
    end

    conv_compute cal( 	.clock(clock), 
                        .reset(reset),
                        .nero_out(cov_out_wire),
                        .kernel_index(kernel_index),
                        .input_data1(ad_in_data[sample_index]),
                        .input_data2(ad_in_data[sample_index+1])        
    );              
            


endmodule

module conv_compute(clock, reset, nero_out, input_data1,input_data2, kernel_index);
    input clock;
    input reset;
    input signed [1:0][17:0] input_data1;
    input signed [1:0][17:0] input_data2;
    reg signed [17:0] ad_in_data1[1:0];
    reg signed [17:0] ad_in_data2[1:0];
    //assign ad_in_data1 = {input_data1[17:0], input_data1[35:18]};
    //assign ad_in_data2 = {input_data2[17:0], input_data2[35:18]};

    input [3:0] kernel_index;
    output signed [1:0][17:0] nero_out; // cov 2D: [1,2]
    reg signed [1:0][17:0] nero_out_reg;
    assign nero_out = nero_out_reg;
    //assign nero_out = {nero_out_reg[0], nero_out_reg[1]};


    wire signed [17:0] weight [1:0];
    wire signed [17:0] bias_v;

    reg [17:0] weight_addr [1:0];
    reg [17:0] bias_addr;
    

    //
    always @(posedge clock) begin
	if(reset) begin
            weight_addr[0] <=18'd0;
            weight_addr[1] <=18'd1;
            bias_addr <= 18'd2;
        end
        //finish one kernel // indicate which kernel right now// adjust address 
        else if (kernel_index == 4'd1) begin
            weight_addr[0] <= 18'd3;
            weight_addr[1] <= 18'd4;
            bias_addr <= 18'd5;
        end
        else if (kernel_index == 4'd2) begin
            weight_addr[0] <= 18'd6;
            weight_addr[1] <= 18'd7;
            bias_addr <=18'd8;
        end
        else if (kernel_index == 4'd3) begin
            weight_addr[0] <= 18'd9;
            weight_addr[1] <= 18'd10;
            bias_addr <= 18'd11;
        end
        else if (kernel_index == 4'd4) begin
            weight_addr[0] <= 18'd12;
            weight_addr[1] <= 18'd13; 
            bias_addr <= 18'd14;
        end
        else if (kernel_index == 4'd5) begin
            weight_addr[0] <= 18'd15;
            weight_addr[1] <= 18'd16; 
            bias_addr <= 18'd17;
        end
		  else if (kernel_index == 4'd6) begin
            weight_addr[0] <= 18'd18;
            weight_addr[1] <= 18'd19; 
            bias_addr <= 18'd20;
        end
		  else if (kernel_index == 4'd7) begin
            weight_addr[0] <= 18'd21;
            weight_addr[1] <= 18'd22; 
            bias_addr <= 18'd23;
        end
		  else if (kernel_index == 4'd8) begin
            weight_addr[0] <= 18'd24;
            weight_addr[1] <= 18'd25; 
            bias_addr <= 18'd26;
        end
		  else if (kernel_index == 4'd9) begin
            weight_addr[0] <= 18'd27;
            weight_addr[1] <= 18'd28; 
            bias_addr <= 18'd29;
        end
		 else begin
            weight_addr[0] <= weight_addr[0];
            weight_addr[1] <= weight_addr[1];
            bias_addr <= bias_addr;
        end
    end

    // cal for I 
    signed_mult1 mul1(.dout_relu(nero_out_reg[0]),
                      .a(input_data1[0]), 
                      .b(weight[0]),
                      .c(input_data2[0]),
                      .d(weight[1]),
                      .e(bias_v));
    
    // cal for Q
    signed_mult1 mul2(.dout_relu(nero_out_reg[1]),
                      .a(input_data1[1]), 
                      .b(weight[0]),
                      .c(input_data2[1]),
                      .d(weight[1]),
                      .e(bias_v));

    rom_cov read_weight1(.clock(clock),
                        .address(weight_addr[0]),
                        .data(weight[0]));

    rom_cov read_weight2(.clock(clock),
                        .address(weight_addr[1]),
                        .data(weight[1]));

    rom_cov read_bias(  .clock(clock),
                        .address(bias_addr),
                        .data(bias_v));



endmodule



//multiplier for cov
module signed_mult1 (dout_relu, a, b, c, d, e);
	output 	signed  [17:0]	dout_relu;
	input 	signed	[17:0] 	a;
	input 	signed	[17:0] 	b;
    input   signed	[17:0] 	c;
	input 	signed	[17:0] 	d;
    input   signed  [17:0]  e;
	// intermediate full bit length
	wire signed	[35:0]	mult_out;
    wire signed     [17:0]  out;
wire signed [17:0] out_add;
	assign mult_out = a * b + c*d;
	// select bits for 4.14 fixed point
	assign out = {mult_out[35], mult_out[28:12]};
	assign out_add = out+e;
    assign dout_relu = ((out_add[17] == 0) ? out_add : 18'sd0);
endmodule

//////////////////////////////////////////////////
//// signed mult of 6.12 format 2'comp////////////
//////////////////////////////////////////////////

module signed_mult (out, a, b, c, d);
	output 	signed  [17:0]	out;
	input 	signed	[17:0] 	a;
	input 	signed	[17:0] 	b;
    input   signed	[17:0] 	c;
	input 	signed	[17:0] 	d;
	// intermediate full bit length
	wire 	signed	[35:0]	mult_out;
	assign mult_out = a * b + c*d;
	// select bits for 1.9 fixed point
	assign out = {mult_out[35], mult_out[28:12]};
endmodule
//////////////////////////////////////////////////



////////////////////////////////////////////////
// provides weight/bias parameters in cov layer
// 10*2 + 10
// for easy calculation: 
// 6.1219 table
module rom_cov (clock, address, data);
input [17:0] address;
input clock;
output [17:0] data;
reg signed [17:0] data;

always@(posedge clock)
begin
    case(address)
    /////#load table
        18'd00: data <= 18'b111101111010010001;
        18'd01: data <= 18'b111110001011100100;
        18'd02: data <= 18'b000000000000011101;
        18'd03: data <= 18'b000010000100010011;
        18'd04: data <= 18'b000001101100011010;
        18'd05: data <= 18'b111111111111111101;
        18'd06: data <= 18'b000001011100001010;
        18'd07: data <= 18'b000001100011110001;
        18'd08: data <= 18'b000000000011110001;
        
    endcase
end
endmodule



//0 - 1290: First Class I Weights
// 1290-2580: First Class Q Weights
// 2580 - 3870: Second Class I Weights
// 3870 - 5160: Second Class Q Weights
// 5160 - 6450: Third Class I Weights
// 6450 - 7740: Third Class Q Weights
////////////////////////////////////////////////
// provides bias parameters in dense1 layer
module  rom_dense_i_class1  (clock, address, data);
input clock;
input [17:0] address;
output [17:0] data;
reg signed [17:0] data;

always@(posedge clock)
begin
    case(address)
    ////load tabel
    18'd000: data <= 18'b111111100001001000;
18'd001: data <= 18'b111111000110111000;
18'd002: data <= 18'b111111001111110111;
18'd003: data <= 18'b111111010001001110;
18'd004: data <= 18'b111111010000001011;
18'd005: data <= 18'b111111010011000101;
18'd006: data <= 18'b111111011001110000;
18'd007: data <= 18'b111111010110100011;
18'd008: data <= 18'b111111010101010101;
18'd009: data <= 18'b111111011000010000;
18'd010: data <= 18'b111111011001000011;
18'd011: data <= 18'b111111010111000111;
18'd012: data <= 18'b111111011011000011;
18'd013: data <= 18'b111111010111011000;
18'd014: data <= 18'b111111010010011001;
18'd015: data <= 18'b111111001110010000;
18'd016: data <= 18'b111111010110000111;
18'd017: data <= 18'b111111010010110011;
18'd018: data <= 18'b111111010000111001;
18'd019: data <= 18'b111111010101100110;
18'd020: data <= 18'b111111010100010011;
18'd021: data <= 18'b111111010101011000;
18'd022: data <= 18'b111111001011001011;
18'd023: data <= 18'b111111001111000111;
18'd024: data <= 18'b111111010111100111;
18'd025: data <= 18'b111111011001100111;
18'd026: data <= 18'b111111010101101110;
18'd027: data <= 18'b111111010001010101;
18'd028: data <= 18'b111111010011011000;
18'd029: data <= 18'b111111010110100101;
18'd030: data <= 18'b111111010011010011;
18'd031: data <= 18'b111111010111101101;
18'd032: data <= 18'b111111010010001000;
18'd033: data <= 18'b111111010101100100;
18'd034: data <= 18'b111111010100011010;
18'd035: data <= 18'b111111010101010010;
18'd036: data <= 18'b111111010010010010;
18'd037: data <= 18'b111111010101110110;
18'd038: data <= 18'b111111010110110000;
18'd039: data <= 18'b111111010001110111;
18'd040: data <= 18'b111111010010000010;
18'd041: data <= 18'b111111001101110101;
18'd042: data <= 18'b111111011010010011;
18'd043: data <= 18'b111111001110110011;
18'd044: data <= 18'b111111010110001111;
18'd045: data <= 18'b111111010100001010;
18'd046: data <= 18'b111111010010011010;
18'd047: data <= 18'b111111001111111101;
18'd048: data <= 18'b111111010100101100;
18'd049: data <= 18'b111111010011011100;
18'd050: data <= 18'b111111010001101110;
18'd051: data <= 18'b111111010011111011;
18'd052: data <= 18'b111111011001011101;
18'd053: data <= 18'b111111010011100000;
18'd054: data <= 18'b111111011010111000;
18'd055: data <= 18'b111111010001100001;
18'd056: data <= 18'b111111011110000100;
18'd057: data <= 18'b111111010101001000;
18'd058: data <= 18'b111111010101000111;
18'd059: data <= 18'b111111010110000001;
18'd060: data <= 18'b111111011100110011;
18'd061: data <= 18'b111111010110101010;
18'd062: data <= 18'b111111011001001111;
18'd063: data <= 18'b111111011101011011;
18'd064: data <= 18'b111111011001100101;
18'd065: data <= 18'b111111011100111100;
18'd066: data <= 18'b111111011001100000;
18'd067: data <= 18'b111111010100010000;
18'd068: data <= 18'b111111010110010000;
18'd069: data <= 18'b111111011111011010;
18'd070: data <= 18'b111111011111000111;
18'd071: data <= 18'b111111010111010111;
18'd072: data <= 18'b111111010110000101;
18'd073: data <= 18'b111111010110011111;
18'd074: data <= 18'b111111010111111011;
18'd075: data <= 18'b111111010111001100;
18'd076: data <= 18'b111111010111101110;
18'd077: data <= 18'b111111010111010111;
18'd078: data <= 18'b111111010000010111;
18'd079: data <= 18'b111111010111111101;
18'd080: data <= 18'b111111011100000110;
18'd081: data <= 18'b111111010110111101;
18'd082: data <= 18'b111111011001000001;
18'd083: data <= 18'b111111010000110010;
18'd084: data <= 18'b111111010001111100;
18'd085: data <= 18'b111111010110000100;
18'd086: data <= 18'b111111010100100010;
18'd087: data <= 18'b111111011011101100;
18'd088: data <= 18'b111111011010101000;
18'd089: data <= 18'b111111011000101101;
18'd090: data <= 18'b111111010011011010;
18'd091: data <= 18'b111111010110110000;
18'd092: data <= 18'b111111010100000101;
18'd093: data <= 18'b111111010100001110;
18'd094: data <= 18'b111111010000001100;
18'd095: data <= 18'b111111010001001101;
18'd096: data <= 18'b111111010011111100;
18'd097: data <= 18'b111111011001010110;
18'd098: data <= 18'b111111010100110000;
18'd099: data <= 18'b111111010101011101;
18'd100: data <= 18'b111111010100110111;
18'd101: data <= 18'b111111010110001101;
18'd102: data <= 18'b111111010010001100;
18'd103: data <= 18'b111111011010000100;
18'd104: data <= 18'b111111010101110001;
18'd105: data <= 18'b111111010010111010;
18'd106: data <= 18'b111111001111111010;
18'd107: data <= 18'b111111010010100000;
18'd108: data <= 18'b111111011010010001;
18'd109: data <= 18'b111111010010111100;
18'd110: data <= 18'b111111010000100101;
18'd111: data <= 18'b111111001111010100;
18'd112: data <= 18'b111111001111110000;
18'd113: data <= 18'b111111010110111111;
18'd114: data <= 18'b111111010010001110;
18'd115: data <= 18'b111111010101000000;
18'd116: data <= 18'b111111011000010101;
18'd117: data <= 18'b111111011010011000;
18'd118: data <= 18'b111111010000110101;
18'd119: data <= 18'b111111011000101100;
18'd120: data <= 18'b111111010011000101;
18'd121: data <= 18'b111111010000001110;
18'd122: data <= 18'b111111010011011000;
18'd123: data <= 18'b111111011010100011;
18'd124: data <= 18'b111111011010101101;
18'd125: data <= 18'b111111011011111100;
18'd126: data <= 18'b111111010010001110;
18'd127: data <= 18'b111111010011110111;
18'd128: data <= 18'b111111011111010011;
18'd129: data <= 18'b000000110011110010;
18'd130: data <= 18'b000001000110111100;
18'd131: data <= 18'b000001000000001001;
18'd132: data <= 18'b000001000001000110;
18'd133: data <= 18'b000001000001010011;
18'd134: data <= 18'b000001000000100101;
18'd135: data <= 18'b000001000011110111;
18'd136: data <= 18'b000001000100001001;
18'd137: data <= 18'b000001000000110000;
18'd138: data <= 18'b000000111100001111;
18'd139: data <= 18'b000000111011011100;
18'd140: data <= 18'b000001000001001101;
18'd141: data <= 18'b000000111000101110;
18'd142: data <= 18'b000000111110011000;
18'd143: data <= 18'b000000111001111111;
18'd144: data <= 18'b000000111111101000;
18'd145: data <= 18'b000001000000100001;
18'd146: data <= 18'b000000111101100010;
18'd147: data <= 18'b000001000011111100;
18'd148: data <= 18'b000001000110111101;
18'd149: data <= 18'b000001000000101111;
18'd150: data <= 18'b000001000001111010;
18'd151: data <= 18'b000001000001101110;
18'd152: data <= 18'b000001000110111101;
18'd153: data <= 18'b000001000001100100;
18'd154: data <= 18'b000001000100011001;
18'd155: data <= 18'b000001000001100111;
18'd156: data <= 18'b000001000101111110;
18'd157: data <= 18'b000001000100001110;
18'd158: data <= 18'b000001000011010100;
18'd159: data <= 18'b000001000000110000;
18'd160: data <= 18'b000001000000011101;
18'd161: data <= 18'b000001001001000011;
18'd162: data <= 18'b000001000100010111;
18'd163: data <= 18'b000001000110000011;
18'd164: data <= 18'b000001000110010111;
18'd165: data <= 18'b000001000110001101;
18'd166: data <= 18'b000000111110011100;
18'd167: data <= 18'b000001000011000001;
18'd168: data <= 18'b000001000101001100;
18'd169: data <= 18'b000001000011000000;
18'd170: data <= 18'b000001000110000010;
18'd171: data <= 18'b000001000110000011;
18'd172: data <= 18'b000001000000110000;
18'd173: data <= 18'b000000111110100111;
18'd174: data <= 18'b000001001000100000;
18'd175: data <= 18'b000001000100110001;
18'd176: data <= 18'b000001000100101011;
18'd177: data <= 18'b000001000101010000;
18'd178: data <= 18'b000001000111101000;
18'd179: data <= 18'b000000111101011111;
18'd180: data <= 18'b000001000000000100;
18'd181: data <= 18'b000001000100001000;
18'd182: data <= 18'b000001000001000100;
18'd183: data <= 18'b000001000011101001;
18'd184: data <= 18'b000000111110001001;
18'd185: data <= 18'b000001000011000101;
18'd186: data <= 18'b000000111101010110;
18'd187: data <= 18'b000000111011110111;
18'd188: data <= 18'b000001000000111010;
18'd189: data <= 18'b000001000010011110;
18'd190: data <= 18'b000001000001001111;
18'd191: data <= 18'b000000111100000001;
18'd192: data <= 18'b000000111101100011;
18'd193: data <= 18'b000000111100011001;
18'd194: data <= 18'b000000110110011011;
18'd195: data <= 18'b000000110111101010;
18'd196: data <= 18'b000000111100100010;
18'd197: data <= 18'b000000111000101001;
18'd198: data <= 18'b000000111110010001;
18'd199: data <= 18'b000000110111110000;
18'd200: data <= 18'b000001000011110100;
18'd201: data <= 18'b000000111000101111;
18'd202: data <= 18'b000000111100111110;
18'd203: data <= 18'b000000111001101110;
18'd204: data <= 18'b000000111010000010;
18'd205: data <= 18'b000000111011011011;
18'd206: data <= 18'b000000111110001101;
18'd207: data <= 18'b000000111110110001;
18'd208: data <= 18'b000000111010000101;
18'd209: data <= 18'b000001000000011001;
18'd210: data <= 18'b000000111101001011;
18'd211: data <= 18'b000001000011100010;
18'd212: data <= 18'b000000111011110100;
18'd213: data <= 18'b000000111111101111;
18'd214: data <= 18'b000001000000101111;
18'd215: data <= 18'b000000111001110001;
18'd216: data <= 18'b000000111010010001;
18'd217: data <= 18'b000001000010000010;
18'd218: data <= 18'b000001000101100010;
18'd219: data <= 18'b000001000100101101;
18'd220: data <= 18'b000000111111001100;
18'd221: data <= 18'b000001000100010110;
18'd222: data <= 18'b000001000111000011;
18'd223: data <= 18'b000000111111100111;
18'd224: data <= 18'b000000111101111010;
18'd225: data <= 18'b000001000000011110;
18'd226: data <= 18'b000001001000001100;
18'd227: data <= 18'b000001000000110000;
18'd228: data <= 18'b000001000010110011;
18'd229: data <= 18'b000000111100100001;
18'd230: data <= 18'b000001000000101100;
18'd231: data <= 18'b000000111111111110;
18'd232: data <= 18'b000001000010111111;
18'd233: data <= 18'b000000111111000000;
18'd234: data <= 18'b000000111110001010;
18'd235: data <= 18'b000001000001100101;
18'd236: data <= 18'b000001000110001010;
18'd237: data <= 18'b000001000100110010;
18'd238: data <= 18'b000001000001101111;
18'd239: data <= 18'b000000111111010111;
18'd240: data <= 18'b000001000101100110;
18'd241: data <= 18'b000001000001001001;
18'd242: data <= 18'b000001000001111111;
18'd243: data <= 18'b000001000000101011;
18'd244: data <= 18'b000001000010010010;
18'd245: data <= 18'b000000111111100001;
18'd246: data <= 18'b000001000111000011;
18'd247: data <= 18'b000001000110010111;
18'd248: data <= 18'b000000111110000100;
18'd249: data <= 18'b000001000100100010;
18'd250: data <= 18'b000000111111010111;
18'd251: data <= 18'b000001000000001111;
18'd252: data <= 18'b000000111100100111;
18'd253: data <= 18'b000000111101100010;
18'd254: data <= 18'b000001000010111110;
18'd255: data <= 18'b000001000011000110;
18'd256: data <= 18'b000000111111100000;
18'd257: data <= 18'b000000111001001110;
18'd258: data <= 18'b000000101000010010;
18'd259: data <= 18'b000000101101010001;
18'd260: data <= 18'b000000110011000000;
18'd261: data <= 18'b000000101110100101;
18'd262: data <= 18'b000000101000111010;
18'd263: data <= 18'b000000101010111100;
18'd264: data <= 18'b000000100011010001;
18'd265: data <= 18'b000000101001000010;
18'd266: data <= 18'b000000100110011111;
18'd267: data <= 18'b000000101001001101;
18'd268: data <= 18'b000000101100001001;
18'd269: data <= 18'b000000101001100001;
18'd270: data <= 18'b000000101100100100;
18'd271: data <= 18'b000000101100010011;
18'd272: data <= 18'b000000101000000101;
18'd273: data <= 18'b000000101001011011;
18'd274: data <= 18'b000000101101111110;
18'd275: data <= 18'b000000101011011111;
18'd276: data <= 18'b000000100111101011;
18'd277: data <= 18'b000000110001011100;
18'd278: data <= 18'b000000100100001000;
18'd279: data <= 18'b000000101001101010;
18'd280: data <= 18'b000000101111000101;
18'd281: data <= 18'b000000100111011000;
18'd282: data <= 18'b000000101000001011;
18'd283: data <= 18'b000000101010000011;
18'd284: data <= 18'b000000101011011000;
18'd285: data <= 18'b000000101011100110;
18'd286: data <= 18'b000000101000010111;
18'd287: data <= 18'b000000101010111010;
18'd288: data <= 18'b000000101010000011;
18'd289: data <= 18'b000000100111111011;
18'd290: data <= 18'b000000101100010111;
18'd291: data <= 18'b000000101100000001;
18'd292: data <= 18'b000000101010001110;
18'd293: data <= 18'b000000100101000001;
18'd294: data <= 18'b000000101010001010;
18'd295: data <= 18'b000000101101011011;
18'd296: data <= 18'b000000101110001001;
18'd297: data <= 18'b000000101000111110;
18'd298: data <= 18'b000000100111101000;
18'd299: data <= 18'b000000100110011111;
18'd300: data <= 18'b000000100010111110;
18'd301: data <= 18'b000000101000111111;
18'd302: data <= 18'b000000100111110010;
18'd303: data <= 18'b000000101010100111;
18'd304: data <= 18'b000000101101001101;
18'd305: data <= 18'b000000101101101011;
18'd306: data <= 18'b000000101100000001;
18'd307: data <= 18'b000000100110101101;
18'd308: data <= 18'b000000100111011011;
18'd309: data <= 18'b000000101101100111;
18'd310: data <= 18'b000000101110110100;
18'd311: data <= 18'b000000101010010110;
18'd312: data <= 18'b000000101011110101;
18'd313: data <= 18'b000000100100010111;
18'd314: data <= 18'b000000100111001001;
18'd315: data <= 18'b000000101101110110;
18'd316: data <= 18'b000000100100001001;
18'd317: data <= 18'b000000100011100000;
18'd318: data <= 18'b000000100101100000;
18'd319: data <= 18'b000000101000111111;
18'd320: data <= 18'b000000101010000000;
18'd321: data <= 18'b000000100110000010;
18'd322: data <= 18'b000000100010110000;
18'd323: data <= 18'b000000100011111000;
18'd324: data <= 18'b000000101010011100;
18'd325: data <= 18'b000000101011100001;
18'd326: data <= 18'b000000101010000001;
18'd327: data <= 18'b000000100001001010;
18'd328: data <= 18'b000000101011111010;
18'd329: data <= 18'b000000101001110101;
18'd330: data <= 18'b000000100101001111;
18'd331: data <= 18'b000000100110011110;
18'd332: data <= 18'b000000100110001101;
18'd333: data <= 18'b000000100111111110;
18'd334: data <= 18'b000000100010101000;
18'd335: data <= 18'b000000101000001110;
18'd336: data <= 18'b000000101001011000;
18'd337: data <= 18'b000000100110101100;
18'd338: data <= 18'b000000101100110010;
18'd339: data <= 18'b000000100111011001;
18'd340: data <= 18'b000000101000000011;
18'd341: data <= 18'b000000100111001011;
18'd342: data <= 18'b000000101001110100;
18'd343: data <= 18'b000000100111001001;
18'd344: data <= 18'b000000101110100010;
18'd345: data <= 18'b000000100010101101;
18'd346: data <= 18'b000000101101000111;
18'd347: data <= 18'b000000100110110110;
18'd348: data <= 18'b000000101100011010;
18'd349: data <= 18'b000000101100111111;
18'd350: data <= 18'b000000101011100000;
18'd351: data <= 18'b000000101010001001;
18'd352: data <= 18'b000000100111100001;
18'd353: data <= 18'b000000101110011100;
18'd354: data <= 18'b000000100110100111;
18'd355: data <= 18'b000000101010100111;
18'd356: data <= 18'b000000101110110000;
18'd357: data <= 18'b000000101100100110;
18'd358: data <= 18'b000000101000110010;
18'd359: data <= 18'b000000101101101010;
18'd360: data <= 18'b000000101101101001;
18'd361: data <= 18'b000000101000001010;
18'd362: data <= 18'b000000101010101110;
18'd363: data <= 18'b000000101101001110;
18'd364: data <= 18'b000000101100101111;
18'd365: data <= 18'b000000100110110100;
18'd366: data <= 18'b000000101010111111;
18'd367: data <= 18'b000000100111100011;
18'd368: data <= 18'b000000101101010111;
18'd369: data <= 18'b000000101011001101;
18'd370: data <= 18'b000000101101000010;
18'd371: data <= 18'b000000101011011101;
18'd372: data <= 18'b000000101010100110;
18'd373: data <= 18'b000000101111110100;
18'd374: data <= 18'b000000101000101110;
18'd375: data <= 18'b000000101011110000;
18'd376: data <= 18'b000000100110011111;
18'd377: data <= 18'b000000110000110001;
18'd378: data <= 18'b000000100010010001;
18'd379: data <= 18'b000000101000101011;
18'd380: data <= 18'b000000100111100000;
18'd381: data <= 18'b000000101001000101;
18'd382: data <= 18'b000000101001101110;
18'd383: data <= 18'b000000101001000010;
18'd384: data <= 18'b000000101011101110;
18'd385: data <= 18'b000000110001001010;
18'd386: data <= 18'b000000100000101110;

  

endcase
end
endmodule


module  rom_dense_q_class1  (clock,address, data);
input clock;
input [17:0] address;
output [17:0] data;
reg signed [17:0] data;

always@(posedge clock)
begin
    case(address)
    18'd000: data <= 18'b000000110110010011;
18'd001: data <= 18'b000000101111011000;
18'd002: data <= 18'b000000110011110100;
18'd003: data <= 18'b000000110111100101;
18'd004: data <= 18'b000000110111010011;
18'd005: data <= 18'b000000110011011000;
18'd006: data <= 18'b000000101011001011;
18'd007: data <= 18'b000000110110101011;
18'd008: data <= 18'b000000101111110111;
18'd009: data <= 18'b000000110100011111;
18'd010: data <= 18'b000000110001111011;
18'd011: data <= 18'b000000110011011111;
18'd012: data <= 18'b000000101110001001;
18'd013: data <= 18'b000000110101001111;
18'd014: data <= 18'b000000101111011101;
18'd015: data <= 18'b000000110010101011;
18'd016: data <= 18'b000000110110000001;
18'd017: data <= 18'b000000110001101111;
18'd018: data <= 18'b000000110001111100;
18'd019: data <= 18'b000000110000111011;
18'd020: data <= 18'b000000110100000100;
18'd021: data <= 18'b000000110010010011;
18'd022: data <= 18'b000000110000000110;
18'd023: data <= 18'b000000110111011100;
18'd024: data <= 18'b000000110101001001;
18'd025: data <= 18'b000000101110111001;
18'd026: data <= 18'b000000110101001101;
18'd027: data <= 18'b000000110100001011;
18'd028: data <= 18'b000000110110001000;
18'd029: data <= 18'b000000110000111100;
18'd030: data <= 18'b000000110010010100;
18'd031: data <= 18'b000000110011101011;
18'd032: data <= 18'b000000110010000001;
18'd033: data <= 18'b000000110000000110;
18'd034: data <= 18'b000000101100110101;
18'd035: data <= 18'b000000101010110001;
18'd036: data <= 18'b000000101110111010;
18'd037: data <= 18'b000000110000110100;
18'd038: data <= 18'b000000110000010100;
18'd039: data <= 18'b000000110010001111;
18'd040: data <= 18'b000000110100010011;
18'd041: data <= 18'b000000110100011111;
18'd042: data <= 18'b000000110001101011;
18'd043: data <= 18'b000000110101110110;
18'd044: data <= 18'b000000110010001111;
18'd045: data <= 18'b000000101110000001;
18'd046: data <= 18'b000000110001001001;
18'd047: data <= 18'b000000110110010111;
18'd048: data <= 18'b000000110000100111;
18'd049: data <= 18'b000000110000101101;
18'd050: data <= 18'b000000111000011000;
18'd051: data <= 18'b000000110010011011;
18'd052: data <= 18'b000000101100001011;
18'd053: data <= 18'b000000101111100001;
18'd054: data <= 18'b000000110101000010;
18'd055: data <= 18'b000000110000010011;
18'd056: data <= 18'b000000110110111010;
18'd057: data <= 18'b000000110100111100;
18'd058: data <= 18'b000000110111011110;
18'd059: data <= 18'b000000110111001100;
18'd060: data <= 18'b000000110000110111;
18'd061: data <= 18'b000000101111111101;
18'd062: data <= 18'b000000110011110110;
18'd063: data <= 18'b000000110110111010;
18'd064: data <= 18'b000000110010110001;
18'd065: data <= 18'b000000110111101011;
18'd066: data <= 18'b000000110011111001;
18'd067: data <= 18'b000000111000100100;
18'd068: data <= 18'b000000110110100011;
18'd069: data <= 18'b000000110111111110;
18'd070: data <= 18'b000000111001100000;
18'd071: data <= 18'b000000110100101000;
18'd072: data <= 18'b000000110000000101;
18'd073: data <= 18'b000000110010101100;
18'd074: data <= 18'b000000110000011000;
18'd075: data <= 18'b000000110100100110;
18'd076: data <= 18'b000000110010000001;
18'd077: data <= 18'b000000110010100000;
18'd078: data <= 18'b000000110100010000;
18'd079: data <= 18'b000000101101111101;
18'd080: data <= 18'b000000101110110010;
18'd081: data <= 18'b000000110001011111;
18'd082: data <= 18'b000000110001101001;
18'd083: data <= 18'b000000110001010100;
18'd084: data <= 18'b000000101100001011;
18'd085: data <= 18'b000000110100110000;
18'd086: data <= 18'b000000110000001001;
18'd087: data <= 18'b000000110110100011;
18'd088: data <= 18'b000000110101100000;
18'd089: data <= 18'b000000110011000011;
18'd090: data <= 18'b000000110111011001;
18'd091: data <= 18'b000000110110111000;
18'd092: data <= 18'b000000110100101011;
18'd093: data <= 18'b000000110010100011;
18'd094: data <= 18'b000000110111010011;
18'd095: data <= 18'b000000110101111101;
18'd096: data <= 18'b000000111000111010;
18'd097: data <= 18'b000000110011001101;
18'd098: data <= 18'b000000110000000111;
18'd099: data <= 18'b000000110110010000;
18'd100: data <= 18'b000000110000010001;
18'd101: data <= 18'b000000101101111110;
18'd102: data <= 18'b000000110110001111;
18'd103: data <= 18'b000000101111101001;
18'd104: data <= 18'b000000111000011101;
18'd105: data <= 18'b000000110101100011;
18'd106: data <= 18'b000000110011010000;
18'd107: data <= 18'b000000110001101101;
18'd108: data <= 18'b000000110101011110;
18'd109: data <= 18'b000000110011111110;
18'd110: data <= 18'b000000110011101010;
18'd111: data <= 18'b000000110111010001;
18'd112: data <= 18'b000000110000011000;
18'd113: data <= 18'b000000101100000001;
18'd114: data <= 18'b000000110110101001;
18'd115: data <= 18'b000000101101101000;
18'd116: data <= 18'b000000110000101110;
18'd117: data <= 18'b000000110110101101;
18'd118: data <= 18'b000000110010110111;
18'd119: data <= 18'b000000110001110111;
18'd120: data <= 18'b000000101111010101;
18'd121: data <= 18'b000000110101100010;
18'd122: data <= 18'b000000101101100110;
18'd123: data <= 18'b000000110110000011;
18'd124: data <= 18'b000000110100111111;
18'd125: data <= 18'b000000110011011011;
18'd126: data <= 18'b000000111011101011;
18'd127: data <= 18'b000000110111111011;
18'd128: data <= 18'b000000110000110010;
18'd129: data <= 18'b111111000001011100;
18'd130: data <= 18'b111110111110001010;
18'd131: data <= 18'b111110111101001110;
18'd132: data <= 18'b111110111101111000;
18'd133: data <= 18'b111111000100011101;
18'd134: data <= 18'b111111000000000001;
18'd135: data <= 18'b111111000110010100;
18'd136: data <= 18'b111111000010111100;
18'd137: data <= 18'b111111001010101010;
18'd138: data <= 18'b111110111111100100;
18'd139: data <= 18'b111111000100010111;
18'd140: data <= 18'b111111000100000010;
18'd141: data <= 18'b111111000110010011;
18'd142: data <= 18'b111111000001010010;
18'd143: data <= 18'b111110111111111101;
18'd144: data <= 18'b111111000011010000;
18'd145: data <= 18'b111110111111011101;
18'd146: data <= 18'b111111000000001001;
18'd147: data <= 18'b111110111111100111;
18'd148: data <= 18'b111111000001110110;
18'd149: data <= 18'b111110111101011010;
18'd150: data <= 18'b111111000010010110;
18'd151: data <= 18'b111110111111110001;
18'd152: data <= 18'b111111000011010100;
18'd153: data <= 18'b111110111111011011;
18'd154: data <= 18'b111111000010110111;
18'd155: data <= 18'b111111000000010011;
18'd156: data <= 18'b111110111010100001;
18'd157: data <= 18'b111111000110000010;
18'd158: data <= 18'b111110111110100111;
18'd159: data <= 18'b111110111111111011;
18'd160: data <= 18'b111111000100001000;
18'd161: data <= 18'b111111000000110111;
18'd162: data <= 18'b111111000011111110;
18'd163: data <= 18'b111111000011101111;
18'd164: data <= 18'b111110111101111001;
18'd165: data <= 18'b111110111101111011;
18'd166: data <= 18'b111111000000011111;
18'd167: data <= 18'b111111000101111101;
18'd168: data <= 18'b111110111110001000;
18'd169: data <= 18'b111111000001000011;
18'd170: data <= 18'b111111000000011010;
18'd171: data <= 18'b111111000100010111;
18'd172: data <= 18'b111110111101000101;
18'd173: data <= 18'b111111000100010011;
18'd174: data <= 18'b111110111100100000;
18'd175: data <= 18'b111110111111111100;
18'd176: data <= 18'b111111001000110001;
18'd177: data <= 18'b111111000100010011;
18'd178: data <= 18'b111111000000010100;
18'd179: data <= 18'b111111000010010011;
18'd180: data <= 18'b111111000101110101;
18'd181: data <= 18'b111110111111100001;
18'd182: data <= 18'b111111001001010001;
18'd183: data <= 18'b111111000101001000;
18'd184: data <= 18'b111111000111001000;
18'd185: data <= 18'b111110111101101001;
18'd186: data <= 18'b111111000001010011;
18'd187: data <= 18'b111111000010111111;
18'd188: data <= 18'b111111000100011001;
18'd189: data <= 18'b111110111111011110;
18'd190: data <= 18'b111111000111101100;
18'd191: data <= 18'b111111000001101101;
18'd192: data <= 18'b111111000111011000;
18'd193: data <= 18'b111111000110001100;
18'd194: data <= 18'b111111000100101110;
18'd195: data <= 18'b111111000000000111;
18'd196: data <= 18'b111111000001011001;
18'd197: data <= 18'b111111000010110010;
18'd198: data <= 18'b111111000010101010;
18'd199: data <= 18'b111110111111000000;
18'd200: data <= 18'b111111000011110010;
18'd201: data <= 18'b111111000011010100;
18'd202: data <= 18'b111111000000010101;
18'd203: data <= 18'b111111000010100100;
18'd204: data <= 18'b111111000001101010;
18'd205: data <= 18'b111111000101100011;
18'd206: data <= 18'b111111000010100011;
18'd207: data <= 18'b111111000101111100;
18'd208: data <= 18'b111111000010010000;
18'd209: data <= 18'b111111000011111010;
18'd210: data <= 18'b111110111111111010;
18'd211: data <= 18'b111111000110000000;
18'd212: data <= 18'b111111000101011101;
18'd213: data <= 18'b111111001000100101;
18'd214: data <= 18'b111110111100110110;
18'd215: data <= 18'b111111000001100000;
18'd216: data <= 18'b111111000100011100;
18'd217: data <= 18'b111111000010011110;
18'd218: data <= 18'b111111000001110000;
18'd219: data <= 18'b111111000011000101;
18'd220: data <= 18'b111110111101101000;
18'd221: data <= 18'b111111000100000100;
18'd222: data <= 18'b111110111101110000;
18'd223: data <= 18'b111110111110110001;
18'd224: data <= 18'b111111000001011110;
18'd225: data <= 18'b111111000101100110;
18'd226: data <= 18'b111111000001110000;
18'd227: data <= 18'b111111000001011111;
18'd228: data <= 18'b111111000011100010;
18'd229: data <= 18'b111111001000100110;
18'd230: data <= 18'b111111000110110101;
18'd231: data <= 18'b111111000010010100;
18'd232: data <= 18'b111111000010111001;
18'd233: data <= 18'b111111000101101011;
18'd234: data <= 18'b111111000001010011;
18'd235: data <= 18'b111110111110010001;
18'd236: data <= 18'b111110111110111101;
18'd237: data <= 18'b111110111110111000;
18'd238: data <= 18'b111111000000010110;
18'd239: data <= 18'b111111000010011111;
18'd240: data <= 18'b111111000011001100;
18'd241: data <= 18'b111110111101111000;
18'd242: data <= 18'b111110111011100010;
18'd243: data <= 18'b111111000000010101;
18'd244: data <= 18'b111111000001110000;
18'd245: data <= 18'b111110111111100111;
18'd246: data <= 18'b111110111100010000;
18'd247: data <= 18'b111111000110101000;
18'd248: data <= 18'b111110111110101011;
18'd249: data <= 18'b111111000111100001;
18'd250: data <= 18'b111111000000010110;
18'd251: data <= 18'b111111000010001101;
18'd252: data <= 18'b111110111000100010;
18'd253: data <= 18'b111110111001010100;
18'd254: data <= 18'b111110111101011100;
18'd255: data <= 18'b111110111010100011;
18'd256: data <= 18'b111110111111110010;
18'd257: data <= 18'b111111001000000101;
18'd258: data <= 18'b111111011111000110;
18'd259: data <= 18'b111111001001101111;
18'd260: data <= 18'b111111001011100011;
18'd261: data <= 18'b111111010011001011;
18'd262: data <= 18'b111111001111011011;
18'd263: data <= 18'b111111001101101010;
18'd264: data <= 18'b111111001110111110;
18'd265: data <= 18'b111111010011010111;
18'd266: data <= 18'b111111010010011000;
18'd267: data <= 18'b111111010000100010;
18'd268: data <= 18'b111111001010111100;
18'd269: data <= 18'b111111001100011111;
18'd270: data <= 18'b111111001101111101;
18'd271: data <= 18'b111111001111110011;
18'd272: data <= 18'b111111001110101011;
18'd273: data <= 18'b111111010010101011;
18'd274: data <= 18'b111111010011101111;
18'd275: data <= 18'b111111010011010100;
18'd276: data <= 18'b111111001110001111;
18'd277: data <= 18'b111111001011011000;
18'd278: data <= 18'b111111001100000011;
18'd279: data <= 18'b111111001010110011;
18'd280: data <= 18'b111111010001010111;
18'd281: data <= 18'b111111010011110110;
18'd282: data <= 18'b111111010000111111;
18'd283: data <= 18'b111111010010100100;
18'd284: data <= 18'b111111001010100011;
18'd285: data <= 18'b111111010000110111;
18'd286: data <= 18'b111111001101001001;
18'd287: data <= 18'b111111010010010110;
18'd288: data <= 18'b111111010101110100;
18'd289: data <= 18'b111111001011111001;
18'd290: data <= 18'b111111010001100011;
18'd291: data <= 18'b111111010001101101;
18'd292: data <= 18'b111111001101000111;
18'd293: data <= 18'b111111010001100100;
18'd294: data <= 18'b111111001100000101;
18'd295: data <= 18'b111111001111111000;
18'd296: data <= 18'b111111001111100100;
18'd297: data <= 18'b111111001010010110;
18'd298: data <= 18'b111111001100011111;
18'd299: data <= 18'b111111001101001010;
18'd300: data <= 18'b111111001011100111;
18'd301: data <= 18'b111111001011100110;
18'd302: data <= 18'b111111001011111011;
18'd303: data <= 18'b111111001110100001;
18'd304: data <= 18'b111111010010100101;
18'd305: data <= 18'b111111010100111110;
18'd306: data <= 18'b111111010000110111;
18'd307: data <= 18'b111111001110100010;
18'd308: data <= 18'b111111010010100001;
18'd309: data <= 18'b111111010111001001;
18'd310: data <= 18'b111111010100001000;
18'd311: data <= 18'b111111001111011011;
18'd312: data <= 18'b111111010111001100;
18'd313: data <= 18'b111111010011010110;
18'd314: data <= 18'b111111001111101001;
18'd315: data <= 18'b111111010101010111;
18'd316: data <= 18'b111111010010000000;
18'd317: data <= 18'b111111010101110110;
18'd318: data <= 18'b111111001111110110;
18'd319: data <= 18'b111111010100110101;
18'd320: data <= 18'b111111010001001110;
18'd321: data <= 18'b111111001110101101;
18'd322: data <= 18'b111111010010001100;
18'd323: data <= 18'b111111001001100100;
18'd324: data <= 18'b111111001100110101;
18'd325: data <= 18'b111111010010001000;
18'd326: data <= 18'b111111001111101101;
18'd327: data <= 18'b111111010011111111;
18'd328: data <= 18'b111111001101111110;
18'd329: data <= 18'b111111001101111010;
18'd330: data <= 18'b111111010001010000;
18'd331: data <= 18'b111111001111111000;
18'd332: data <= 18'b111111001100000000;
18'd333: data <= 18'b111111010001100100;
18'd334: data <= 18'b111111001111001001;
18'd335: data <= 18'b111111001111010011;
18'd336: data <= 18'b111111001111001000;
18'd337: data <= 18'b111111010011011110;
18'd338: data <= 18'b111111001101101100;
18'd339: data <= 18'b111111010000011001;
18'd340: data <= 18'b111111010001001001;
18'd341: data <= 18'b111111010001011001;
18'd342: data <= 18'b111111011000001001;
18'd343: data <= 18'b111111001111111000;
18'd344: data <= 18'b111111001110000111;
18'd345: data <= 18'b111111010101000110;
18'd346: data <= 18'b111111001101011011;
18'd347: data <= 18'b111111010000110111;
18'd348: data <= 18'b111111010000011001;
18'd349: data <= 18'b111111001101011001;
18'd350: data <= 18'b111111001001100101;
18'd351: data <= 18'b111111010011010100;
18'd352: data <= 18'b111111001000010101;
18'd353: data <= 18'b111111001111111111;
18'd354: data <= 18'b111111001110000101;
18'd355: data <= 18'b111111001100110010;
18'd356: data <= 18'b111111010000110001;
18'd357: data <= 18'b111111001100001100;
18'd358: data <= 18'b111111001111110111;
18'd359: data <= 18'b111111001100010110;
18'd360: data <= 18'b111111010001010101;
18'd361: data <= 18'b111111010101111001;
18'd362: data <= 18'b111111001111001000;
18'd363: data <= 18'b111111001110001110;
18'd364: data <= 18'b111111010001100011;
18'd365: data <= 18'b111111001100111010;
18'd366: data <= 18'b111111001010110111;
18'd367: data <= 18'b111111010010001001;
18'd368: data <= 18'b111111010011011101;
18'd369: data <= 18'b111111010001000110;
18'd370: data <= 18'b111111010001111011;
18'd371: data <= 18'b111111010000001110;
18'd372: data <= 18'b111111001100010101;
18'd373: data <= 18'b111111001001110000;
18'd374: data <= 18'b111111000111101010;
18'd375: data <= 18'b111111010001010000;
18'd376: data <= 18'b111111010110001000;
18'd377: data <= 18'b111111001101001000;
18'd378: data <= 18'b111111001010111010;
18'd379: data <= 18'b111111001100001010;
18'd380: data <= 18'b111111001101101000;
18'd381: data <= 18'b111111001101010110;
18'd382: data <= 18'b111111001110010000;
18'd383: data <= 18'b111111000101000101;
18'd384: data <= 18'b111111001100110011;
18'd385: data <= 18'b111111010000101100;
18'd386: data <= 18'b111111100010001011;
 
endcase
end
endmodule

module  rom_dense_i_class2  (clock, address, data);
input clock;
input [17:0] address;
output [17:0] data;
reg signed [17:0] data;

always@(posedge clock)
begin
    case(address)
    18'd000: data <= 18'b111111111110110100;
18'd001: data <= 18'b111111101100001000;
18'd002: data <= 18'b111111100000110111;
18'd003: data <= 18'b111111011011001000;
18'd004: data <= 18'b111111100111110010;
18'd005: data <= 18'b111111101010100011;
18'd006: data <= 18'b111111100000011011;
18'd007: data <= 18'b111111101000010111;
18'd008: data <= 18'b111111100001011110;
18'd009: data <= 18'b111111011100011111;
18'd010: data <= 18'b111111100001111011;
18'd011: data <= 18'b111111100001101010;
18'd012: data <= 18'b111111011100111000;
18'd013: data <= 18'b111111011110010100;
18'd014: data <= 18'b111111011101010010;
18'd015: data <= 18'b111111100000000100;
18'd016: data <= 18'b111111101001000100;
18'd017: data <= 18'b111111100100000110;
18'd018: data <= 18'b111111011011111001;
18'd019: data <= 18'b111111100001011100;
18'd020: data <= 18'b111111011110000110;
18'd021: data <= 18'b111111011011011101;
18'd022: data <= 18'b111111011110000101;
18'd023: data <= 18'b111111101001010001;
18'd024: data <= 18'b111111100101111101;
18'd025: data <= 18'b111111100100011011;
18'd026: data <= 18'b111111011101101011;
18'd027: data <= 18'b111111100001100000;
18'd028: data <= 18'b111111100101011010;
18'd029: data <= 18'b111111100101011110;
18'd030: data <= 18'b111111011111001000;
18'd031: data <= 18'b111111100100000011;
18'd032: data <= 18'b111111100111010010;
18'd033: data <= 18'b111111011111101001;
18'd034: data <= 18'b111111011001111111;
18'd035: data <= 18'b111111101100010111;
18'd036: data <= 18'b111111101100000010;
18'd037: data <= 18'b111111011100010011;
18'd038: data <= 18'b111111100011100000;
18'd039: data <= 18'b111111101110011001;
18'd040: data <= 18'b111111100111111111;
18'd041: data <= 18'b111111100010001101;
18'd042: data <= 18'b111111100100000111;
18'd043: data <= 18'b111111100010011000;
18'd044: data <= 18'b111111011011100001;
18'd045: data <= 18'b111111011111010000;
18'd046: data <= 18'b111111100011110111;
18'd047: data <= 18'b111111100010011101;
18'd048: data <= 18'b111111100101011101;
18'd049: data <= 18'b111111100001111111;
18'd050: data <= 18'b111111100000110011;
18'd051: data <= 18'b111111100100101101;
18'd052: data <= 18'b111111100000010011;
18'd053: data <= 18'b111111100101011000;
18'd054: data <= 18'b111111100100111011;
18'd055: data <= 18'b111111011110001101;
18'd056: data <= 18'b111111101000110011;
18'd057: data <= 18'b111111101011011010;
18'd058: data <= 18'b111111100001100010;
18'd059: data <= 18'b111111100011001010;
18'd060: data <= 18'b111111110000001010;
18'd061: data <= 18'b111111100110111001;
18'd062: data <= 18'b111111101110000100;
18'd063: data <= 18'b111111101000101010;
18'd064: data <= 18'b111111101000110111;
18'd065: data <= 18'b111111100100100010;
18'd066: data <= 18'b111111101010000110;
18'd067: data <= 18'b111111101010011100;
18'd068: data <= 18'b111111101000011010;
18'd069: data <= 18'b111111100011111100;
18'd070: data <= 18'b111111100010010000;
18'd071: data <= 18'b111111100101010111;
18'd072: data <= 18'b111111100001100101;
18'd073: data <= 18'b111111100011011100;
18'd074: data <= 18'b111111110011000011;
18'd075: data <= 18'b111111101001010011;
18'd076: data <= 18'b111111100000101011;
18'd077: data <= 18'b111111011100010000;
18'd078: data <= 18'b111111011101110101;
18'd079: data <= 18'b111111100000001000;
18'd080: data <= 18'b111111100011110000;
18'd081: data <= 18'b111111100100111011;
18'd082: data <= 18'b111111100111101100;
18'd083: data <= 18'b111111100111110111;
18'd084: data <= 18'b111111011100111010;
18'd085: data <= 18'b111111100111011010;
18'd086: data <= 18'b111111101011100110;
18'd087: data <= 18'b111111011011110001;
18'd088: data <= 18'b111111011010111001;
18'd089: data <= 18'b111111011000001101;
18'd090: data <= 18'b111111011111101010;
18'd091: data <= 18'b111111101011010011;
18'd092: data <= 18'b111111100000011101;
18'd093: data <= 18'b111111100100011001;
18'd094: data <= 18'b111111101001000100;
18'd095: data <= 18'b111111100011110001;
18'd096: data <= 18'b111111011111000011;
18'd097: data <= 18'b111111101111011001;
18'd098: data <= 18'b111111101010011111;
18'd099: data <= 18'b111111011010010111;
18'd100: data <= 18'b111111011110010000;
18'd101: data <= 18'b111111100011101101;
18'd102: data <= 18'b111111100010000111;
18'd103: data <= 18'b111111100011110110;
18'd104: data <= 18'b111111100000011101;
18'd105: data <= 18'b111111100110110010;
18'd106: data <= 18'b111111100101110001;
18'd107: data <= 18'b111111100011000000;
18'd108: data <= 18'b111111100010010100;
18'd109: data <= 18'b111111100010011000;
18'd110: data <= 18'b111111100001100101;
18'd111: data <= 18'b111111101010100000;
18'd112: data <= 18'b111111100010111011;
18'd113: data <= 18'b111111100000111000;
18'd114: data <= 18'b111111100100101000;
18'd115: data <= 18'b111111011010001001;
18'd116: data <= 18'b111111100101111100;
18'd117: data <= 18'b111111101010101000;
18'd118: data <= 18'b111111101100111011;
18'd119: data <= 18'b111111101110111110;
18'd120: data <= 18'b111111100111110001;
18'd121: data <= 18'b111111100101100010;
18'd122: data <= 18'b111111100110001110;
18'd123: data <= 18'b111111100011000000;
18'd124: data <= 18'b111111100101100100;
18'd125: data <= 18'b111111101000110111;
18'd126: data <= 18'b111111101000001011;
18'd127: data <= 18'b111111110011000011;
18'd128: data <= 18'b111111111010110000;
18'd129: data <= 18'b000000011000101101;
18'd130: data <= 18'b111111110110110100;
18'd131: data <= 18'b111111111001010000;
18'd132: data <= 18'b000000000100001110;
18'd133: data <= 18'b111111110100100011;
18'd134: data <= 18'b111111111100000010;
18'd135: data <= 18'b111111111110000101;
18'd136: data <= 18'b111111111000000001;
18'd137: data <= 18'b111111111111101000;
18'd138: data <= 18'b000000000001101111;
18'd139: data <= 18'b000000000110001000;
18'd140: data <= 18'b000000000101110000;
18'd141: data <= 18'b000000000100101010;
18'd142: data <= 18'b111111111110100101;
18'd143: data <= 18'b000000000101001001;
18'd144: data <= 18'b000000001100010100;
18'd145: data <= 18'b111111111010111100;
18'd146: data <= 18'b111111111001111101;
18'd147: data <= 18'b111111111110101101;
18'd148: data <= 18'b000000000110001000;
18'd149: data <= 18'b111111111101110110;
18'd150: data <= 18'b000000000011011111;
18'd151: data <= 18'b111111111010110111;
18'd152: data <= 18'b111111110100111100;
18'd153: data <= 18'b111111110011000101;
18'd154: data <= 18'b111111111111101001;
18'd155: data <= 18'b000000000110111000;
18'd156: data <= 18'b000000001100000110;
18'd157: data <= 18'b111111110110110111;
18'd158: data <= 18'b111111111110101110;
18'd159: data <= 18'b111111110110001010;
18'd160: data <= 18'b111111111010101000;
18'd161: data <= 18'b111111110101011001;
18'd162: data <= 18'b000000000110010010;
18'd163: data <= 18'b000000010001111101;
18'd164: data <= 18'b000000000100011111;
18'd165: data <= 18'b111111101011001001;
18'd166: data <= 18'b000000000001110010;
18'd167: data <= 18'b111111111011110001;
18'd168: data <= 18'b111111110100000000;
18'd169: data <= 18'b111111111011000111;
18'd170: data <= 18'b000000000001111011;
18'd171: data <= 18'b000000000101000001;
18'd172: data <= 18'b000000000001111001;
18'd173: data <= 18'b000000000010111101;
18'd174: data <= 18'b000000000000111000;
18'd175: data <= 18'b111111111101001000;
18'd176: data <= 18'b111111110010100101;
18'd177: data <= 18'b111111110101011100;
18'd178: data <= 18'b111111111101001110;
18'd179: data <= 18'b000000000011001110;
18'd180: data <= 18'b000000000011010110;
18'd181: data <= 18'b111111111111100001;
18'd182: data <= 18'b000000000101100111;
18'd183: data <= 18'b111111111000000111;
18'd184: data <= 18'b000000000100010111;
18'd185: data <= 18'b111111101011100100;
18'd186: data <= 18'b111111101100000010;
18'd187: data <= 18'b111111111101000101;
18'd188: data <= 18'b000000000000100000;
18'd189: data <= 18'b111111110000100010;
18'd190: data <= 18'b111111111101100111;
18'd191: data <= 18'b111111101011110000;
18'd192: data <= 18'b111111111011000001;
18'd193: data <= 18'b111111111010111100;
18'd194: data <= 18'b000000001001010011;
18'd195: data <= 18'b111111111011011110;
18'd196: data <= 18'b111111110010101100;
18'd197: data <= 18'b111111110111111010;
18'd198: data <= 18'b000000000010001001;
18'd199: data <= 18'b111111111011011100;
18'd200: data <= 18'b111111110011101011;
18'd201: data <= 18'b000000000011000000;
18'd202: data <= 18'b111111111100100010;
18'd203: data <= 18'b111111101110000010;
18'd204: data <= 18'b111111110111110001;
18'd205: data <= 18'b000000000011111110;
18'd206: data <= 18'b000000000100001001;
18'd207: data <= 18'b111111111011001001;
18'd208: data <= 18'b000000001001100110;
18'd209: data <= 18'b000000000110000011;
18'd210: data <= 18'b000000000110010100;
18'd211: data <= 18'b111111111011001000;
18'd212: data <= 18'b111111110110111010;
18'd213: data <= 18'b111111111111001110;
18'd214: data <= 18'b111111111111011011;
18'd215: data <= 18'b111111110001000001;
18'd216: data <= 18'b111111111010110100;
18'd217: data <= 18'b000000010001010010;
18'd218: data <= 18'b000000010000000010;
18'd219: data <= 18'b000000001101010011;
18'd220: data <= 18'b111111111001111110;
18'd221: data <= 18'b111111111110110011;
18'd222: data <= 18'b111111110011101101;
18'd223: data <= 18'b111111111011010111;
18'd224: data <= 18'b000000000010111011;
18'd225: data <= 18'b000000000000100111;
18'd226: data <= 18'b111111111000001100;
18'd227: data <= 18'b111111101011011011;
18'd228: data <= 18'b000000000101111100;
18'd229: data <= 18'b000000001110101100;
18'd230: data <= 18'b111111111100100010;
18'd231: data <= 18'b000000000000010111;
18'd232: data <= 18'b111111110010101101;
18'd233: data <= 18'b111111111111010011;
18'd234: data <= 18'b111111111011001100;
18'd235: data <= 18'b111111111111011100;
18'd236: data <= 18'b000000000101000100;
18'd237: data <= 18'b111111111100111110;
18'd238: data <= 18'b111111111100111101;
18'd239: data <= 18'b000000000101000001;
18'd240: data <= 18'b111111110111101110;
18'd241: data <= 18'b000000000010110000;
18'd242: data <= 18'b000000000110011101;
18'd243: data <= 18'b000000000011011001;
18'd244: data <= 18'b000000010011011101;
18'd245: data <= 18'b111111111101101110;
18'd246: data <= 18'b000000000010101101;
18'd247: data <= 18'b111111110111010111;
18'd248: data <= 18'b111111101110110001;
18'd249: data <= 18'b111111110011000011;
18'd250: data <= 18'b111111101110001010;
18'd251: data <= 18'b111111110011101011;
18'd252: data <= 18'b111111111101100110;
18'd253: data <= 18'b111111111110100111;
18'd254: data <= 18'b111111111101100111;
18'd255: data <= 18'b111111111110101011;
18'd256: data <= 18'b111111101000010010;
18'd257: data <= 18'b000000101010110010;
18'd258: data <= 18'b000000001011111110;
18'd259: data <= 18'b000000001001100010;
18'd260: data <= 18'b000000001110011100;
18'd261: data <= 18'b000000010000001111;
18'd262: data <= 18'b000000001011111100;
18'd263: data <= 18'b000000001110100011;
18'd264: data <= 18'b000000010010011111;
18'd265: data <= 18'b000000010100001100;
18'd266: data <= 18'b000000001010010101;
18'd267: data <= 18'b000000010000111110;
18'd268: data <= 18'b000000010000100101;
18'd269: data <= 18'b000000010101000101;
18'd270: data <= 18'b000000010110001101;
18'd271: data <= 18'b000000001110010101;
18'd272: data <= 18'b000000010010001001;
18'd273: data <= 18'b000000010000001101;
18'd274: data <= 18'b000000010010100110;
18'd275: data <= 18'b000000001100010101;
18'd276: data <= 18'b000000010110001111;
18'd277: data <= 18'b000000001110111001;
18'd278: data <= 18'b000000010100110000;
18'd279: data <= 18'b000000010011111101;
18'd280: data <= 18'b000000010001110010;
18'd281: data <= 18'b000000000101011011;
18'd282: data <= 18'b000000001101001011;
18'd283: data <= 18'b000000010001100011;
18'd284: data <= 18'b000000010001001011;
18'd285: data <= 18'b000000010010010111;
18'd286: data <= 18'b000000010000111000;
18'd287: data <= 18'b000000010010001100;
18'd288: data <= 18'b000000010010100001;
18'd289: data <= 18'b000000001011001110;
18'd290: data <= 18'b000000010110000101;
18'd291: data <= 18'b000000010110111001;
18'd292: data <= 18'b000000010011111101;
18'd293: data <= 18'b000000010000101001;
18'd294: data <= 18'b000000001101111110;
18'd295: data <= 18'b000000010000011000;
18'd296: data <= 18'b000000001101111100;
18'd297: data <= 18'b000000010001100011;
18'd298: data <= 18'b000000010010100111;
18'd299: data <= 18'b000000010111100000;
18'd300: data <= 18'b000000010100001101;
18'd301: data <= 18'b000000001110010100;
18'd302: data <= 18'b000000010011001010;
18'd303: data <= 18'b000000001111001111;
18'd304: data <= 18'b000000010011010101;
18'd305: data <= 18'b000000001100000001;
18'd306: data <= 18'b000000001110010000;
18'd307: data <= 18'b000000010011001111;
18'd308: data <= 18'b000000010011011011;
18'd309: data <= 18'b000000010000001000;
18'd310: data <= 18'b000000010001001100;
18'd311: data <= 18'b000000010011000011;
18'd312: data <= 18'b000000010011011000;
18'd313: data <= 18'b000000010100111101;
18'd314: data <= 18'b000000001001010011;
18'd315: data <= 18'b000000001101110101;
18'd316: data <= 18'b000000010111110001;
18'd317: data <= 18'b000000001010001101;
18'd318: data <= 18'b000000001010010001;
18'd319: data <= 18'b000000000110100000;
18'd320: data <= 18'b000000010001001110;
18'd321: data <= 18'b000000001100011000;
18'd322: data <= 18'b000000001111110100;
18'd323: data <= 18'b000000010010010010;
18'd324: data <= 18'b000000010010001011;
18'd325: data <= 18'b000000001010100010;
18'd326: data <= 18'b000000001101000011;
18'd327: data <= 18'b000000010001000010;
18'd328: data <= 18'b000000001111101100;
18'd329: data <= 18'b000000001010101001;
18'd330: data <= 18'b000000010000000011;
18'd331: data <= 18'b000000001011110111;
18'd332: data <= 18'b000000001010110010;
18'd333: data <= 18'b000000001101011111;
18'd334: data <= 18'b000000010100000111;
18'd335: data <= 18'b000000001110001001;
18'd336: data <= 18'b000000010000101011;
18'd337: data <= 18'b000000010001100010;
18'd338: data <= 18'b000000010001011100;
18'd339: data <= 18'b000000001111110110;
18'd340: data <= 18'b000000000110000100;
18'd341: data <= 18'b000000010010100100;
18'd342: data <= 18'b000000010111010011;
18'd343: data <= 18'b000000001001110000;
18'd344: data <= 18'b000000000110100010;
18'd345: data <= 18'b000000010011111011;
18'd346: data <= 18'b000000010001111111;
18'd347: data <= 18'b000000010011000001;
18'd348: data <= 18'b000000001110111111;
18'd349: data <= 18'b000000010000001000;
18'd350: data <= 18'b000000001001101011;
18'd351: data <= 18'b000000010011000111;
18'd352: data <= 18'b000000001101101001;
18'd353: data <= 18'b000000001100101001;
18'd354: data <= 18'b000000010000011101;
18'd355: data <= 18'b000000001100010011;
18'd356: data <= 18'b000000010011000110;
18'd357: data <= 18'b000000010011100111;
18'd358: data <= 18'b000000001111111111;
18'd359: data <= 18'b000000010010011001;
18'd360: data <= 18'b000000010001001010;
18'd361: data <= 18'b000000001111001110;
18'd362: data <= 18'b000000010000010010;
18'd363: data <= 18'b000000001101001110;
18'd364: data <= 18'b000000001101111000;
18'd365: data <= 18'b000000010000111101;
18'd366: data <= 18'b000000001011000011;
18'd367: data <= 18'b000000001110001110;
18'd368: data <= 18'b000000001010000001;
18'd369: data <= 18'b000000001111101111;
18'd370: data <= 18'b000000010101000011;
18'd371: data <= 18'b000000001101011000;
18'd372: data <= 18'b000000010000001011;
18'd373: data <= 18'b000000010100101101;
18'd374: data <= 18'b000000001111110111;
18'd375: data <= 18'b000000001110011100;
18'd376: data <= 18'b000000001011010101;
18'd377: data <= 18'b000000001011000010;
18'd378: data <= 18'b000000000110010101;
18'd379: data <= 18'b000000000110000100;
18'd380: data <= 18'b000000001101101101;
18'd381: data <= 18'b000000010011111111;
18'd382: data <= 18'b000000001100100011;
18'd383: data <= 18'b000000010001110100;
18'd384: data <= 18'b000000001011110001;
18'd385: data <= 18'b000000001111011000;
18'd386: data <= 18'b000000001111000010;

   

endcase
end
endmodule

module  rom_dense_q_class2  (clock, address, data);
input clock;
input [17:0] address;
output [17:0] data;
reg signed [17:0] data;

always@(posedge clock)
begin
    case(address)
    18'd000: data <= 18'b111111111010100001;
18'd001: data <= 18'b111111100111110010;
18'd002: data <= 18'b111111100010011010;
18'd003: data <= 18'b111111101010001011;
18'd004: data <= 18'b111111101110010110;
18'd005: data <= 18'b111111100110001010;
18'd006: data <= 18'b111111011110011110;
18'd007: data <= 18'b111111011111100011;
18'd008: data <= 18'b111111100010110110;
18'd009: data <= 18'b111111101011000101;
18'd010: data <= 18'b111111100011110101;
18'd011: data <= 18'b111111101001111110;
18'd012: data <= 18'b111111100101100000;
18'd013: data <= 18'b111111101011010000;
18'd014: data <= 18'b111111100111110111;
18'd015: data <= 18'b111111101010001000;
18'd016: data <= 18'b111111100110001000;
18'd017: data <= 18'b111111101111000110;
18'd018: data <= 18'b111111100101011001;
18'd019: data <= 18'b111111101010110111;
18'd020: data <= 18'b111111110000001000;
18'd021: data <= 18'b111111101110111000;
18'd022: data <= 18'b111111110000101000;
18'd023: data <= 18'b111111101100010000;
18'd024: data <= 18'b111111101100010111;
18'd025: data <= 18'b111111101101100100;
18'd026: data <= 18'b111111101001101100;
18'd027: data <= 18'b111111100010011100;
18'd028: data <= 18'b111111101000010110;
18'd029: data <= 18'b111111101010011101;
18'd030: data <= 18'b111111101011101011;
18'd031: data <= 18'b111111100111100000;
18'd032: data <= 18'b111111100100110001;
18'd033: data <= 18'b111111100111101010;
18'd034: data <= 18'b111111101010110011;
18'd035: data <= 18'b111111101101100110;
18'd036: data <= 18'b111111100000011001;
18'd037: data <= 18'b111111100011101110;
18'd038: data <= 18'b111111101011001101;
18'd039: data <= 18'b111111011110101110;
18'd040: data <= 18'b111111100100100010;
18'd041: data <= 18'b111111101100100010;
18'd042: data <= 18'b111111101101000100;
18'd043: data <= 18'b111111100011110010;
18'd044: data <= 18'b111111100010000111;
18'd045: data <= 18'b111111100000011001;
18'd046: data <= 18'b111111100110111110;
18'd047: data <= 18'b111111100000011101;
18'd048: data <= 18'b111111101001111110;
18'd049: data <= 18'b111111100010111101;
18'd050: data <= 18'b111111100100110001;
18'd051: data <= 18'b111111100100100001;
18'd052: data <= 18'b111111101000001110;
18'd053: data <= 18'b111111101110111001;
18'd054: data <= 18'b111111101001110001;
18'd055: data <= 18'b111111100010111011;
18'd056: data <= 18'b111111100010010100;
18'd057: data <= 18'b111111101110010111;
18'd058: data <= 18'b111111100011011110;
18'd059: data <= 18'b111111100100001100;
18'd060: data <= 18'b111111100110011010;
18'd061: data <= 18'b111111101011001011;
18'd062: data <= 18'b111111101011101100;
18'd063: data <= 18'b111111100100010000;
18'd064: data <= 18'b111111101100010011;
18'd065: data <= 18'b111111101000111001;
18'd066: data <= 18'b111111101110000100;
18'd067: data <= 18'b111111100000111111;
18'd068: data <= 18'b111111101000001000;
18'd069: data <= 18'b111111100101101001;
18'd070: data <= 18'b111111100000010100;
18'd071: data <= 18'b111111011110001110;
18'd072: data <= 18'b111111100000100010;
18'd073: data <= 18'b111111100010000010;
18'd074: data <= 18'b111111100100101110;
18'd075: data <= 18'b111111100001001100;
18'd076: data <= 18'b111111101001001110;
18'd077: data <= 18'b111111100111100100;
18'd078: data <= 18'b111111100010010000;
18'd079: data <= 18'b111111101101001100;
18'd080: data <= 18'b111111101000001110;
18'd081: data <= 18'b111111101100100111;
18'd082: data <= 18'b111111100010010100;
18'd083: data <= 18'b111111101101110111;
18'd084: data <= 18'b111111100110011101;
18'd085: data <= 18'b111111101100001011;
18'd086: data <= 18'b111111100110001001;
18'd087: data <= 18'b111111101110010000;
18'd088: data <= 18'b111111101010100010;
18'd089: data <= 18'b111111100000100111;
18'd090: data <= 18'b111111100110001001;
18'd091: data <= 18'b111111100011001111;
18'd092: data <= 18'b111111100111001001;
18'd093: data <= 18'b111111100110100001;
18'd094: data <= 18'b111111011111001111;
18'd095: data <= 18'b111111100111111011;
18'd096: data <= 18'b111111101100110100;
18'd097: data <= 18'b111111101010011100;
18'd098: data <= 18'b111111101110001100;
18'd099: data <= 18'b111111101111011100;
18'd100: data <= 18'b111111101001011110;
18'd101: data <= 18'b111111100001101100;
18'd102: data <= 18'b111111101111010011;
18'd103: data <= 18'b111111110000100010;
18'd104: data <= 18'b111111100011000100;
18'd105: data <= 18'b111111100100100100;
18'd106: data <= 18'b111111100011110000;
18'd107: data <= 18'b111111100110001011;
18'd108: data <= 18'b111111100111011110;
18'd109: data <= 18'b111111101000111111;
18'd110: data <= 18'b111111101000001110;
18'd111: data <= 18'b111111100010100111;
18'd112: data <= 18'b111111100101110101;
18'd113: data <= 18'b111111100110100100;
18'd114: data <= 18'b111111100011100101;
18'd115: data <= 18'b111111101101100110;
18'd116: data <= 18'b111111101101100011;
18'd117: data <= 18'b111111101000110100;
18'd118: data <= 18'b111111101101100101;
18'd119: data <= 18'b111111100010011111;
18'd120: data <= 18'b111111011110101011;
18'd121: data <= 18'b111111100101101100;
18'd122: data <= 18'b111111100101110010;
18'd123: data <= 18'b111111101110101101;
18'd124: data <= 18'b111111101110100011;
18'd125: data <= 18'b111111100101101010;
18'd126: data <= 18'b111111011110110101;
18'd127: data <= 18'b111111011100010100;
18'd128: data <= 18'b111111111100101011;
18'd129: data <= 18'b000000001011001100;
18'd130: data <= 18'b111111011110101010;
18'd131: data <= 18'b111111100101111001;
18'd132: data <= 18'b111111011011001010;
18'd133: data <= 18'b111111100000010110;
18'd134: data <= 18'b111111100000000110;
18'd135: data <= 18'b111111101001100001;
18'd136: data <= 18'b111111100101111010;
18'd137: data <= 18'b111111101001101110;
18'd138: data <= 18'b111111011110111101;
18'd139: data <= 18'b111111100011111110;
18'd140: data <= 18'b111111101010110101;
18'd141: data <= 18'b111111100010000100;
18'd142: data <= 18'b111111011110010010;
18'd143: data <= 18'b111111011110110111;
18'd144: data <= 18'b111111100000100110;
18'd145: data <= 18'b111111100000100000;
18'd146: data <= 18'b111111010100110110;
18'd147: data <= 18'b111111100011101001;
18'd148: data <= 18'b111111011111111010;
18'd149: data <= 18'b111111011010100001;
18'd150: data <= 18'b111111010111100000;
18'd151: data <= 18'b111111011111000000;
18'd152: data <= 18'b111111010111100110;
18'd153: data <= 18'b111111010111101000;
18'd154: data <= 18'b111111010101010100;
18'd155: data <= 18'b111111011010001011;
18'd156: data <= 18'b111111011110110001;
18'd157: data <= 18'b111111011100100100;
18'd158: data <= 18'b111111100001101010;
18'd159: data <= 18'b111111100010000111;
18'd160: data <= 18'b111111011110110111;
18'd161: data <= 18'b111111100100011110;
18'd162: data <= 18'b111111101000001101;
18'd163: data <= 18'b111111011000101101;
18'd164: data <= 18'b111111011011100010;
18'd165: data <= 18'b111111100111010000;
18'd166: data <= 18'b111111011111000100;
18'd167: data <= 18'b111111010010111111;
18'd168: data <= 18'b111111100100000000;
18'd169: data <= 18'b111111100110101100;
18'd170: data <= 18'b111111100001100100;
18'd171: data <= 18'b111111100001000011;
18'd172: data <= 18'b111111100000000110;
18'd173: data <= 18'b111111100010011100;
18'd174: data <= 18'b111111101000001011;
18'd175: data <= 18'b111111100101100110;
18'd176: data <= 18'b111111011101111011;
18'd177: data <= 18'b111111011011101111;
18'd178: data <= 18'b111111011110100010;
18'd179: data <= 18'b111111101010100001;
18'd180: data <= 18'b111111101001111100;
18'd181: data <= 18'b111111011011001110;
18'd182: data <= 18'b111111011000001110;
18'd183: data <= 18'b111111011111001001;
18'd184: data <= 18'b111111100110001001;
18'd185: data <= 18'b111111011111010010;
18'd186: data <= 18'b111111011100011100;
18'd187: data <= 18'b111111011101011011;
18'd188: data <= 18'b111111100100010000;
18'd189: data <= 18'b111111100010011101;
18'd190: data <= 18'b111111010101110101;
18'd191: data <= 18'b111111100000110010;
18'd192: data <= 18'b111111011011100001;
18'd193: data <= 18'b111111011111000101;
18'd194: data <= 18'b111111011111011001;
18'd195: data <= 18'b111111011001101110;
18'd196: data <= 18'b111111011100010000;
18'd197: data <= 18'b111111010111001001;
18'd198: data <= 18'b111111011110100001;
18'd199: data <= 18'b111111100100000110;
18'd200: data <= 18'b111111100111011011;
18'd201: data <= 18'b111111100101110100;
18'd202: data <= 18'b111111100011100001;
18'd203: data <= 18'b111111011011111011;
18'd204: data <= 18'b111111100100101111;
18'd205: data <= 18'b111111011110111011;
18'd206: data <= 18'b111111011100000000;
18'd207: data <= 18'b111111011011000011;
18'd208: data <= 18'b111111100110011001;
18'd209: data <= 18'b111111100100011010;
18'd210: data <= 18'b111111011011101001;
18'd211: data <= 18'b111111011101110011;
18'd212: data <= 18'b111111010101010100;
18'd213: data <= 18'b111111010100001110;
18'd214: data <= 18'b111111011111010000;
18'd215: data <= 18'b111111011101001110;
18'd216: data <= 18'b111111010010010011;
18'd217: data <= 18'b111111100010111010;
18'd218: data <= 18'b111111101101010011;
18'd219: data <= 18'b111111101010010001;
18'd220: data <= 18'b111111010110001110;
18'd221: data <= 18'b111111100100100111;
18'd222: data <= 18'b111111100000000111;
18'd223: data <= 18'b111111011010110111;
18'd224: data <= 18'b111111100010011001;
18'd225: data <= 18'b111111010100010000;
18'd226: data <= 18'b111111010111011110;
18'd227: data <= 18'b111111010111000100;
18'd228: data <= 18'b111111001111000100;
18'd229: data <= 18'b111111011010110001;
18'd230: data <= 18'b111111100100100011;
18'd231: data <= 18'b111111011111000011;
18'd232: data <= 18'b111111010011111100;
18'd233: data <= 18'b111111010100101001;
18'd234: data <= 18'b111111010111100011;
18'd235: data <= 18'b111111101000011100;
18'd236: data <= 18'b111111011100100100;
18'd237: data <= 18'b111111011001101000;
18'd238: data <= 18'b111111011110100000;
18'd239: data <= 18'b111111100000011011;
18'd240: data <= 18'b111111100100001010;
18'd241: data <= 18'b111111011110011010;
18'd242: data <= 18'b111111010101111011;
18'd243: data <= 18'b111111011011001111;
18'd244: data <= 18'b111111010101000001;
18'd245: data <= 18'b111111011100000111;
18'd246: data <= 18'b111111011000110110;
18'd247: data <= 18'b111111011001111110;
18'd248: data <= 18'b111111100110011101;
18'd249: data <= 18'b111111101101111111;
18'd250: data <= 18'b111111011111101111;
18'd251: data <= 18'b111111100101100111;
18'd252: data <= 18'b111111011101010111;
18'd253: data <= 18'b111111010000111000;
18'd254: data <= 18'b111111011111110110;
18'd255: data <= 18'b111111100011111111;
18'd256: data <= 18'b111111011110110111;
18'd257: data <= 18'b000000010111101100;
18'd258: data <= 18'b000000001000011001;
18'd259: data <= 18'b000000010010000010;
18'd260: data <= 18'b000000001101110110;
18'd261: data <= 18'b000000010010000101;
18'd262: data <= 18'b000000001101001101;
18'd263: data <= 18'b000000001111111010;
18'd264: data <= 18'b000000010000001101;
18'd265: data <= 18'b000000010010111000;
18'd266: data <= 18'b000000011000011010;
18'd267: data <= 18'b000000010001010011;
18'd268: data <= 18'b000000010000100110;
18'd269: data <= 18'b000000011000001111;
18'd270: data <= 18'b000000011000010010;
18'd271: data <= 18'b000000001101100101;
18'd272: data <= 18'b000000001011111001;
18'd273: data <= 18'b000000010011011001;
18'd274: data <= 18'b000000001100011111;
18'd275: data <= 18'b000000010001110011;
18'd276: data <= 18'b000000011000010001;
18'd277: data <= 18'b000000010001111111;
18'd278: data <= 18'b000000001101011000;
18'd279: data <= 18'b000000010001100001;
18'd280: data <= 18'b000000010000111001;
18'd281: data <= 18'b000000001100010001;
18'd282: data <= 18'b000000001011110101;
18'd283: data <= 18'b000000010001011001;
18'd284: data <= 18'b000000010011001111;
18'd285: data <= 18'b000000001100000000;
18'd286: data <= 18'b000000001110010100;
18'd287: data <= 18'b000000010001000110;
18'd288: data <= 18'b000000010110100000;
18'd289: data <= 18'b000000010001011110;
18'd290: data <= 18'b000000010101010001;
18'd291: data <= 18'b000000010010010101;
18'd292: data <= 18'b000000010100000011;
18'd293: data <= 18'b000000001111001101;
18'd294: data <= 18'b000000001010101100;
18'd295: data <= 18'b000000001101110101;
18'd296: data <= 18'b000000010101010000;
18'd297: data <= 18'b000000001110101101;
18'd298: data <= 18'b000000001011010000;
18'd299: data <= 18'b000000001001100001;
18'd300: data <= 18'b000000010010100100;
18'd301: data <= 18'b000000010100010000;
18'd302: data <= 18'b000000010000110010;
18'd303: data <= 18'b000000010101000000;
18'd304: data <= 18'b000000001101110000;
18'd305: data <= 18'b000000011000101111;
18'd306: data <= 18'b000000010101010000;
18'd307: data <= 18'b000000010001111000;
18'd308: data <= 18'b000000010011011010;
18'd309: data <= 18'b000000001110000101;
18'd310: data <= 18'b000000010001011100;
18'd311: data <= 18'b000000001111000000;
18'd312: data <= 18'b000000001111000111;
18'd313: data <= 18'b000000010100000011;
18'd314: data <= 18'b000000010000011000;
18'd315: data <= 18'b000000010110010001;
18'd316: data <= 18'b000000010010001001;
18'd317: data <= 18'b000000010010010101;
18'd318: data <= 18'b000000010011001111;
18'd319: data <= 18'b000000010000111110;
18'd320: data <= 18'b000000010101100010;
18'd321: data <= 18'b000000010100001110;
18'd322: data <= 18'b000000010010011001;
18'd323: data <= 18'b000000001010011000;
18'd324: data <= 18'b000000001100011001;
18'd325: data <= 18'b000000001100110111;
18'd326: data <= 18'b000000001110110010;
18'd327: data <= 18'b000000001110101001;
18'd328: data <= 18'b000000011010100111;
18'd329: data <= 18'b000000011010001110;
18'd330: data <= 18'b000000010100100000;
18'd331: data <= 18'b000000010001010111;
18'd332: data <= 18'b000000001111001011;
18'd333: data <= 18'b000000010100110011;
18'd334: data <= 18'b000000010010110001;
18'd335: data <= 18'b000000001010100110;
18'd336: data <= 18'b000000011000111011;
18'd337: data <= 18'b000000001101000101;
18'd338: data <= 18'b000000010110011100;
18'd339: data <= 18'b000000010110111110;
18'd340: data <= 18'b000000001111100111;
18'd341: data <= 18'b000000001111101100;
18'd342: data <= 18'b000000001001101110;
18'd343: data <= 18'b000000010001000011;
18'd344: data <= 18'b000000010100001111;
18'd345: data <= 18'b000000001111000100;
18'd346: data <= 18'b000000010010110000;
18'd347: data <= 18'b000000010100111011;
18'd348: data <= 18'b000000001110011111;
18'd349: data <= 18'b000000001100010001;
18'd350: data <= 18'b000000010100101010;
18'd351: data <= 18'b000000001111101011;
18'd352: data <= 18'b000000010001110001;
18'd353: data <= 18'b000000010100111111;
18'd354: data <= 18'b000000010100001111;
18'd355: data <= 18'b000000001101110010;
18'd356: data <= 18'b000000001101111011;
18'd357: data <= 18'b000000000101111011;
18'd358: data <= 18'b000000001101001001;
18'd359: data <= 18'b000000001100000110;
18'd360: data <= 18'b000000000101111111;
18'd361: data <= 18'b000000001111000000;
18'd362: data <= 18'b000000001011110110;
18'd363: data <= 18'b000000010100100000;
18'd364: data <= 18'b000000001110111001;
18'd365: data <= 18'b000000010001010100;
18'd366: data <= 18'b000000010001000001;
18'd367: data <= 18'b000000010011000101;
18'd368: data <= 18'b000000001111011000;
18'd369: data <= 18'b000000001111100101;
18'd370: data <= 18'b000000001100100010;
18'd371: data <= 18'b000000001100110011;
18'd372: data <= 18'b000000001110000111;
18'd373: data <= 18'b000000001110010000;
18'd374: data <= 18'b000000010100000100;
18'd375: data <= 18'b000000001011110111;
18'd376: data <= 18'b000000001110111110;
18'd377: data <= 18'b000000010110010010;
18'd378: data <= 18'b000000001101100011;
18'd379: data <= 18'b000000001001111011;
18'd380: data <= 18'b000000010100000001;
18'd381: data <= 18'b000000001100000111;
18'd382: data <= 18'b000000010001000010;
18'd383: data <= 18'b000000001100000101;
18'd384: data <= 18'b000000010101001100;
18'd385: data <= 18'b000000010011110110;
18'd386: data <= 18'b000000001101101010;
 

endcase
end
endmodule

module  rom_dense_i_class3  (clock, address, data);
input clock;
input [17:0] address;
output [17:0] data;
reg signed [17:0] data;

always@(posedge clock)
begin
    case(address)
    18'd000: data <= 18'b111111110000111010;
18'd001: data <= 18'b111111110010101110;
18'd002: data <= 18'b111111101111001010;
18'd003: data <= 18'b111111111010000010;
18'd004: data <= 18'b111111110101011110;
18'd005: data <= 18'b111111110101110001;
18'd006: data <= 18'b111111111000010110;
18'd007: data <= 18'b111111110011111011;
18'd008: data <= 18'b111111110110110001;
18'd009: data <= 18'b111111101111010101;
18'd010: data <= 18'b111111110110011001;
18'd011: data <= 18'b111111110011111011;
18'd012: data <= 18'b111111111010011010;
18'd013: data <= 18'b111111110110010010;
18'd014: data <= 18'b111111111111100101;
18'd015: data <= 18'b111111110100111001;
18'd016: data <= 18'b111111110000110101;
18'd017: data <= 18'b111111111000100111;
18'd018: data <= 18'b111111111000010101;
18'd019: data <= 18'b111111110010100000;
18'd020: data <= 18'b111111111000101011;
18'd021: data <= 18'b111111111101111101;
18'd022: data <= 18'b111111111100001011;
18'd023: data <= 18'b111111110000010001;
18'd024: data <= 18'b111111101101101001;
18'd025: data <= 18'b111111110001001111;
18'd026: data <= 18'b111111111010111110;
18'd027: data <= 18'b000000000001001110;
18'd028: data <= 18'b111111110001000000;
18'd029: data <= 18'b111111110111111101;
18'd030: data <= 18'b111111111000110001;
18'd031: data <= 18'b111111110111101001;
18'd032: data <= 18'b111111110011111000;
18'd033: data <= 18'b111111111001001101;
18'd034: data <= 18'b111111111100000010;
18'd035: data <= 18'b111111111001110100;
18'd036: data <= 18'b111111110011111111;
18'd037: data <= 18'b111111111000000001;
18'd038: data <= 18'b111111110101100000;
18'd039: data <= 18'b111111110010011110;
18'd040: data <= 18'b111111110010100000;
18'd041: data <= 18'b111111111001111011;
18'd042: data <= 18'b111111111010111110;
18'd043: data <= 18'b000000000000100111;
18'd044: data <= 18'b000000000101000110;
18'd045: data <= 18'b000000000010111100;
18'd046: data <= 18'b111111110111001000;
18'd047: data <= 18'b111111111001000111;
18'd048: data <= 18'b111111110111101000;
18'd049: data <= 18'b111111111010100010;
18'd050: data <= 18'b111111111010100000;
18'd051: data <= 18'b111111111000110010;
18'd052: data <= 18'b111111111100011101;
18'd053: data <= 18'b111111111010000101;
18'd054: data <= 18'b111111110100111100;
18'd055: data <= 18'b111111110011101011;
18'd056: data <= 18'b111111101111101010;
18'd057: data <= 18'b111111101100010010;
18'd058: data <= 18'b111111110101010101;
18'd059: data <= 18'b111111110011001001;
18'd060: data <= 18'b111111110010001110;
18'd061: data <= 18'b111111101111111101;
18'd062: data <= 18'b111111101011001010;
18'd063: data <= 18'b111111110000100001;
18'd064: data <= 18'b111111110011111010;
18'd065: data <= 18'b111111111010111111;
18'd066: data <= 18'b111111111000010110;
18'd067: data <= 18'b111111110000101111;
18'd068: data <= 18'b111111110000101101;
18'd069: data <= 18'b111111110101101101;
18'd070: data <= 18'b111111110000011110;
18'd071: data <= 18'b111111101011011010;
18'd072: data <= 18'b111111111000110101;
18'd073: data <= 18'b111111110100101100;
18'd074: data <= 18'b111111101110111000;
18'd075: data <= 18'b111111101000011001;
18'd076: data <= 18'b111111110101111110;
18'd077: data <= 18'b111111111001110010;
18'd078: data <= 18'b111111111101001111;
18'd079: data <= 18'b111111111001110001;
18'd080: data <= 18'b111111111010111001;
18'd081: data <= 18'b111111110011001000;
18'd082: data <= 18'b111111101110001111;
18'd083: data <= 18'b111111110000100110;
18'd084: data <= 18'b111111111000000110;
18'd085: data <= 18'b111111110011000111;
18'd086: data <= 18'b111111101000100101;
18'd087: data <= 18'b111111110000011110;
18'd088: data <= 18'b111111110010111100;
18'd089: data <= 18'b111111110111101010;
18'd090: data <= 18'b111111110100100110;
18'd091: data <= 18'b111111101110010000;
18'd092: data <= 18'b111111101110101000;
18'd093: data <= 18'b111111101101101111;
18'd094: data <= 18'b111111101111010111;
18'd095: data <= 18'b111111111000001001;
18'd096: data <= 18'b111111110100111111;
18'd097: data <= 18'b111111101111100001;
18'd098: data <= 18'b111111100111011011;
18'd099: data <= 18'b111111110001111100;
18'd100: data <= 18'b111111111011010000;
18'd101: data <= 18'b111111110000011111;
18'd102: data <= 18'b111111101111001100;
18'd103: data <= 18'b111111110010000111;
18'd104: data <= 18'b111111110101111000;
18'd105: data <= 18'b111111110010110111;
18'd106: data <= 18'b111111110000100101;
18'd107: data <= 18'b111111110101101000;
18'd108: data <= 18'b111111110010100100;
18'd109: data <= 18'b111111111011111101;
18'd110: data <= 18'b111111110010101110;
18'd111: data <= 18'b111111101110100101;
18'd112: data <= 18'b111111110010000101;
18'd113: data <= 18'b111111101011111101;
18'd114: data <= 18'b111111110000111000;
18'd115: data <= 18'b111111111010011010;
18'd116: data <= 18'b111111110001001110;
18'd117: data <= 18'b111111101101101010;
18'd118: data <= 18'b111111101000000001;
18'd119: data <= 18'b111111101001000011;
18'd120: data <= 18'b111111110011101011;
18'd121: data <= 18'b111111100111111111;
18'd122: data <= 18'b111111101100100111;
18'd123: data <= 18'b111111101110011010;
18'd124: data <= 18'b111111101100101000;
18'd125: data <= 18'b111111110000100011;
18'd126: data <= 18'b111111101010110000;
18'd127: data <= 18'b111111101110000110;
18'd128: data <= 18'b111111101111100111;
18'd129: data <= 18'b000000000010010010;
18'd130: data <= 18'b000000110001100010;
18'd131: data <= 18'b000000110011001111;
18'd132: data <= 18'b000000101110101010;
18'd133: data <= 18'b000000111001001011;
18'd134: data <= 18'b000000110110001000;
18'd135: data <= 18'b000000110010110101;
18'd136: data <= 18'b000000111100011000;
18'd137: data <= 18'b000000110101101100;
18'd138: data <= 18'b000000101111011111;
18'd139: data <= 18'b000000110000101001;
18'd140: data <= 18'b000000110000001111;
18'd141: data <= 18'b000000110000000000;
18'd142: data <= 18'b000000110000111100;
18'd143: data <= 18'b000000101100000011;
18'd144: data <= 18'b000000100101100100;
18'd145: data <= 18'b000000111000101111;
18'd146: data <= 18'b000000110001000111;
18'd147: data <= 18'b000000101110110111;
18'd148: data <= 18'b000000101001111110;
18'd149: data <= 18'b000000101110110101;
18'd150: data <= 18'b000000101011111000;
18'd151: data <= 18'b000000110101000100;
18'd152: data <= 18'b000000110001000010;
18'd153: data <= 18'b000000111010010001;
18'd154: data <= 18'b000000101100111100;
18'd155: data <= 18'b000000101011110010;
18'd156: data <= 18'b000000101011111101;
18'd157: data <= 18'b000000110011111111;
18'd158: data <= 18'b000000110111111011;
18'd159: data <= 18'b000000110111001101;
18'd160: data <= 18'b000000110111000111;
18'd161: data <= 18'b000000110100011100;
18'd162: data <= 18'b000000100101011101;
18'd163: data <= 18'b000000011110011010;
18'd164: data <= 18'b000000100100010010;
18'd165: data <= 18'b000000111100101001;
18'd166: data <= 18'b000000101010100001;
18'd167: data <= 18'b000000110101100011;
18'd168: data <= 18'b000000111011100000;
18'd169: data <= 18'b000000110011100011;
18'd170: data <= 18'b000000101111000111;
18'd171: data <= 18'b000000110000111001;
18'd172: data <= 18'b000000100100111011;
18'd173: data <= 18'b000000101101111111;
18'd174: data <= 18'b000000101100110010;
18'd175: data <= 18'b000000110001000101;
18'd176: data <= 18'b000000110010001111;
18'd177: data <= 18'b000000101100000101;
18'd178: data <= 18'b000000100101111111;
18'd179: data <= 18'b000000100100001010;
18'd180: data <= 18'b000000100110000001;
18'd181: data <= 18'b000000110010101001;
18'd182: data <= 18'b000000110001011001;
18'd183: data <= 18'b000000110001000000;
18'd184: data <= 18'b000000101101010000;
18'd185: data <= 18'b000000110011111101;
18'd186: data <= 18'b000001000000110111;
18'd187: data <= 18'b000000101101110101;
18'd188: data <= 18'b000000110010010011;
18'd189: data <= 18'b000000111011001100;
18'd190: data <= 18'b000000110111001101;
18'd191: data <= 18'b000000111111110001;
18'd192: data <= 18'b000000110110111000;
18'd193: data <= 18'b000000110000110010;
18'd194: data <= 18'b000000101011111000;
18'd195: data <= 18'b000000110110001111;
18'd196: data <= 18'b000000111100110101;
18'd197: data <= 18'b000000111000000001;
18'd198: data <= 18'b000000110001111010;
18'd199: data <= 18'b000000111010010101;
18'd200: data <= 18'b000000110111110111;
18'd201: data <= 18'b000000101101100111;
18'd202: data <= 18'b000000110011110101;
18'd203: data <= 18'b000000111111101100;
18'd204: data <= 18'b000000110101101010;
18'd205: data <= 18'b000000101011010000;
18'd206: data <= 18'b000000110100011001;
18'd207: data <= 18'b000000110010100110;
18'd208: data <= 18'b000000110101100111;
18'd209: data <= 18'b000000110100011001;
18'd210: data <= 18'b000000110001000011;
18'd211: data <= 18'b000000110000000001;
18'd212: data <= 18'b000000111111000011;
18'd213: data <= 18'b000000110000111001;
18'd214: data <= 18'b000000111010111110;
18'd215: data <= 18'b000000111111010101;
18'd216: data <= 18'b000000111101101111;
18'd217: data <= 18'b000000100111110000;
18'd218: data <= 18'b000000100111001100;
18'd219: data <= 18'b000000110001011011;
18'd220: data <= 18'b000000111010000100;
18'd221: data <= 18'b000000110101010100;
18'd222: data <= 18'b000000111110100011;
18'd223: data <= 18'b000000111111111011;
18'd224: data <= 18'b000000101101110110;
18'd225: data <= 18'b000000110011100100;
18'd226: data <= 18'b000000111111111011;
18'd227: data <= 18'b000000111011010000;
18'd228: data <= 18'b000000101101001010;
18'd229: data <= 18'b000000101011010000;
18'd230: data <= 18'b000000110011100011;
18'd231: data <= 18'b000000110110110010;
18'd232: data <= 18'b000000111001000011;
18'd233: data <= 18'b000000110000111001;
18'd234: data <= 18'b000000110111111111;
18'd235: data <= 18'b000000111111010010;
18'd236: data <= 18'b000000110010101111;
18'd237: data <= 18'b000000111100110010;
18'd238: data <= 18'b000000110111110000;
18'd239: data <= 18'b000000110010011101;
18'd240: data <= 18'b000001000011100010;
18'd241: data <= 18'b000000110010010001;
18'd242: data <= 18'b000000101010001111;
18'd243: data <= 18'b000000110011111011;
18'd244: data <= 18'b000000101001010111;
18'd245: data <= 18'b000000110100001110;
18'd246: data <= 18'b000000110010111011;
18'd247: data <= 18'b000001000000010000;
18'd248: data <= 18'b000001000101011110;
18'd249: data <= 18'b000000111011101010;
18'd250: data <= 18'b000001000000100110;
18'd251: data <= 18'b000000111100011110;
18'd252: data <= 18'b000000111001101100;
18'd253: data <= 18'b000000110010100111;
18'd254: data <= 18'b000000111010011001;
18'd255: data <= 18'b000000111001010000;
18'd256: data <= 18'b000000111100100000;
18'd257: data <= 18'b000000000001110000;
18'd258: data <= 18'b000000000011001001;
18'd259: data <= 18'b000000000110101010;
18'd260: data <= 18'b000000000011101011;
18'd261: data <= 18'b111111111100000001;
18'd262: data <= 18'b000000001000000001;
18'd263: data <= 18'b000000000111000010;
18'd264: data <= 18'b000000000000010110;
18'd265: data <= 18'b000000001000101111;
18'd266: data <= 18'b000000001010001000;
18'd267: data <= 18'b000000000100111110;
18'd268: data <= 18'b000000000111000100;
18'd269: data <= 18'b000000000100001110;
18'd270: data <= 18'b000000000011001100;
18'd271: data <= 18'b000000000111000000;
18'd272: data <= 18'b111111111111000010;
18'd273: data <= 18'b000000000110110000;
18'd274: data <= 18'b000000000011010000;
18'd275: data <= 18'b000000000110111000;
18'd276: data <= 18'b000000000011100010;
18'd277: data <= 18'b000000000111100001;
18'd278: data <= 18'b111111111111011111;
18'd279: data <= 18'b000000000000001101;
18'd280: data <= 18'b000000000101000001;
18'd281: data <= 18'b000000000011011010;
18'd282: data <= 18'b000000000010100001;
18'd283: data <= 18'b000000000110101000;
18'd284: data <= 18'b000000000001010011;
18'd285: data <= 18'b000000000101000001;
18'd286: data <= 18'b000000000111001110;
18'd287: data <= 18'b000000000011010101;
18'd288: data <= 18'b000000000000111011;
18'd289: data <= 18'b000000000101111100;
18'd290: data <= 18'b000000000011010000;
18'd291: data <= 18'b111111111110001000;
18'd292: data <= 18'b000000000000001010;
18'd293: data <= 18'b000000000011000110;
18'd294: data <= 18'b000000000010110110;
18'd295: data <= 18'b111111111111111101;
18'd296: data <= 18'b000000000000111110;
18'd297: data <= 18'b000000000110110111;
18'd298: data <= 18'b000000000111011111;
18'd299: data <= 18'b111111111111111111;
18'd300: data <= 18'b000000000100000100;
18'd301: data <= 18'b111111111100111110;
18'd302: data <= 18'b111111111110000000;
18'd303: data <= 18'b111111111101111000;
18'd304: data <= 18'b111111111101110000;
18'd305: data <= 18'b000000000010011111;
18'd306: data <= 18'b000000000000100000;
18'd307: data <= 18'b000000000100010011;
18'd308: data <= 18'b000000000001111100;
18'd309: data <= 18'b000000000011110001;
18'd310: data <= 18'b111111111111010111;
18'd311: data <= 18'b000000000101100110;
18'd312: data <= 18'b000000001000001001;
18'd313: data <= 18'b111111111101101011;
18'd314: data <= 18'b000000000010100110;
18'd315: data <= 18'b000000000101011011;
18'd316: data <= 18'b111111111111000000;
18'd317: data <= 18'b000000001001010001;
18'd318: data <= 18'b000000000011111011;
18'd319: data <= 18'b000000000010010100;
18'd320: data <= 18'b000000000110010111;
18'd321: data <= 18'b111111111111000100;
18'd322: data <= 18'b000000000000011000;
18'd323: data <= 18'b000000000000001101;
18'd324: data <= 18'b000000000110111000;
18'd325: data <= 18'b000000000101010111;
18'd326: data <= 18'b000000000001000110;
18'd327: data <= 18'b000000000001111111;
18'd328: data <= 18'b000000001001100001;
18'd329: data <= 18'b000000001000100100;
18'd330: data <= 18'b000000000101000100;
18'd331: data <= 18'b000000001010100001;
18'd332: data <= 18'b000000000101101101;
18'd333: data <= 18'b000000000111000001;
18'd334: data <= 18'b000000000101011110;
18'd335: data <= 18'b000000000111000011;
18'd336: data <= 18'b000000000001100110;
18'd337: data <= 18'b111111111110111010;
18'd338: data <= 18'b000000000010000010;
18'd339: data <= 18'b000000000110011100;
18'd340: data <= 18'b000000001000110000;
18'd341: data <= 18'b000000000011110100;
18'd342: data <= 18'b000000000011001111;
18'd343: data <= 18'b111111111111111100;
18'd344: data <= 18'b000000010000000010;
18'd345: data <= 18'b000000000110111001;
18'd346: data <= 18'b000000000000010011;
18'd347: data <= 18'b000000000010101011;
18'd348: data <= 18'b000000000011111101;
18'd349: data <= 18'b000000000111011000;
18'd350: data <= 18'b000000000100001100;
18'd351: data <= 18'b000000000110110010;
18'd352: data <= 18'b000000001001111111;
18'd353: data <= 18'b000000000010111010;
18'd354: data <= 18'b000000000011000111;
18'd355: data <= 18'b000000001110001010;
18'd356: data <= 18'b000000000110001100;
18'd357: data <= 18'b000000001000100000;
18'd358: data <= 18'b000000000111000111;
18'd359: data <= 18'b000000000101010101;
18'd360: data <= 18'b000000000001101011;
18'd361: data <= 18'b000000000011110111;
18'd362: data <= 18'b000000000001001111;
18'd363: data <= 18'b000000000111000001;
18'd364: data <= 18'b000000000101110110;
18'd365: data <= 18'b000000000101100101;
18'd366: data <= 18'b000000001000111010;
18'd367: data <= 18'b111111111111110111;
18'd368: data <= 18'b000000000000101010;
18'd369: data <= 18'b000000000111101000;
18'd370: data <= 18'b000000000001100010;
18'd371: data <= 18'b000000000001011010;
18'd372: data <= 18'b000000000000000100;
18'd373: data <= 18'b000000000000110001;
18'd374: data <= 18'b000000000111001101;
18'd375: data <= 18'b000000000111011011;
18'd376: data <= 18'b000000001011011010;
18'd377: data <= 18'b000000000100100011;
18'd378: data <= 18'b000000001001000100;
18'd379: data <= 18'b000000000110110101;
18'd380: data <= 18'b000000000100110111;
18'd381: data <= 18'b000000001110011101;
18'd382: data <= 18'b000000001011100100;
18'd383: data <= 18'b000000000011000011;
18'd384: data <= 18'b000000001001001111;
18'd385: data <= 18'b000000001011111100;
18'd386: data <= 18'b111111111101111101;
 
endcase
end
endmodule


module  rom_dense_q_class3  (clock, address, data);
input clock;
input [17:0] address;
output [17:0] data;
reg signed [17:0] data;

always@(posedge clock)
begin
    case(address)
    18'd000: data <= 18'b111111110011011010;
18'd001: data <= 18'b000000000111100000;
18'd002: data <= 18'b000000000101000110;
18'd003: data <= 18'b000000000100000101;
18'd004: data <= 18'b000000000110010001;
18'd005: data <= 18'b000000001101111101;
18'd006: data <= 18'b000000010110100010;
18'd007: data <= 18'b000000010001110101;
18'd008: data <= 18'b000000010011100000;
18'd009: data <= 18'b000000010011000100;
18'd010: data <= 18'b000000010110110111;
18'd011: data <= 18'b000000011000100010;
18'd012: data <= 18'b000000010100010110;
18'd013: data <= 18'b000000001110010100;
18'd014: data <= 18'b000000001100010000;
18'd015: data <= 18'b000000001011000010;
18'd016: data <= 18'b000000001011111110;
18'd017: data <= 18'b000000001111001010;
18'd018: data <= 18'b000000001111110000;
18'd019: data <= 18'b000000010000000110;
18'd020: data <= 18'b000000001001000010;
18'd021: data <= 18'b000000001101001100;
18'd022: data <= 18'b000000001010111010;
18'd023: data <= 18'b000000000011000111;
18'd024: data <= 18'b000000001001001100;
18'd025: data <= 18'b000000000101001110;
18'd026: data <= 18'b000000001000011000;
18'd027: data <= 18'b000000010001000000;
18'd028: data <= 18'b000000001101001110;
18'd029: data <= 18'b000000010101010101;
18'd030: data <= 18'b000000010000001000;
18'd031: data <= 18'b000000010000110101;
18'd032: data <= 18'b000000010010101100;
18'd033: data <= 18'b000000001111000001;
18'd034: data <= 18'b000000010010001110;
18'd035: data <= 18'b000000001110001000;
18'd036: data <= 18'b000000010000010011;
18'd037: data <= 18'b000000000111111111;
18'd038: data <= 18'b000000000110001110;
18'd039: data <= 18'b000000001000001111;
18'd040: data <= 18'b000000000111100010;
18'd041: data <= 18'b000000001100000110;
18'd042: data <= 18'b000000001000111010;
18'd043: data <= 18'b000000000001100010;
18'd044: data <= 18'b000000001111000010;
18'd045: data <= 18'b000000010010101011;
18'd046: data <= 18'b000000000111101010;
18'd047: data <= 18'b000000001001100111;
18'd048: data <= 18'b000000001111100001;
18'd049: data <= 18'b000000001111101011;
18'd050: data <= 18'b000000010111111000;
18'd051: data <= 18'b000000001101001000;
18'd052: data <= 18'b111111111111000100;
18'd053: data <= 18'b000000001000010100;
18'd054: data <= 18'b000000000010110110;
18'd055: data <= 18'b000000001101011010;
18'd056: data <= 18'b000000001010110001;
18'd057: data <= 18'b000000000000110001;
18'd058: data <= 18'b000000001011100000;
18'd059: data <= 18'b000000000111010110;
18'd060: data <= 18'b000000000110100000;
18'd061: data <= 18'b111111111110101101;
18'd062: data <= 18'b000000000110111111;
18'd063: data <= 18'b000000000110001011;
18'd064: data <= 18'b000000001010000001;
18'd065: data <= 18'b000000000001011111;
18'd066: data <= 18'b111111111111011010;
18'd067: data <= 18'b000000000101000001;
18'd068: data <= 18'b000000000001011101;
18'd069: data <= 18'b000000000001100101;
18'd070: data <= 18'b000000001001111101;
18'd071: data <= 18'b000000001010000011;
18'd072: data <= 18'b000000000010000110;
18'd073: data <= 18'b000000000101111000;
18'd074: data <= 18'b000000000111001010;
18'd075: data <= 18'b000000001101101001;
18'd076: data <= 18'b000000000111101101;
18'd077: data <= 18'b000000001000100010;
18'd078: data <= 18'b000000000110110000;
18'd079: data <= 18'b000000001100100100;
18'd080: data <= 18'b000000000110011010;
18'd081: data <= 18'b000000000111011011;
18'd082: data <= 18'b000000000110000101;
18'd083: data <= 18'b000000001010001000;
18'd084: data <= 18'b000000000100011110;
18'd085: data <= 18'b000000000110010111;
18'd086: data <= 18'b000000001100100111;
18'd087: data <= 18'b000000001010110000;
18'd088: data <= 18'b000000001101010011;
18'd089: data <= 18'b000000010010000000;
18'd090: data <= 18'b000000001100000111;
18'd091: data <= 18'b000000001011000000;
18'd092: data <= 18'b000000010000000111;
18'd093: data <= 18'b000000001101100010;
18'd094: data <= 18'b000000010011000000;
18'd095: data <= 18'b000000010001101000;
18'd096: data <= 18'b000000000100101001;
18'd097: data <= 18'b000000000110100011;
18'd098: data <= 18'b000000001010011111;
18'd099: data <= 18'b000000000010011000;
18'd100: data <= 18'b000000001101111110;
18'd101: data <= 18'b000000001101011000;
18'd102: data <= 18'b000000001000011100;
18'd103: data <= 18'b000000001101010101;
18'd104: data <= 18'b000000001101011000;
18'd105: data <= 18'b000000001110010101;
18'd106: data <= 18'b000000001110101100;
18'd107: data <= 18'b000000010101100100;
18'd108: data <= 18'b000000001000010011;
18'd109: data <= 18'b000000010101101100;
18'd110: data <= 18'b000000010111100011;
18'd111: data <= 18'b000000001110010000;
18'd112: data <= 18'b000000001101010000;
18'd113: data <= 18'b000000001100010101;
18'd114: data <= 18'b000000001010101011;
18'd115: data <= 18'b000000001001001000;
18'd116: data <= 18'b000000000111101011;
18'd117: data <= 18'b000000001000011111;
18'd118: data <= 18'b000000001000111001;
18'd119: data <= 18'b000000010000000011;
18'd120: data <= 18'b000000001011100111;
18'd121: data <= 18'b000000001111101100;
18'd122: data <= 18'b000000001101111000;
18'd123: data <= 18'b000000001100001001;
18'd124: data <= 18'b000000001011000100;
18'd125: data <= 18'b000000001100000001;
18'd126: data <= 18'b000000001111011100;
18'd127: data <= 18'b000000001001101000;
18'd128: data <= 18'b111111111100111100;
18'd129: data <= 18'b111111111011110100;
18'd130: data <= 18'b000000111101110111;
18'd131: data <= 18'b000000111011110000;
18'd132: data <= 18'b000000111111000011;
18'd133: data <= 18'b000000111010001000;
18'd134: data <= 18'b000001000000111001;
18'd135: data <= 18'b000000110011101001;
18'd136: data <= 18'b000000110001100000;
18'd137: data <= 18'b000000101101010111;
18'd138: data <= 18'b000000111010100000;
18'd139: data <= 18'b000000101011001111;
18'd140: data <= 18'b000000101011110101;
18'd141: data <= 18'b000000101100110110;
18'd142: data <= 18'b000000110110110100;
18'd143: data <= 18'b000000110011100011;
18'd144: data <= 18'b000000101110011101;
18'd145: data <= 18'b000000101111111011;
18'd146: data <= 18'b000000111101001010;
18'd147: data <= 18'b000000110101001110;
18'd148: data <= 18'b000000110110000010;
18'd149: data <= 18'b000000111000010100;
18'd150: data <= 18'b000000111010001100;
18'd151: data <= 18'b000000111001111101;
18'd152: data <= 18'b000000111101011110;
18'd153: data <= 18'b000000111011010011;
18'd154: data <= 18'b000000111001000000;
18'd155: data <= 18'b000000110000101010;
18'd156: data <= 18'b000000110100011001;
18'd157: data <= 18'b000000111011010110;
18'd158: data <= 18'b000000110010010000;
18'd159: data <= 18'b000000110101101010;
18'd160: data <= 18'b000000110001010001;
18'd161: data <= 18'b000000101110101100;
18'd162: data <= 18'b000000101101000010;
18'd163: data <= 18'b000000111001010000;
18'd164: data <= 18'b000000111100110001;
18'd165: data <= 18'b000000110000111001;
18'd166: data <= 18'b000000111100010110;
18'd167: data <= 18'b000001000011000011;
18'd168: data <= 18'b000000110101111110;
18'd169: data <= 18'b000000111010010010;
18'd170: data <= 18'b000000111001101110;
18'd171: data <= 18'b000000111111010110;
18'd172: data <= 18'b000001000001001001;
18'd173: data <= 18'b000000111101000010;
18'd174: data <= 18'b000000111001101110;
18'd175: data <= 18'b000000110101000010;
18'd176: data <= 18'b000000111100001001;
18'd177: data <= 18'b000000111011100110;
18'd178: data <= 18'b000000111110000101;
18'd179: data <= 18'b000000101101010010;
18'd180: data <= 18'b000000111000000110;
18'd181: data <= 18'b000001000010000000;
18'd182: data <= 18'b000000111110000110;
18'd183: data <= 18'b000001000100010000;
18'd184: data <= 18'b000000110111110000;
18'd185: data <= 18'b000000111001011011;
18'd186: data <= 18'b000000111110101010;
18'd187: data <= 18'b000001000011000000;
18'd188: data <= 18'b000001000000000100;
18'd189: data <= 18'b000000110111000101;
18'd190: data <= 18'b000001000101100010;
18'd191: data <= 18'b000001000010111100;
18'd192: data <= 18'b000000111111011101;
18'd193: data <= 18'b000000111110001011;
18'd194: data <= 18'b000001000000111010;
18'd195: data <= 18'b000001001000101011;
18'd196: data <= 18'b000001000100000101;
18'd197: data <= 18'b000001000101011010;
18'd198: data <= 18'b000001000100110100;
18'd199: data <= 18'b000000111000000000;
18'd200: data <= 18'b000000111001100000;
18'd201: data <= 18'b000000111110001011;
18'd202: data <= 18'b000000111101100101;
18'd203: data <= 18'b000001000001010011;
18'd204: data <= 18'b000000110101001110;
18'd205: data <= 18'b000000111111010111;
18'd206: data <= 18'b000001000110101000;
18'd207: data <= 18'b000000111000011110;
18'd208: data <= 18'b000000110101010111;
18'd209: data <= 18'b000000111111011100;
18'd210: data <= 18'b000000111110001110;
18'd211: data <= 18'b000000111100001010;
18'd212: data <= 18'b000001000111010000;
18'd213: data <= 18'b000001000101001001;
18'd214: data <= 18'b000000110111001000;
18'd215: data <= 18'b000000111001001010;
18'd216: data <= 18'b000000111001010111;
18'd217: data <= 18'b000000111011101001;
18'd218: data <= 18'b000000101011110001;
18'd219: data <= 18'b000000110101001011;
18'd220: data <= 18'b000000111010110111;
18'd221: data <= 18'b000000110111001101;
18'd222: data <= 18'b000000110110010001;
18'd223: data <= 18'b000000110110010111;
18'd224: data <= 18'b000000110000010000;
18'd225: data <= 18'b000000110111000100;
18'd226: data <= 18'b000000111111000100;
18'd227: data <= 18'b000000111011000110;
18'd228: data <= 18'b000000111110110100;
18'd229: data <= 18'b000000111011100001;
18'd230: data <= 18'b000000110011111101;
18'd231: data <= 18'b000000111000010111;
18'd232: data <= 18'b000000110110010001;
18'd233: data <= 18'b000000111011101011;
18'd234: data <= 18'b000000111000111101;
18'd235: data <= 18'b000000110001101101;
18'd236: data <= 18'b000000110101010011;
18'd237: data <= 18'b000000111010000110;
18'd238: data <= 18'b000000111100010111;
18'd239: data <= 18'b000000110101010100;
18'd240: data <= 18'b000000110001111010;
18'd241: data <= 18'b000000111011101000;
18'd242: data <= 18'b000000111001011100;
18'd243: data <= 18'b000000110101001010;
18'd244: data <= 18'b000001000000110111;
18'd245: data <= 18'b000000111011000100;
18'd246: data <= 18'b000001000101110000;
18'd247: data <= 18'b000001000000001110;
18'd248: data <= 18'b000000110101110011;
18'd249: data <= 18'b000000110001111110;
18'd250: data <= 18'b000001000000000000;
18'd251: data <= 18'b000000110110111101;
18'd252: data <= 18'b000000111111101010;
18'd253: data <= 18'b000001001011111111;
18'd254: data <= 18'b000000111111001111;
18'd255: data <= 18'b000000111011011001;
18'd256: data <= 18'b000000111101001111;
18'd257: data <= 18'b000000001010100001;
18'd258: data <= 18'b000000000010000111;
18'd259: data <= 18'b111111111100101001;
18'd260: data <= 18'b111111111111010100;
18'd261: data <= 18'b111111111100010101;
18'd262: data <= 18'b000000000000101100;
18'd263: data <= 18'b000000000000010000;
18'd264: data <= 18'b111111111011011011;
18'd265: data <= 18'b111111111011100011;
18'd266: data <= 18'b111111111010111011;
18'd267: data <= 18'b111111111100100001;
18'd268: data <= 18'b111111111011110001;
18'd269: data <= 18'b111111111000100101;
18'd270: data <= 18'b111111111000000100;
18'd271: data <= 18'b111111111010110010;
18'd272: data <= 18'b111111111100100011;
18'd273: data <= 18'b111111111100001110;
18'd274: data <= 18'b111111111111000011;
18'd275: data <= 18'b111111111110100110;
18'd276: data <= 18'b111111111011011010;
18'd277: data <= 18'b000000000000000100;
18'd278: data <= 18'b000000000100100001;
18'd279: data <= 18'b111111111011010100;
18'd280: data <= 18'b111111111100100010;
18'd281: data <= 18'b111111111101001011;
18'd282: data <= 18'b111111111011001011;
18'd283: data <= 18'b000000001000000100;
18'd284: data <= 18'b111111110111001010;
18'd285: data <= 18'b111111111111010101;
18'd286: data <= 18'b000000000011000110;
18'd287: data <= 18'b111111111011111001;
18'd288: data <= 18'b111111111101110001;
18'd289: data <= 18'b111111111101100100;
18'd290: data <= 18'b111111111011101100;
18'd291: data <= 18'b111111110011001110;
18'd292: data <= 18'b111111111101101000;
18'd293: data <= 18'b111111111100011011;
18'd294: data <= 18'b111111111100001010;
18'd295: data <= 18'b111111111011111011;
18'd296: data <= 18'b111111111100111100;
18'd297: data <= 18'b111111110111111111;
18'd298: data <= 18'b000000000001100001;
18'd299: data <= 18'b111111111110001100;
18'd300: data <= 18'b111111111010100011;
18'd301: data <= 18'b111111111111111011;
18'd302: data <= 18'b111111111111001000;
18'd303: data <= 18'b111111111011000000;
18'd304: data <= 18'b111111111100011110;
18'd305: data <= 18'b000000000000000101;
18'd306: data <= 18'b000000000000100111;
18'd307: data <= 18'b000000000000010010;
18'd308: data <= 18'b111111111100000011;
18'd309: data <= 18'b000000000000100011;
18'd310: data <= 18'b111111111011100111;
18'd311: data <= 18'b000000000001001101;
18'd312: data <= 18'b111111111111100000;
18'd313: data <= 18'b000000000010011110;
18'd314: data <= 18'b111111111111101101;
18'd315: data <= 18'b000000001000100011;
18'd316: data <= 18'b000000000011101010;
18'd317: data <= 18'b000000000110010111;
18'd318: data <= 18'b000000000001010100;
18'd319: data <= 18'b000000000101010011;
18'd320: data <= 18'b000000001000010101;
18'd321: data <= 18'b000000000011011000;
18'd322: data <= 18'b111111111110110110;
18'd323: data <= 18'b000000000100101000;
18'd324: data <= 18'b000000001000011001;
18'd325: data <= 18'b000000000110011111;
18'd326: data <= 18'b000000000111110101;
18'd327: data <= 18'b111111111110011011;
18'd328: data <= 18'b111111111001011100;
18'd329: data <= 18'b111111111100011010;
18'd330: data <= 18'b000000000000100111;
18'd331: data <= 18'b000000000001000110;
18'd332: data <= 18'b000000000001000100;
18'd333: data <= 18'b000000000100110010;
18'd334: data <= 18'b000000000110001100;
18'd335: data <= 18'b111111111111011001;
18'd336: data <= 18'b111111111111000100;
18'd337: data <= 18'b000000000000111100;
18'd338: data <= 18'b000000000101011111;
18'd339: data <= 18'b111111111011100101;
18'd340: data <= 18'b111111111100101011;
18'd341: data <= 18'b000000000011001111;
18'd342: data <= 18'b000000000110111000;
18'd343: data <= 18'b111111111110101010;
18'd344: data <= 18'b000000000011000001;
18'd345: data <= 18'b000000000011110011;
18'd346: data <= 18'b111111111011101100;
18'd347: data <= 18'b111111111010001010;
18'd348: data <= 18'b111111111110000100;
18'd349: data <= 18'b111111111110011000;
18'd350: data <= 18'b111111111010001101;
18'd351: data <= 18'b000000000010111000;
18'd352: data <= 18'b111111111100000010;
18'd353: data <= 18'b111111111110111100;
18'd354: data <= 18'b000000000011111111;
18'd355: data <= 18'b000000000011011001;
18'd356: data <= 18'b111111111011100101;
18'd357: data <= 18'b000000000001001011;
18'd358: data <= 18'b000000000011111101;
18'd359: data <= 18'b000000000000111011;
18'd360: data <= 18'b000000000011010001;
18'd361: data <= 18'b000000000100000101;
18'd362: data <= 18'b111111111000001011;
18'd363: data <= 18'b000000000010101010;
18'd364: data <= 18'b111111111001101110;
18'd365: data <= 18'b111111111001011100;
18'd366: data <= 18'b000000000000000011;
18'd367: data <= 18'b111111111101100011;
18'd368: data <= 18'b111111111011101111;
18'd369: data <= 18'b111111110111010001;
18'd370: data <= 18'b111111111110001011;
18'd371: data <= 18'b111111111101000000;
18'd372: data <= 18'b111111111001000100;
18'd373: data <= 18'b110000000000000000;
18'd374: data <= 18'b111111111000101000;
18'd375: data <= 18'b111111111110101001;
18'd376: data <= 18'b000000000000010010;
18'd377: data <= 18'b111111111000101101;
18'd378: data <= 18'b111111111111101000;
18'd379: data <= 18'b000000000010010011;
18'd380: data <= 18'b111111111111110000;
18'd381: data <= 18'b000000001000100011;
18'd382: data <= 18'b000000000110000101;
18'd383: data <= 18'b111111111101111000;
18'd384: data <= 18'b111111111010111100;
18'd385: data <= 18'b000000000010011000;
18'd386: data <= 18'b111111111110111011;
   

endcase
end
endmodule
