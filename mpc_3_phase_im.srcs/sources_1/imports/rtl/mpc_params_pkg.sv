`ifndef MPC_PARAMS_PKG_SV
`define MPC_PARAMS_PKG_SV

package mpc_params_pkg;

    parameter DATA_WIDTH = 32;              
    parameter FRAC_BITS  = 20;              

    parameter SYS_CLK_FREQ   = 100_000_000;  
    parameter SWITCHING_FREQ = 10_000;        
    parameter TS_COUNTER_MAX = SYS_CLK_FREQ / SWITCHING_FREQ;  

    parameter DEAD_TIME_NS     = 2000;        
    parameter DEAD_TIME_CYCLES = SYS_CLK_FREQ / (1_000_000_000 / DEAD_TIME_NS);

    parameter ADC_BITS     = 12;              
    parameter ADC_SCLK_DIV = 8;               
    parameter ADC_NUM_BITS = 16;             
    parameter ADC_OFFSET   = 2048;            

    parameter signed [DATA_WIDTH-1:0] ADC_SCALE = 32'sd10240;

    parameter ENCODER_PPR = 2500;              
    parameter ENCODER_CPR = ENCODER_PPR * 4;  
    parameter NUM_POLES   = 4;
    parameter ENCODER_Z_RESET = 1'b1;                

    parameter signed [DATA_WIDTH-1:0] SPEED_SCALE = 32'sd13176795;

    parameter signed [DATA_WIDTH-1:0] SPEED_ALPHA         = 32'sd104858;
    parameter signed [DATA_WIDTH-1:0] SPEED_ONE_MINUS_ALPHA = 32'sd943718;

    parameter signed [DATA_WIDTH-1:0] C11 = 32'sd1029293;
    parameter signed [DATA_WIDTH-1:0] C12 = 32'sd45224;
    parameter signed [DATA_WIDTH-1:0] C13 = 32'sd8758;
    parameter signed [DATA_WIDTH-1:0] D1  = 32'sd9016;
    parameter signed [DATA_WIDTH-1:0] E21 = 32'sd110;
    parameter signed [DATA_WIDTH-1:0] E22 = 32'sd1048034;
    parameter signed [DATA_WIDTH-1:0] TS_Q = 32'sd105;
    parameter signed [DATA_WIDTH-1:0] KT = 32'sd3055130;

    parameter VDC_DEFAULT_INT = 10'd311;      
    parameter VDC_COARSE_STEP = 10'd25;       
    parameter VDC_FINE_STEP   = 10'd2;        

    parameter signed [DATA_WIDTH-1:0] TWO_THIRDS = 32'sd699051;   
    parameter signed [DATA_WIDTH-1:0] ONE_THIRD  = 32'sd349525;   
    parameter signed [DATA_WIDTH-1:0] INV_SQRT3  = 32'sd605510;   

    parameter signed [DATA_WIDTH-1:0] LAMBDA_T = 32'sd1048576;    
    parameter signed [DATA_WIDTH-1:0] LAMBDA_PSI = 32'sd104857600; 

    parameter signed [DATA_WIDTH-1:0] PSI_REF_SQ = 32'sd966367;
    parameter signed [DATA_WIDTH-1:0] TE_REF_DEFAULT = 32'sd5242880;
    parameter signed [DATA_WIDTH-1:0] SPEED_REF_DEFAULT = 32'sd104857600; 
    
    parameter signed [DATA_WIDTH-1:0] PI_KP = 32'sd5242880;  
    parameter signed [DATA_WIDTH-1:0] PI_KI = 32'sd104857;   
    parameter signed [DATA_WIDTH-1:0] TE_MAX = 32'sd20971520; 
    parameter signed [DATA_WIDTH-1:0] TE_MIN = -32'sd20971520; 
    
    parameter MAX_CURRENT_RAW = 12'd3800;
    parameter MIN_CURRENT_RAW = 12'd200;
    parameter NUM_VECTORS = 8;                

endpackage
`endif