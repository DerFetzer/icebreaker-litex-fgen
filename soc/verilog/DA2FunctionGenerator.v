// Generator : SpinalHDL v1.4.0    git head : ecb5a80b713566f417ea3ea061f9969e73770a7f
// Date      : 03/08/2020, 18:47:58
// Component : DA2FunctionGenerator


`define sendFsm_enumDefinition_binary_sequential_type [2:0]
`define sendFsm_enumDefinition_binary_sequential_boot 3'b000
`define sendFsm_enumDefinition_binary_sequential_sendFsm_stateIdle 3'b001
`define sendFsm_enumDefinition_binary_sequential_sendFsm_stateSSActivate 3'b010
`define sendFsm_enumDefinition_binary_sequential_sendFsm_stateSSDeactivate 3'b011
`define sendFsm_enumDefinition_binary_sequential_sendFsm_stateData 3'b100


module SpiXdrMasterCtrl (
  input               io_config_kind_cpol,
  input               io_config_kind_cpha,
  input      [3:0]    io_config_sclkToogle,
  input      [0:0]    io_config_ss_activeHigh,
  input      [3:0]    io_config_ss_setup,
  input      [3:0]    io_config_ss_hold,
  input      [3:0]    io_config_ss_disable,
  input               io_cmd_valid,
  output reg          io_cmd_ready,
  input               io_cmd_payload_kind,
  input               io_cmd_payload_read,
  input               io_cmd_payload_write,
  input      [15:0]   io_cmd_payload_data,
  output              io_rsp_valid,
  output     [15:0]   io_rsp_payload_data,
  output     [0:0]    io_spi_sclk_write,
  output reg          io_spi_data_0_writeEnable,
  input      [0:0]    io_spi_data_0_read,
  output reg [0:0]    io_spi_data_0_write,
  output              io_spi_data_1_writeEnable,
  input      [0:0]    io_spi_data_1_read,
  output     [0:0]    io_spi_data_1_write,
  output     [0:0]    io_spi_ss,
  input               clk,
  input               reset 
);
  reg        [0:0]    _zz_3_;
  wire                _zz_4_;
  wire                _zz_5_;
  wire                _zz_6_;
  wire                _zz_7_;
  wire                _zz_8_;
  wire                _zz_9_;
  wire       [3:0]    _zz_10_;
  wire       [3:0]    _zz_11_;
  wire       [1:0]    _zz_12_;
  reg        [3:0]    timer_counter;
  reg                 timer_reset;
  wire                timer_ss_setupHit;
  wire                timer_ss_holdHit;
  wire                timer_ss_disableHit;
  wire                timer_sclkToogleHit;
  reg                 fsm_state;
  reg        [3:0]    fsm_counter;
  reg        [0:0]    _zz_1_;
  wire       [3:0]    fsm_counterPlus;
  reg                 fsm_fastRate;
  reg                 fsm_isDdr;
  reg                 fsm_readFill;
  reg                 fsm_readDone;
  reg        [0:0]    fsm_ss;
  reg        [0:0]    outputPhy_sclkWrite;
  wire       [0:0]    _zz_2_;
  reg        [0:0]    outputPhy_dataWrite;
  reg        [0:0]    outputPhy_widthSel;
  reg                 fsm_readFill_delay_1;
  reg                 inputPhy_readFill;
  reg                 fsm_readDone_delay_1;
  reg                 inputPhy_readDone;
  reg        [14:0]   inputPhy_buffer;
  reg        [15:0]   inputPhy_bufferNext;
  reg        [0:0]    inputPhy_widthSel;
  wire       [0:0]    inputPhy_dataWrite;
  reg        [0:0]    inputPhy_dataRead;
  reg                 fsm_state_delay_1;
  reg                 fsm_state_delay_2;
  reg        [1:0]    inputPhy_dataReadBuffer;
  function [0:0] zz__zz_1_(input dummy);
    begin
      zz__zz_1_ = (1'bx);
      zz__zz_1_ = (1'b1);
    end
  endfunction
  wire [0:0] _zz_13_;
  function  zz_fsm_fastRate(input dummy);
    begin
      zz_fsm_fastRate = 1'bx;
      zz_fsm_fastRate = 1'b0;
    end
  endfunction
  wire  _zz_14_;
  function  zz_fsm_isDdr(input dummy);
    begin
      zz_fsm_isDdr = 1'bx;
      zz_fsm_isDdr = 1'b0;
    end
  endfunction
  wire  _zz_15_;
  function [0:0] zz_outputPhy_widthSel(input dummy);
    begin
      zz_outputPhy_widthSel = (1'bx);
      zz_outputPhy_widthSel = (1'b0);
    end
  endfunction
  wire [0:0] _zz_16_;
  function  zz_io_spi_data_0_writeEnable(input dummy);
    begin
      zz_io_spi_data_0_writeEnable = 1'b0;
      zz_io_spi_data_0_writeEnable = 1'b1;
    end
  endfunction
  wire  _zz_17_;
  function [0:0] zz_inputPhy_widthSel(input dummy);
    begin
      zz_inputPhy_widthSel = (1'bx);
      zz_inputPhy_widthSel = (1'b0);
    end
  endfunction
  wire [0:0] _zz_18_;

  assign _zz_4_ = (! io_cmd_payload_kind);
  assign _zz_5_ = io_cmd_payload_data[15];
  assign _zz_6_ = (! fsm_state);
  assign _zz_7_ = ((! io_cmd_valid) || io_cmd_ready);
  assign _zz_8_ = ((timer_sclkToogleHit && (fsm_state || fsm_isDdr)) || fsm_fastRate);
  assign _zz_9_ = (fsm_counterPlus == (4'b0000));
  assign _zz_10_ = {3'd0, _zz_1_};
  assign _zz_11_ = (fsm_counter >>> 0);
  assign _zz_12_ = {io_spi_data_1_read[0],io_spi_data_0_read[0]};
  always @(*) begin
    case(_zz_11_)
      4'b0000 : begin
        _zz_3_ = io_cmd_payload_data[15 : 15];
      end
      4'b0001 : begin
        _zz_3_ = io_cmd_payload_data[14 : 14];
      end
      4'b0010 : begin
        _zz_3_ = io_cmd_payload_data[13 : 13];
      end
      4'b0011 : begin
        _zz_3_ = io_cmd_payload_data[12 : 12];
      end
      4'b0100 : begin
        _zz_3_ = io_cmd_payload_data[11 : 11];
      end
      4'b0101 : begin
        _zz_3_ = io_cmd_payload_data[10 : 10];
      end
      4'b0110 : begin
        _zz_3_ = io_cmd_payload_data[9 : 9];
      end
      4'b0111 : begin
        _zz_3_ = io_cmd_payload_data[8 : 8];
      end
      4'b1000 : begin
        _zz_3_ = io_cmd_payload_data[7 : 7];
      end
      4'b1001 : begin
        _zz_3_ = io_cmd_payload_data[6 : 6];
      end
      4'b1010 : begin
        _zz_3_ = io_cmd_payload_data[5 : 5];
      end
      4'b1011 : begin
        _zz_3_ = io_cmd_payload_data[4 : 4];
      end
      4'b1100 : begin
        _zz_3_ = io_cmd_payload_data[3 : 3];
      end
      4'b1101 : begin
        _zz_3_ = io_cmd_payload_data[2 : 2];
      end
      4'b1110 : begin
        _zz_3_ = io_cmd_payload_data[1 : 1];
      end
      default : begin
        _zz_3_ = io_cmd_payload_data[0 : 0];
      end
    endcase
  end

  always @ (*) begin
    timer_reset = 1'b0;
    if(io_cmd_valid)begin
      if(_zz_4_)begin
        timer_reset = timer_sclkToogleHit;
      end else begin
        if(! _zz_5_) begin
          if(_zz_6_)begin
            if(timer_ss_holdHit)begin
              timer_reset = 1'b1;
            end
          end
        end
      end
    end
    if(_zz_7_)begin
      timer_reset = 1'b1;
    end
  end

  assign timer_ss_setupHit = (timer_counter == io_config_ss_setup);
  assign timer_ss_holdHit = (timer_counter == io_config_ss_hold);
  assign timer_ss_disableHit = (timer_counter == io_config_ss_disable);
  assign timer_sclkToogleHit = (timer_counter == io_config_sclkToogle);
  assign _zz_13_ = zz__zz_1_(1'b0);
  always @ (*) _zz_1_ = _zz_13_;
  assign fsm_counterPlus = (fsm_counter + _zz_10_);
  assign _zz_14_ = zz_fsm_fastRate(1'b0);
  always @ (*) fsm_fastRate = _zz_14_;
  assign _zz_15_ = zz_fsm_isDdr(1'b0);
  always @ (*) fsm_isDdr = _zz_15_;
  always @ (*) begin
    fsm_readFill = 1'b0;
    if(io_cmd_valid)begin
      if(_zz_4_)begin
        if(_zz_8_)begin
          fsm_readFill = 1'b1;
        end
      end
    end
  end

  always @ (*) begin
    fsm_readDone = 1'b0;
    if(io_cmd_valid)begin
      if(_zz_4_)begin
        if(_zz_8_)begin
          if(_zz_9_)begin
            fsm_readDone = io_cmd_payload_read;
          end
        end
      end
    end
  end

  assign io_spi_ss = (~ (fsm_ss ^ io_config_ss_activeHigh));
  always @ (*) begin
    io_cmd_ready = 1'b0;
    if(io_cmd_valid)begin
      if(_zz_4_)begin
        if(_zz_8_)begin
          if(_zz_9_)begin
            io_cmd_ready = 1'b1;
          end
        end
      end else begin
        if(_zz_5_)begin
          if(timer_ss_setupHit)begin
            io_cmd_ready = 1'b1;
          end
        end else begin
          if(! _zz_6_) begin
            if(timer_ss_disableHit)begin
              io_cmd_ready = 1'b1;
            end
          end
        end
      end
    end
  end

  always @ (*) begin
    outputPhy_sclkWrite = (1'b0);
    if((io_cmd_valid && (! io_cmd_payload_kind)))begin
      outputPhy_sclkWrite = ((fsm_state ^ io_config_kind_cpha) ? (1'b1) : (1'b0));
    end
  end

  assign _zz_2_[0] = io_config_kind_cpol;
  assign io_spi_sclk_write = (outputPhy_sclkWrite ^ _zz_2_);
  assign _zz_16_ = zz_outputPhy_widthSel(1'b0);
  always @ (*) outputPhy_widthSel = _zz_16_;
  always @ (*) begin
    outputPhy_dataWrite = (1'bx);
    case(outputPhy_widthSel)
      1'b0 : begin
        outputPhy_dataWrite[0 : 0] = _zz_3_;
      end
      default : begin
      end
    endcase
  end

  assign _zz_17_ = zz_io_spi_data_0_writeEnable(1'b0);
  always @ (*) io_spi_data_0_writeEnable = _zz_17_;
  assign io_spi_data_1_writeEnable = 1'b0;
  always @ (*) begin
    io_spi_data_0_write = (1'bx);
    io_spi_data_0_write[0] = (outputPhy_dataWrite[0] || (! (io_cmd_valid && io_cmd_payload_write)));
  end

  assign io_spi_data_1_write = (1'bx);
  always @ (*) begin
    inputPhy_bufferNext = 16'h0;
    case(inputPhy_widthSel)
      1'b0 : begin
        inputPhy_bufferNext = {inputPhy_buffer,inputPhy_dataRead[0 : 0]};
      end
      default : begin
      end
    endcase
  end

  assign _zz_18_ = zz_inputPhy_widthSel(1'b0);
  always @ (*) inputPhy_widthSel = _zz_18_;
  always @ (*) begin
    inputPhy_dataRead = (1'bx);
    inputPhy_dataRead[0] = _zz_12_[1];
  end

  assign io_rsp_valid = inputPhy_readDone;
  assign io_rsp_payload_data = inputPhy_bufferNext;
  always @ (posedge clk) begin
    timer_counter <= (timer_counter + (4'b0001));
    if(timer_reset)begin
      timer_counter <= (4'b0000);
    end
    fsm_state_delay_1 <= fsm_state;
    fsm_state_delay_2 <= fsm_state_delay_1;
    if((! fsm_state_delay_2))begin
      inputPhy_dataReadBuffer <= {io_spi_data_1_read[0],io_spi_data_0_read[0]};
    end
    case(inputPhy_widthSel)
      1'b0 : begin
        if(inputPhy_readFill)begin
          inputPhy_buffer <= inputPhy_bufferNext[14:0];
        end
      end
      default : begin
      end
    endcase
  end

  always @ (posedge clk or posedge reset) begin
    if (reset) begin
      fsm_state <= 1'b0;
      fsm_counter <= (4'b0000);
      fsm_ss <= (1'b0);
      fsm_readFill_delay_1 <= 1'b0;
      inputPhy_readFill <= 1'b0;
      fsm_readDone_delay_1 <= 1'b0;
      inputPhy_readDone <= 1'b0;
    end else begin
      if(io_cmd_valid)begin
        if(_zz_4_)begin
          if(timer_sclkToogleHit)begin
            fsm_state <= (! fsm_state);
          end
          if(_zz_8_)begin
            fsm_counter <= fsm_counterPlus;
            if(_zz_9_)begin
              fsm_state <= 1'b0;
            end
          end
        end else begin
          if(_zz_5_)begin
            fsm_ss[0] <= 1'b1;
          end else begin
            if(_zz_6_)begin
              if(timer_ss_holdHit)begin
                fsm_state <= 1'b1;
              end
            end else begin
              fsm_ss[0] <= 1'b0;
            end
          end
        end
      end
      if(_zz_7_)begin
        fsm_state <= 1'b0;
        fsm_counter <= (4'b0000);
      end
      fsm_readFill_delay_1 <= fsm_readFill;
      inputPhy_readFill <= fsm_readFill_delay_1;
      fsm_readDone_delay_1 <= fsm_readDone;
      inputPhy_readDone <= fsm_readDone_delay_1;
    end
  end


endmodule
//SpiXdrMasterCtrl_1_ replaced by SpiXdrMasterCtrl

module DA2 (
  input               io_active,
  input               io_write,
  input               io_data1_valid,
  output reg          io_data1_ready,
  input      [15:0]   io_data1_payload,
  input               io_data2_valid,
  output reg          io_data2_ready,
  input      [15:0]   io_data2_payload,
  output reg          io_busy,
  output              io_sync,
  output              io_dina,
  output              io_dinb,
  output              io_sclk,
  input               clk,
  input               reset 
);
  reg                 _zz_1_;
  reg                 _zz_2_;
  wire                _zz_3_;
  wire                _zz_4_;
  reg        [15:0]   _zz_5_;
  wire       [0:0]    _zz_6_;
  wire       [0:0]    _zz_7_;
  reg                 _zz_8_;
  reg                 _zz_9_;
  wire                _zz_10_;
  wire                _zz_11_;
  reg        [15:0]   _zz_12_;
  wire       [0:0]    _zz_13_;
  wire       [0:0]    _zz_14_;
  wire                spi1_io_cmd_ready;
  wire                spi1_io_rsp_valid;
  wire       [15:0]   spi1_io_rsp_payload_data;
  wire       [0:0]    spi1_io_spi_sclk_write;
  wire       [0:0]    spi1_io_spi_ss;
  wire       [0:0]    spi1_io_spi_data_0_write;
  wire                spi1_io_spi_data_0_writeEnable;
  wire       [0:0]    spi1_io_spi_data_1_write;
  wire                spi1_io_spi_data_1_writeEnable;
  wire                spi2_io_cmd_ready;
  wire                spi2_io_rsp_valid;
  wire       [15:0]   spi2_io_rsp_payload_data;
  wire       [0:0]    spi2_io_spi_sclk_write;
  wire       [0:0]    spi2_io_spi_ss;
  wire       [0:0]    spi2_io_spi_data_0_write;
  wire                spi2_io_spi_data_0_writeEnable;
  wire       [0:0]    spi2_io_spi_data_1_write;
  wire                spi2_io_spi_data_1_writeEnable;
  wire                _zz_15_;
  wire                spiConfig_kind_cpol;
  wire                spiConfig_kind_cpha;
  wire       [3:0]    spiConfig_sclkToogle;
  wire       [0:0]    spiConfig_ss_activeHigh;
  wire       [3:0]    spiConfig_ss_setup;
  wire       [3:0]    spiConfig_ss_hold;
  wire       [3:0]    spiConfig_ss_disable;
  wire                sendFsm_wantExit;
  reg        [15:0]   sendFsm_data1Reg;
  reg        [15:0]   sendFsm_data2Reg;
  reg        `sendFsm_enumDefinition_binary_sequential_type sendFsm_stateReg;
  reg        `sendFsm_enumDefinition_binary_sequential_type sendFsm_stateNext;
  reg                 io_write_regNext;
  `ifndef SYNTHESIS
  reg [199:0] sendFsm_stateReg_string;
  reg [199:0] sendFsm_stateNext_string;
  `endif


  assign _zz_15_ = ((((io_active && io_data1_valid) && io_data2_valid) && io_active) && (io_write && (! io_write_regNext)));
  SpiXdrMasterCtrl spi1 ( 
    .io_config_kind_cpol          (spiConfig_kind_cpol             ), //i
    .io_config_kind_cpha          (spiConfig_kind_cpha             ), //i
    .io_config_sclkToogle         (spiConfig_sclkToogle[3:0]       ), //i
    .io_config_ss_activeHigh      (spiConfig_ss_activeHigh         ), //i
    .io_config_ss_setup           (spiConfig_ss_setup[3:0]         ), //i
    .io_config_ss_hold            (spiConfig_ss_hold[3:0]          ), //i
    .io_config_ss_disable         (spiConfig_ss_disable[3:0]       ), //i
    .io_cmd_valid                 (_zz_1_                          ), //i
    .io_cmd_ready                 (spi1_io_cmd_ready               ), //o
    .io_cmd_payload_kind          (_zz_2_                          ), //i
    .io_cmd_payload_read          (_zz_3_                          ), //i
    .io_cmd_payload_write         (_zz_4_                          ), //i
    .io_cmd_payload_data          (_zz_5_[15:0]                    ), //i
    .io_rsp_valid                 (spi1_io_rsp_valid               ), //o
    .io_rsp_payload_data          (spi1_io_rsp_payload_data[15:0]  ), //o
    .io_spi_sclk_write            (spi1_io_spi_sclk_write          ), //o
    .io_spi_data_0_writeEnable    (spi1_io_spi_data_0_writeEnable  ), //o
    .io_spi_data_0_read           (_zz_6_                          ), //i
    .io_spi_data_0_write          (spi1_io_spi_data_0_write        ), //o
    .io_spi_data_1_writeEnable    (spi1_io_spi_data_1_writeEnable  ), //o
    .io_spi_data_1_read           (_zz_7_                          ), //i
    .io_spi_data_1_write          (spi1_io_spi_data_1_write        ), //o
    .io_spi_ss                    (spi1_io_spi_ss                  ), //o
    .clk                          (clk                             ), //i
    .reset                        (reset                           )  //i
  );
  SpiXdrMasterCtrl spi2 ( 
    .io_config_kind_cpol          (spiConfig_kind_cpol             ), //i
    .io_config_kind_cpha          (spiConfig_kind_cpha             ), //i
    .io_config_sclkToogle         (spiConfig_sclkToogle[3:0]       ), //i
    .io_config_ss_activeHigh      (spiConfig_ss_activeHigh         ), //i
    .io_config_ss_setup           (spiConfig_ss_setup[3:0]         ), //i
    .io_config_ss_hold            (spiConfig_ss_hold[3:0]          ), //i
    .io_config_ss_disable         (spiConfig_ss_disable[3:0]       ), //i
    .io_cmd_valid                 (_zz_8_                          ), //i
    .io_cmd_ready                 (spi2_io_cmd_ready               ), //o
    .io_cmd_payload_kind          (_zz_9_                          ), //i
    .io_cmd_payload_read          (_zz_10_                         ), //i
    .io_cmd_payload_write         (_zz_11_                         ), //i
    .io_cmd_payload_data          (_zz_12_[15:0]                   ), //i
    .io_rsp_valid                 (spi2_io_rsp_valid               ), //o
    .io_rsp_payload_data          (spi2_io_rsp_payload_data[15:0]  ), //o
    .io_spi_sclk_write            (spi2_io_spi_sclk_write          ), //o
    .io_spi_data_0_writeEnable    (spi2_io_spi_data_0_writeEnable  ), //o
    .io_spi_data_0_read           (_zz_13_                         ), //i
    .io_spi_data_0_write          (spi2_io_spi_data_0_write        ), //o
    .io_spi_data_1_writeEnable    (spi2_io_spi_data_1_writeEnable  ), //o
    .io_spi_data_1_read           (_zz_14_                         ), //i
    .io_spi_data_1_write          (spi2_io_spi_data_1_write        ), //o
    .io_spi_ss                    (spi2_io_spi_ss                  ), //o
    .clk                          (clk                             ), //i
    .reset                        (reset                           )  //i
  );
  `ifndef SYNTHESIS
  always @(*) begin
    case(sendFsm_stateReg)
      `sendFsm_enumDefinition_binary_sequential_boot : sendFsm_stateReg_string = "boot                     ";
      `sendFsm_enumDefinition_binary_sequential_sendFsm_stateIdle : sendFsm_stateReg_string = "sendFsm_stateIdle        ";
      `sendFsm_enumDefinition_binary_sequential_sendFsm_stateSSActivate : sendFsm_stateReg_string = "sendFsm_stateSSActivate  ";
      `sendFsm_enumDefinition_binary_sequential_sendFsm_stateSSDeactivate : sendFsm_stateReg_string = "sendFsm_stateSSDeactivate";
      `sendFsm_enumDefinition_binary_sequential_sendFsm_stateData : sendFsm_stateReg_string = "sendFsm_stateData        ";
      default : sendFsm_stateReg_string = "?????????????????????????";
    endcase
  end
  always @(*) begin
    case(sendFsm_stateNext)
      `sendFsm_enumDefinition_binary_sequential_boot : sendFsm_stateNext_string = "boot                     ";
      `sendFsm_enumDefinition_binary_sequential_sendFsm_stateIdle : sendFsm_stateNext_string = "sendFsm_stateIdle        ";
      `sendFsm_enumDefinition_binary_sequential_sendFsm_stateSSActivate : sendFsm_stateNext_string = "sendFsm_stateSSActivate  ";
      `sendFsm_enumDefinition_binary_sequential_sendFsm_stateSSDeactivate : sendFsm_stateNext_string = "sendFsm_stateSSDeactivate";
      `sendFsm_enumDefinition_binary_sequential_sendFsm_stateData : sendFsm_stateNext_string = "sendFsm_stateData        ";
      default : sendFsm_stateNext_string = "?????????????????????????";
    endcase
  end
  `endif

  assign spiConfig_sclkToogle = (4'b0000);
  assign spiConfig_kind_cpol = 1'b0;
  assign spiConfig_kind_cpha = 1'b1;
  assign spiConfig_ss_activeHigh = (1'b0);
  assign spiConfig_ss_disable = (4'b0000);
  assign spiConfig_ss_hold = (4'b0001);
  assign spiConfig_ss_setup = (4'b0001);
  assign io_sync = spi1_io_spi_ss[0];
  assign io_sclk = spi1_io_spi_sclk_write[0];
  assign io_dina = spi1_io_spi_data_0_write[0];
  assign io_dinb = spi2_io_spi_data_0_write[0];
  assign sendFsm_wantExit = 1'b0;
  always @ (*) begin
    _zz_2_ = 1'b0;
    case(sendFsm_stateReg)
      `sendFsm_enumDefinition_binary_sequential_sendFsm_stateIdle : begin
      end
      `sendFsm_enumDefinition_binary_sequential_sendFsm_stateSSActivate : begin
        _zz_2_ = 1'b1;
      end
      `sendFsm_enumDefinition_binary_sequential_sendFsm_stateSSDeactivate : begin
        _zz_2_ = 1'b1;
      end
      `sendFsm_enumDefinition_binary_sequential_sendFsm_stateData : begin
        _zz_2_ = 1'b0;
      end
      default : begin
      end
    endcase
  end

  assign _zz_4_ = 1'b1;
  always @ (*) begin
    _zz_5_ = 16'hffff;
    case(sendFsm_stateReg)
      `sendFsm_enumDefinition_binary_sequential_sendFsm_stateIdle : begin
      end
      `sendFsm_enumDefinition_binary_sequential_sendFsm_stateSSActivate : begin
        _zz_5_[15] = 1'b1;
        _zz_5_[0] = 1'b1;
      end
      `sendFsm_enumDefinition_binary_sequential_sendFsm_stateSSDeactivate : begin
        _zz_5_ = 16'h0;
      end
      `sendFsm_enumDefinition_binary_sequential_sendFsm_stateData : begin
        _zz_5_ = sendFsm_data1Reg;
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    _zz_1_ = 1'b0;
    case(sendFsm_stateReg)
      `sendFsm_enumDefinition_binary_sequential_sendFsm_stateIdle : begin
        _zz_1_ = 1'b0;
      end
      `sendFsm_enumDefinition_binary_sequential_sendFsm_stateSSActivate : begin
        _zz_1_ = 1'b1;
      end
      `sendFsm_enumDefinition_binary_sequential_sendFsm_stateSSDeactivate : begin
        _zz_1_ = 1'b1;
      end
      `sendFsm_enumDefinition_binary_sequential_sendFsm_stateData : begin
        _zz_1_ = 1'b1;
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    _zz_9_ = 1'b0;
    case(sendFsm_stateReg)
      `sendFsm_enumDefinition_binary_sequential_sendFsm_stateIdle : begin
      end
      `sendFsm_enumDefinition_binary_sequential_sendFsm_stateSSActivate : begin
        _zz_9_ = 1'b1;
      end
      `sendFsm_enumDefinition_binary_sequential_sendFsm_stateSSDeactivate : begin
        _zz_9_ = 1'b1;
      end
      `sendFsm_enumDefinition_binary_sequential_sendFsm_stateData : begin
        _zz_9_ = 1'b0;
      end
      default : begin
      end
    endcase
  end

  assign _zz_11_ = 1'b1;
  always @ (*) begin
    _zz_12_ = 16'hffff;
    case(sendFsm_stateReg)
      `sendFsm_enumDefinition_binary_sequential_sendFsm_stateIdle : begin
      end
      `sendFsm_enumDefinition_binary_sequential_sendFsm_stateSSActivate : begin
        _zz_12_[15] = 1'b1;
        _zz_12_[0] = 1'b1;
      end
      `sendFsm_enumDefinition_binary_sequential_sendFsm_stateSSDeactivate : begin
        _zz_12_ = 16'h0;
      end
      `sendFsm_enumDefinition_binary_sequential_sendFsm_stateData : begin
        _zz_12_ = sendFsm_data2Reg;
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    _zz_8_ = 1'b0;
    case(sendFsm_stateReg)
      `sendFsm_enumDefinition_binary_sequential_sendFsm_stateIdle : begin
        _zz_8_ = 1'b0;
      end
      `sendFsm_enumDefinition_binary_sequential_sendFsm_stateSSActivate : begin
        _zz_8_ = 1'b1;
      end
      `sendFsm_enumDefinition_binary_sequential_sendFsm_stateSSDeactivate : begin
        _zz_8_ = 1'b1;
      end
      `sendFsm_enumDefinition_binary_sequential_sendFsm_stateData : begin
        _zz_8_ = 1'b1;
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    io_data1_ready = 1'b0;
    case(sendFsm_stateReg)
      `sendFsm_enumDefinition_binary_sequential_sendFsm_stateIdle : begin
      end
      `sendFsm_enumDefinition_binary_sequential_sendFsm_stateSSActivate : begin
      end
      `sendFsm_enumDefinition_binary_sequential_sendFsm_stateSSDeactivate : begin
      end
      `sendFsm_enumDefinition_binary_sequential_sendFsm_stateData : begin
        io_data1_ready = 1'b1;
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    io_data2_ready = 1'b0;
    case(sendFsm_stateReg)
      `sendFsm_enumDefinition_binary_sequential_sendFsm_stateIdle : begin
      end
      `sendFsm_enumDefinition_binary_sequential_sendFsm_stateSSActivate : begin
      end
      `sendFsm_enumDefinition_binary_sequential_sendFsm_stateSSDeactivate : begin
      end
      `sendFsm_enumDefinition_binary_sequential_sendFsm_stateData : begin
        io_data2_ready = 1'b1;
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    io_busy = 1'b0;
    case(sendFsm_stateReg)
      `sendFsm_enumDefinition_binary_sequential_sendFsm_stateIdle : begin
        io_busy = 1'b0;
      end
      `sendFsm_enumDefinition_binary_sequential_sendFsm_stateSSActivate : begin
        io_busy = 1'b1;
      end
      `sendFsm_enumDefinition_binary_sequential_sendFsm_stateSSDeactivate : begin
        io_busy = 1'b1;
      end
      `sendFsm_enumDefinition_binary_sequential_sendFsm_stateData : begin
        io_busy = 1'b1;
      end
      default : begin
      end
    endcase
  end

  always @ (*) begin
    sendFsm_stateNext = sendFsm_stateReg;
    case(sendFsm_stateReg)
      `sendFsm_enumDefinition_binary_sequential_sendFsm_stateIdle : begin
        if(_zz_15_)begin
          sendFsm_stateNext = `sendFsm_enumDefinition_binary_sequential_sendFsm_stateSSActivate;
        end
      end
      `sendFsm_enumDefinition_binary_sequential_sendFsm_stateSSActivate : begin
        if(spi1_io_cmd_ready)begin
          sendFsm_stateNext = `sendFsm_enumDefinition_binary_sequential_sendFsm_stateData;
        end
      end
      `sendFsm_enumDefinition_binary_sequential_sendFsm_stateSSDeactivate : begin
        if(spi1_io_cmd_ready)begin
          sendFsm_stateNext = `sendFsm_enumDefinition_binary_sequential_sendFsm_stateIdle;
        end
      end
      `sendFsm_enumDefinition_binary_sequential_sendFsm_stateData : begin
        if(spi1_io_cmd_ready)begin
          sendFsm_stateNext = `sendFsm_enumDefinition_binary_sequential_sendFsm_stateSSDeactivate;
        end
      end
      default : begin
        sendFsm_stateNext = `sendFsm_enumDefinition_binary_sequential_sendFsm_stateIdle;
      end
    endcase
  end

  always @ (posedge clk or posedge reset) begin
    if (reset) begin
      sendFsm_stateReg <= `sendFsm_enumDefinition_binary_sequential_boot;
    end else begin
      sendFsm_stateReg <= sendFsm_stateNext;
    end
  end

  always @ (posedge clk) begin
    io_write_regNext <= io_write;
  end

  always @ (posedge clk) begin
    case(sendFsm_stateReg)
      `sendFsm_enumDefinition_binary_sequential_sendFsm_stateIdle : begin
        if(_zz_15_)begin
          sendFsm_data1Reg <= io_data1_payload;
          sendFsm_data2Reg <= io_data2_payload;
        end
      end
      `sendFsm_enumDefinition_binary_sequential_sendFsm_stateSSActivate : begin
      end
      `sendFsm_enumDefinition_binary_sequential_sendFsm_stateSSDeactivate : begin
      end
      `sendFsm_enumDefinition_binary_sequential_sendFsm_stateData : begin
      end
      default : begin
      end
    endcase
  end


endmodule

module DA2FunctionGenerator (
  input               io_enable,
  input      [31:0]   io_prescaler,
  input      [16:0]   io_fcw,
  input               io_lutEnable,
  input               io_lutWrite,
  input      [10:0]   io_lutAddress,
  input      [11:0]   io_lutWriteData,
  output     [11:0]   io_lutReadData,
  output              io_sync,
  output              io_dina,
  output              io_dinb,
  output              io_sclk,
  input               clk,
  input               reset 
);
  reg                 _zz_3_;
  wire                _zz_4_;
  reg        [15:0]   _zz_5_;
  wire                _zz_6_;
  wire       [15:0]   _zz_7_;
  reg        [11:0]   _zz_8_;
  reg        [11:0]   _zz_9_;
  wire                da2_1__io_data1_ready;
  wire                da2_1__io_data2_ready;
  wire                da2_1__io_busy;
  wire                da2_1__io_sync;
  wire                da2_1__io_dina;
  wire                da2_1__io_dinb;
  wire                da2_1__io_sclk;
  wire                _zz_10_;
  wire       [0:0]    _zz_11_;
  wire       [31:0]   _zz_12_;
  wire       [31:0]   _zz_13_;
  wire                _zz_14_;
  reg                 cnt_willIncrement;
  reg                 cnt_willClear;
  reg        [31:0]   cnt_valueNext;
  reg        [31:0]   cnt_value;
  wire                cnt_willOverflowIfInc;
  wire                cnt_willOverflow;
  wire       [11:0]   _zz_1_;
  reg        [16:0]   phase;
  wire       [10:0]   _zz_2_;
  reg [11:0] rom [0:2047];

  assign _zz_10_ = (cnt_value == _zz_13_);
  assign _zz_11_ = cnt_willIncrement;
  assign _zz_12_ = {31'd0, _zz_11_};
  assign _zz_13_ = (io_prescaler - 32'h00000001);
  assign _zz_14_ = 1'b1;
  initial begin
    $readmemb("../../verilog/DA2FunctionGenerator.v_toplevel_rom.bin",rom);
  end
  always @ (posedge clk) begin
    if(io_lutEnable) begin
      _zz_8_ <= rom[io_lutAddress];
    end
  end

  always @ (posedge clk) begin
    if(io_lutEnable && io_lutWrite ) begin
      rom[io_lutAddress] <= _zz_1_;
    end
  end

  always @ (posedge clk) begin
    if(_zz_14_) begin
      _zz_9_ <= rom[_zz_2_];
    end
  end

  DA2 da2_1_ ( 
    .io_active           (io_enable              ), //i
    .io_write            (_zz_3_                 ), //i
    .io_data1_valid      (_zz_4_                 ), //i
    .io_data1_ready      (da2_1__io_data1_ready  ), //o
    .io_data1_payload    (_zz_5_[15:0]           ), //i
    .io_data2_valid      (_zz_6_                 ), //i
    .io_data2_ready      (da2_1__io_data2_ready  ), //o
    .io_data2_payload    (_zz_7_[15:0]           ), //i
    .io_busy             (da2_1__io_busy         ), //o
    .io_sync             (da2_1__io_sync         ), //o
    .io_dina             (da2_1__io_dina         ), //o
    .io_dinb             (da2_1__io_dinb         ), //o
    .io_sclk             (da2_1__io_sclk         ), //o
    .clk                 (clk                    ), //i
    .reset               (reset                  )  //i
  );
  always @ (*) begin
    cnt_willIncrement = 1'b0;
    if(io_enable)begin
      cnt_willIncrement = 1'b1;
    end else begin
      cnt_willIncrement = 1'b0;
    end
  end

  always @ (*) begin
    cnt_willClear = 1'b0;
    if(! io_enable) begin
      cnt_willClear = 1'b1;
    end
    if(_zz_10_)begin
      cnt_willClear = 1'b1;
    end
  end

  assign cnt_willOverflowIfInc = (cnt_value == 32'hffffffff);
  assign cnt_willOverflow = (cnt_willOverflowIfInc && cnt_willIncrement);
  always @ (*) begin
    cnt_valueNext = (cnt_value + _zz_12_);
    if(cnt_willClear)begin
      cnt_valueNext = 32'h0;
    end
  end

  assign _zz_1_ = io_lutWriteData;
  assign io_lutReadData = _zz_8_;
  always @ (*) begin
    if(_zz_10_)begin
      _zz_3_ = 1'b1;
    end else begin
      _zz_3_ = 1'b0;
    end
  end

  assign io_sync = da2_1__io_sync;
  assign io_dina = da2_1__io_dina;
  assign io_dinb = da2_1__io_dinb;
  assign io_sclk = da2_1__io_sclk;
  always @ (*) begin
    _zz_5_[15 : 12] = (1'b0 ? (4'b1111) : (4'b0000));
    _zz_5_[11 : 0] = _zz_9_;
  end

  assign _zz_2_ = (phase >>> 6);
  assign _zz_4_ = 1'b1;
  assign _zz_7_ = (1'b0 ? 16'hffff : 16'h0);
  assign _zz_6_ = 1'b1;
  always @ (posedge clk or posedge reset) begin
    if (reset) begin
      cnt_value <= 32'h0;
      phase <= 17'h0;
    end else begin
      cnt_value <= cnt_valueNext;
      if(_zz_10_)begin
        phase <= (phase + io_fcw);
      end
    end
  end


endmodule
