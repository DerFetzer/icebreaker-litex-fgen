# This file is Copyright (c) 2020 DerFetzer <kontakt@der-fetzer.de>
# License: BSD

import os

from migen import *
from migen.fhdl.specials import _MemoryPort

from litex.soc.interconnect.csr import *
from litex.soc.interconnect import wishbone
from litex.soc.integration.doc import AutoDoc, ModuleDoc


class FunctionGenerator(Module, AutoCSR, AutoDoc):
    def __init__(self, platform, sync_pin, dina_pin, dinb_pin, sclk_pin):
        self.bus = bus = wishbone.Interface()

        enable = Signal()
        prescaler = Signal(32)
        fcw = Signal(17)

        lut_write = Signal()
        lut_address = Signal(11)
        lut_write_data = Signal(12)
        lut_read_data = Signal(12)

        self._en = CSRStorage(fields=[CSRField("enable", description="Enable function generator")])
        self._fcw = CSRStorage(size=17, fields=[CSRField("fcw", size=17, description="Function generator frequency control word")])
        self._prescaler = CSRStorage(size=32, fields=[CSRField("prescaler", size=32, description="Function generator clock prescaler")])

        self._lut = LutMemory(_MemoryPort(lut_address, lut_read_data, lut_write, lut_write_data))

        platform.add_source_dir(os.path.join("verilog"))

        self.specials += Instance("DA2FunctionGenerator",
                                  i_io_enable=enable,
                                  i_io_prescaler=prescaler,
                                  i_io_fcw=fcw,
                                  i_io_lutEnable=1,
                                  i_io_lutWrite=lut_write,
                                  i_io_lutAddress=lut_address,
                                  i_io_lutWriteData=lut_write_data,
                                  o_io_lutReadData=lut_read_data,
                                  o_io_sync=sync_pin,
                                  o_io_dina=dina_pin,
                                  o_io_dinb=dinb_pin,
                                  o_io_sclk=sclk_pin,
                                  i_clk=ClockSignal(),
                                  i_reset=ResetSignal())

        self.comb += enable.eq(self._en.storage)
        self.comb += fcw.eq(self._fcw.storage)
        self.comb += prescaler.eq(self._prescaler.storage)

    def get_memories(self):
        return [(False, self._lut)]


class LutMemory(Memory):
    def __init__(self, port):
        Memory.__init__(self, 32, 2048, init=None, name=None)
        self._port = port
        self.ports.append(port)

    def get_port(self, write_capable=False, async_read=False,
                 has_re=False, we_granularity=0, mode=WRITE_FIRST,
                 clock_domain="sys"):
        return self._port

    @staticmethod
    def emit_verilog(memory, ns, add_data_file):
        return ""
