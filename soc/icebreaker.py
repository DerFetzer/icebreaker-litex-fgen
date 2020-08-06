#!/usr/bin/env python3

# This file is Copyright (c) 2019 Sean Cross <sean@xobs.io>
# This file is Copyright (c) 2018 David Shah <dave@ds0.me>
# This file is Copyright (c) 2020 Piotr Esden-Tempski <piotr@esden.net>
# This file is Copyright (c) 2020 DerFetzer <kontakt@der-fetzer.de>
# License: BSD

# This target was originally based on the Fomu target.

# Import lxbuildenv to integrate the deps/ directory

import argparse
import re

from migen import *
from migen.genlib.resetsync import AsyncResetSynchronizer
from migen.genlib.cdc import MultiReg

from litex.soc.cores.up5kspram import Up5kSPRAM
from litex.soc.cores.spi_flash import SpiFlash
from litex.soc.cores.clock import iCE40PLL
from litex.soc.integration.soc_core import SoCCore
from litex.soc.integration.builder import Builder, builder_argdict, builder_args
from litex.build.lattice.programmer import IceStormProgrammer
from litex.soc.integration.soc_core import soc_core_argdict, soc_core_args
from litex.soc.integration.doc import AutoDoc
from litex.soc.interconnect.csr import AutoCSR, CSRStorage, CSRStatus, CSRField

from litex.build.generic_platform import *

from litex_boards.platforms.icebreaker import Platform, break_off_pmod

from litex.soc.cores.uart import UARTWishboneBridge
from rtl.leds import Leds
from rtl.functiongenerator import FunctionGenerator

import litex.soc.doc as lxsocdoc

# TristateWithFields -------------------------------------------------------------------------------

class GPIOTristateWithFields(Module, AutoCSR):
    def __init__(self, pads, gpio_name):
        nbits     = len(pads)
        fields = [CSRField(fld[0], description=fld[1]) for fld in gpio_name]
        self._oe  = CSRStorage(nbits, description="GPIO Tristate(s) Control.", fields=fields)
        self._in  = CSRStatus(nbits,  description="GPIO Input(s) Status.", fields=fields)
        self._out = CSRStorage(nbits, description="GPIO Output(s) Control.", fields=fields)

        # # #

        _pads = Signal(nbits)
        self.comb += _pads.eq(pads)

        for i in range(nbits):
            t = TSTriple()
            self.specials += t.get_tristate(_pads[i])
            self.comb += t.oe.eq(self._oe.storage[i])
            self.comb += t.o.eq(self._out.storage[i])
            self.specials += MultiReg(t.i, self._in.status[i])

# CRG ----------------------------------------------------------------------------------------------


class _CRG(Module, AutoDoc):
    """Icebreaker Clock Resource Generator

    The system is clocked by the external 12MHz clock. But if a sys_clk_freq is set to a value
    that is different from the default 12MHz we will feed it through the PLL block and try to
    generate a clock as close as possible to the selected frequency.
    """
    def __init__(self, platform, sys_clk_freq):
        self.clock_domains.cd_sys = ClockDomain()
        self.clock_domains.cd_por = ClockDomain()

        # # #

        # Clocking
        clk12 = platform.request("clk12")
        rst_n = platform.request("user_btn_n")
        if sys_clk_freq == 12e6:
            self.comb += self.cd_sys.clk.eq(clk12)
        else:
            self.submodules.pll = pll = iCE40PLL(primitive="SB_PLL40_PAD")
            pll.register_clkin(clk12, 12e6)
            pll.create_clkout(self.cd_sys, sys_clk_freq)
        platform.add_period_constraint(self.cd_sys.clk, 1e9/sys_clk_freq)

        # Power On Reset
        por_cycles  = 4096
        por_counter = Signal(log2_int(por_cycles), reset=por_cycles-1)
        self.comb += self.cd_por.clk.eq(self.cd_sys.clk)
        platform.add_period_constraint(self.cd_por.clk, 1e9/sys_clk_freq)
        self.sync.por += If(por_counter != 0, por_counter.eq(por_counter - 1))
        self.specials += AsyncResetSynchronizer(self.cd_por, ~rst_n)
        self.specials += AsyncResetSynchronizer(self.cd_sys, (por_counter != 0))


# BaseSoC ------------------------------------------------------------------------------------------

class BaseSoC(SoCCore):
    """A SoC on iCEBreaker, optionally with a softcore CPU"""

    # Statically-define the memory map, to prevent it from shifting across various litex versions.
    SoCCore.mem_map = {
        "sram":             0x10000000,  # (default shadow @0xa0000000)
        "spiflash":         0x20000000,  # (default shadow @0xa0000000)
        "csr":              0xe0000000,  # (default shadow @0x60000000)
        "vexriscv_debug":   0xf00f0000,
    }

    def __init__(self, debug, flash_offset, sys_clk_freq, **kwargs):
        """Create a basic SoC for iCEBreaker.

        Create a basic SoC for iCEBreaker.  The `sys` frequency will run at 12 MHz.

        Returns:
            Newly-constructed SoC
        """
        platform = Platform()

        # Set cpu name and variant defaults when none are provided
        if "cpu_variant" not in kwargs:
            if debug:
                kwargs["cpu_variant"] = "lite+debug"
            else:
                kwargs["cpu_variant"] = "lite"

        # Force the SRAM size to 0, because we add our own SRAM with SPRAM
        kwargs["integrated_sram_size"] = 0
        kwargs["integrated_rom_size"]  = 0

        kwargs["csr_data_width"] = 32

        # Set CPU reset address
        kwargs["cpu_reset_address"] = self.mem_map["spiflash"] + flash_offset

        # Select "crossover" as soc uart instead of "serial"
        # We have to make that selection before calling the parent initializer
        if debug:
            kwargs["uart_name"]   = "crossover"

        # SoCCore
        SoCCore.__init__(self, platform, sys_clk_freq, **kwargs)

        self.submodules.crg = _CRG(platform, sys_clk_freq)

        # UP5K has single port RAM, which is a dedicated 128 kilobyte block.
        # Use this as CPU RAM.
        spram_size = 128 * 1024
        self.submodules.spram = Up5kSPRAM(size=spram_size)
        self.register_mem("sram", self.mem_map["sram"], self.spram.bus, spram_size)

        # The litex SPI module supports memory-mapped reads, as well as a bit-banged mode
        # for doing writes.
        spiflash_size = 16 * 1024 * 1024
        self.submodules.spiflash = SpiFlash(platform.request("spiflash4x"), dummy=6, endianness="little")
        self.register_mem("spiflash", self.mem_map["spiflash"], self.spiflash.bus, size=spiflash_size)
        self.add_csr("spiflash")

        # Add ROM linker region
        self.add_memory_region("rom", self.mem_map["spiflash"] + flash_offset, spiflash_size - flash_offset, type="cached+linker")

        # In debug mode, add a UART bridge.  This takes over from the normal UART bridge,
        # however you can use the "crossover" UART to communicate with this over the bridge.
        if debug:
            self.submodules.uart_bridge = UARTWishboneBridge(platform.request("serial"), sys_clk_freq, baudrate=115200)
            self.add_wb_master(self.uart_bridge.wishbone)
            if hasattr(self, "cpu") and self.cpu.name == "vexriscv":
                self.register_mem("vexriscv_debug", 0xf00f0000, self.cpu.debug_bus, 0x100)

        platform.add_extension(break_off_pmod)

        self.submodules.leds = Leds(Cat(
            platform.request("user_ledr_n"),
            platform.request("user_ledg_n"),
            platform.request("user_ledr"),
            platform.request("user_ledg", 0),
            platform.request("user_ledg", 1),
            platform.request("user_ledg", 2),
            platform.request("user_ledg", 3)),
            led_polarity=0x03,
            led_name=[
                ["ledr", "The Red LED on the main iCEBreaker board."],
                ["ledg", "The Green LED on the main iCEBreaker board."],
                ["hledr1", "The center Red LED #1 on the iCEBreaker head."],
                ["hledg2", "Green LED #2 on the iCEBreaker head."],
                ["hledg3", "Green LED #3 on the iCEBreaker head."],
                ["hledg4", "Green LED #4 on the iCEBreaker head."],
                ["hledg5", "Green LED #5 on the iCEBreaker head."]])

        self.add_csr("leds")

        self.submodules.gpiob = GPIOTristateWithFields(Cat(
            platform.request("user_btn", 0),
            platform.request("user_btn", 1),
            platform.request("user_btn", 2),
            ),
            gpio_name=[
                ["userbtn1", "The Button 1"],
                ["userbtn2", "The Button 2"],
                ["userbtn3", "The Button 3"]]
            )
        self.add_csr("gpiob")

        platform.add_extension([
            ("fg_sync", 0, Pins("PMOD1A:4"), IOStandard("LVCMOS33")),
            ("fg_dina", 0, Pins("PMOD1A:5"), IOStandard("LVCMOS33")),
            ("fg_dinb", 0, Pins("PMOD1A:6"), IOStandard("LVCMOS33")),
            ("fg_sclk", 0, Pins("PMOD1A:7"), IOStandard("LVCMOS33"))])

        self.submodules.fgen = FunctionGenerator(platform,
                                                 platform.request("fg_sync", 0),
                                                 platform.request("fg_dina", 0),
                                                 platform.request("fg_dinb", 0),
                                                 platform.request("fg_sclk", 0))
        self.add_csr("fgen_lut")
        self.add_csr("fgen")

    def set_yosys_nextpnr_settings(self, nextpnr_seed=0, nextpnr_placer="heap"):
        """Set Yosys/Nextpnr settings by overriding default LiteX's settings.
        Args:
            nextpnr_seed   (int): Seed to use in Nextpnr
            nextpnr_placer (str): Placer to use in Nextpnr
        """
        assert hasattr(self.platform.toolchain, "yosys_template")
        assert hasattr(self.platform.toolchain, "build_template")
        self.platform.toolchain.yosys_template = [
            "{read_files}",
            "attrmap -tocase keep -imap keep=\"true\" keep=1 -imap keep=\"false\" keep=0 -remove keep=0",
            # Use "-relut -dffe_min_ce_use 4" to the synth_ice40 command. The "-reult" adds an additional
            # LUT pass to pack more stuff in, and the "-dffe_min_ce_use 4" flag prevents Yosys from
            # generating a Clock Enable signal for a LUT that has fewer than 4 flip-flops. This increases
            # density, and lets us use the FPGA more efficiently.
            "synth_ice40 -json {build_name}.json -top {build_name} -relut -abc2 -dffe_min_ce_use 4 -relut",
        ]
        self.platform.toolchain.build_template = [
            "yosys -q -l {build_name}.rpt {build_name}.ys",
            "nextpnr-ice40 --json {build_name}.json --pcf {build_name}.pcf --asc {build_name}.txt" +
            " --pre-pack {build_name}_pre_pack.py --{architecture} --package {package}" +
            " --seed {}".format(nextpnr_seed) +
            " --placer {}".format(nextpnr_placer),
            # Disable final deep-sleep power down so firmware words are loaded onto softcore's address bus.
            "icepack -s {build_name}.txt {build_name}.bin"
        ]


# Build --------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="LiteX SoC on iCEBreaker")
    parser.add_argument("--flash-offset", default=0x40000, help="Boot offset in SPI Flash")
    parser.add_argument("--sys-clk-freq", type=float, default=21e6, help="Select system clock frequency")
    parser.add_argument("--nextpnr-seed", default=0, help="Select nextpnr pseudo random seed")
    parser.add_argument("--nextpnr-placer", default="heap", choices=["sa", "heap"], help="Select nextpnr placer algorithm")
    parser.add_argument("--debug", action="store_true", help="Enable debug features. (UART has to be used with the wishbone-tool.)")
    parser.add_argument("--document-only", action="store_true", help="Do not build a soc. Only generate documentation.")
    parser.add_argument("--flash", action="store_true", help="Load bitstream")
    builder_args(parser)
    soc_core_args(parser)
    args = parser.parse_args()

    # Create the SOC
    soc = BaseSoC(debug=args.debug, flash_offset=args.flash_offset, sys_clk_freq=int(args.sys_clk_freq), **soc_core_argdict(args))
    soc.set_yosys_nextpnr_settings(nextpnr_seed=args.nextpnr_seed, nextpnr_placer=args.nextpnr_placer)

    # Configure command line parameter defaults
    # Don't build software -- we don't include it since we just jump to SPI flash.
    builder_kwargs = builder_argdict(args)
    builder_kwargs["compile_software"] = False

    if args.document_only:
        builder_kwargs["compile_gateware"] = False
    if builder_kwargs["csr_svd"] is None:
        builder_kwargs["csr_svd"] = "../rust/litex-pac/iCEBESOC.svd"
    if builder_kwargs["memory_x"] is None:
        builder_kwargs["memory_x"] = "../rust/litex-pac/memory.x"

    # Create and run the builder
    builder = Builder(soc, **builder_kwargs)
    builder.build()
    lxsocdoc.generate_docs(soc, "build/documentation/", project_name="iCEBreaker LiteX Riscv Function Generator SOC", author="DerFetzer")

    # Modify SVD to handle LUT memory as array
    repl = "                    <name>FGEN_LUT[%s]</name>\n" \
        + "                    <dim>512</dim>\n" \
        + "                    <dimIncrement>4</dimIncrement>\n"

    with open("../rust/litex-pac/iCEBESOC.svd", "r") as f:
        svd = f.read()
        svd = re.sub("[ ]{20}<name>FGEN_LUT</name>\n", repl, svd)

    with open("../rust/litex-pac/iCEBESOC.svd", "w") as f:
        f.write(svd)

    # If requested load the resulting bitstream onto the iCEBreaker
    if args.flash:
        IceStormProgrammer().flash(0x00000000, "soc_basesoc_icebreaker/gateware/top.bin")


if __name__ == "__main__":
    main()
