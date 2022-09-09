#
# This file is part of XTRX-Julia.
#
# Copyright (c) 2021 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

from migen import *

from litex.soc.interconnect.csr import *

# PMIC --------------------------------------------------------------------------------------------

class PMIC(Module, AutoCSR):
    def __init__(self):
        self.control = CSRStorage(fields=[
            CSRField("sel", size=1, offset=0, values=[
                ("``0b0``", "Use VCTCXO Clk."),
                ("``0b1``", "Use External Clk.")
            ], reset=0),
            CSRField("en", size=1, offset=1, values=[
                ("``0b0``", "Disable VCTCXO"),
                ("``0b1``", "Enable VCTCXO")
            ], reset=1),
        ])