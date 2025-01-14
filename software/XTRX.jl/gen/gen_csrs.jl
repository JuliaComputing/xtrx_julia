#! /usr/bin/env julia

## This script generates the CSRs.jl file, which contains mappings for all of
## our control status registers defined in the gateware, automatically mapped
## over from the `csr.h` file generated by litex.
using Clang.Generators

options = load_options(joinpath(@__DIR__, "gen_csrs.toml"))

# add compiler flags, e.g. "-DXXXXXXXXX"
args = get_default_args()

# Point it to the headers in this repository
include_dir = joinpath(@__DIR__, "../../litepcie-kernel-module/") |> normpath
headers = joinpath.(include_dir, ["csr.h", "soc.h", "mem.h", "config.h"])
ctx = create_context(headers, args, options)
build!(ctx)
