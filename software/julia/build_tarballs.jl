# Note that this script can accept some limited command-line arguments, run
# `julia build_tarballs.jl --help` to see a usage message.
using BinaryBuilder, Pkg

name = "SoapyXTRXLiteXSDR"
version = v"0.0.1"

# Collection of sources required to complete build
sources = [
    DirectorySource("../soapysdr/", "software/soapysdr", false),
    DirectorySource("../user/", "software/user", false),
    DirectorySource("../kernel/", "software/kernel", false)

]

dependencies = [
    Dependency("soapysdr_jll"; compat="~0.8.0"),
    Dependency("CUDA_jll")
]

# Bash recipe for building across all platforms
script = raw"""
ls
cd software/soapysdr
ls
mkdir build && cd build
cmake -DCMAKE_INSTALL_PREFIX=${prefix} \
    -DCMAKE_TOOLCHAIN_FILE=${CMAKE_TARGET_TOOLCHAIN} \
    -DCMAKE_BUILD_TYPE=Release \
    ..
make -j${nproc}
make install
"""

# These are the platforms we will build for by default, unless further
# platforms are passed in on the command line
platforms = supported_platforms(;experimental=true)
platforms = expand_cxxstring_abis(platforms) # requested by auditor

# The products that we will ensure are always built
products = Product[
    #LibraryProduct("libPlutoSDRSupport", :libPlutoSDRSupport, ["lib/SoapySDR/modules0.8/"])
]

# Build the tarballs, and possibly a `build.jl` as well.
build_tarballs(ARGS, name, version, sources, script, platforms, products, dependencies; julia_compat="1.6")