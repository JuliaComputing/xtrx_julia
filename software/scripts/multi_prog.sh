#! /usr/bin/env bash


#seq 0 8 | parallel ../litepcie-user-library/litepcie_util -c {} flash_write ../../build/fairwaves_xtrx_platform/gateware/fairwaves_xtrx_platform.bin
for i in `seq 0 8`;
    do ../litepcie-user-library/litepcie_util -c $i flash_write ../../build/fairwaves_xtrx_platform/gateware/fairwaves_xtrx_platform.bin;
done