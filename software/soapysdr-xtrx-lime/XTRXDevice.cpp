//
// SoapySDR driver for the LMS7002M-based Fairwaves XTRX.
//
// Copyright (c) 2021 Julia Computing.
// Copyright (c) 2015-2015 Fairwaves, Inc.
// Copyright (c) 2015-2015 Rice University
// SPDX-License-Identifier: Apache-2.0
// http://www.apache.org/licenses/LICENSE-2.0
//

// TODO
//
// - much here is copied from the EVB7 driver, which is also LMS7002M-based,
//   but quite some functionality still needs to be adapted for the XTRX.
//   preferably by somebody who actually knows about SDRs.
//
// - sometimes (after a reboot?) the SoapySDR driver fails to initialize the
//   XTRX, esp. when using the loopback or pattern generator. executing
//   `litex_test record /dev/null 1024`, even when that hangs, fixes that.
//   what are we not properly initializing?

#include "XTRXDevice.hpp"
#include "csr.h"
#include "litepcie_interface.h"
#include <SoapySDR/Registry.hpp>
#include <SoapySDR/Logger.hpp>
#include <LMS7002M/LMS7002M_logger.h>
#include <chrono>
#include <fstream>
#include <string>
#include <sys/mman.h>
#include <lime/LMS7002M.h>
#include <lime/ConnectionHandle.h>
#include <lime/lms7_device.h>
#include <lime/LMS7002M_parameters.h>
#include <lime/Logger.h>
#include <sstream>

using namespace lime;

#define dirName ((direction == SOAPY_SDR_RX)?"Rx":"Tx")

void customLogHandler(const LMS7_log_level_t level, const char *message) {
    switch (level) {
    case LMS7_FATAL:    SoapySDR::log(SOAPY_SDR_FATAL, message);    break;
    case LMS7_CRITICAL: SoapySDR::log(SOAPY_SDR_CRITICAL, message); break;
    case LMS7_ERROR:    SoapySDR::log(SOAPY_SDR_ERROR, message);    break;
    case LMS7_WARNING:  SoapySDR::log(SOAPY_SDR_WARNING, message);  break;
    case LMS7_NOTICE:   SoapySDR::log(SOAPY_SDR_NOTICE, message);   break;
    case LMS7_INFO:     SoapySDR::log(SOAPY_SDR_INFO, message);     break;
    case LMS7_DEBUG:    SoapySDR::log(SOAPY_SDR_DEBUG, message);    break;
    case LMS7_TRACE:    SoapySDR::log(SOAPY_SDR_TRACE, message);    break;
    }
}

static ConnectionHandle argsToHandle(const SoapySDR::Kwargs &args)
{
    ConnectionHandle handle;

    //load handle with key/value provided options
    if (args.count("module") != 0) handle.module = args.at("module");
    if (args.count("media") != 0) handle.media = args.at("media");
    if (args.count("name") != 0) handle.name = args.at("name");
    if (args.count("addr") != 0) handle.addr = args.at("addr");
    if (args.count("serial") != 0) handle.serial = args.at("serial");
    if (args.count("index") != 0) handle.index = std::stoi(args.at("index"));

    return handle;
}

// Forward declaration for usage in constructor.
std::string getXTRXSerial(int fd);
std::string getXTRXIdentification(int fd);

/***********************************************************************
 * Constructor
 **********************************************************************/

void dma_init_cpu(int fd) {
    struct litepcie_ioctl_dma_init m;
    m.use_gpu = 0;
    checked_ioctl(fd, LITEPCIE_IOCTL_DMA_INIT, &m);
}

void dma_init_gpu(int fd, void *addr, size_t size) {
    struct litepcie_ioctl_dma_init m;
    m.use_gpu = 1;
    m.gpu_addr = (uint64_t)addr;
    m.gpu_size = size;
    checked_ioctl(fd, LITEPCIE_IOCTL_DMA_INIT, &m);
}

void dma_set_loopback(int fd, bool loopback_enable) {
    struct litepcie_ioctl_dma m;
    m.loopback_enable = loopback_enable ? 1 : 0;
    checked_ioctl(fd, LITEPCIE_IOCTL_DMA, &m);
}

SoapyXTRX::SoapyXTRX(const ConnectionHandle &handle, const SoapySDR::Kwargs &args)
    : _fd(-1),
      _lms(NULL), _masterClockRate(80.0e6), oversampling(0) {
    LMS7_set_log_handler(&customLogHandler);
    LMS7_set_log_level(LMS7_TRACE);
    SoapySDR::logf(SOAPY_SDR_INFO, "SoapyXTRX initializing...");
    setvbuf(stdout, NULL, _IOLBF, 0);

    // open LitePCIe descriptor
    if (args.count("path") == 0) {
        // if path is not present, then findXTRX had zero devices enumerated
        throw std::runtime_error("No LitePCIe devices found!");
    }

    std::string path = args.at("path");
    ConnectionHandle xtrx_handle;
    xtrx_handle.index = stoi(path.substr(13));

    lms7Device = LMS7_Device::CreateDevice(xtrx_handle);
    if (lms7Device == nullptr) throw std::runtime_error(
        "Failed to make connection with '" + handle.serialize() + "'");

    _fd = lms7Device->GetConnection()->getFileDescriptor();

    SoapySDR::logf(SOAPY_SDR_INFO, "Opened devnode %s, serial %s", path.c_str(), getXTRXSerial(_fd).c_str());

    // Get board revision
    this->board_revision = board_get_revision();
    SoapySDR::logf(SOAPY_SDR_INFO, "Board revision: %d", this->board_revision);
    SoapySDR::logf(SOAPY_SDR_INFO, getXTRXIdentification(_fd).c_str());

    // reset the LMS7002M
    litepcie_writel(_fd, CSR_LMS7002M_CONTROL_ADDR,
        1 * (1 << CSR_LMS7002M_CONTROL_RESET_OFFSET)
    );
    litepcie_writel(_fd, CSR_LMS7002M_CONTROL_ADDR,
        0 * (1 << CSR_LMS7002M_CONTROL_RESET_OFFSET)
    );

    // reset XTRX-specific LMS7002M controls
    litepcie_writel(_fd, CSR_LMS7002M_CONTROL_ADDR,
        0 * (1 << CSR_LMS7002M_CONTROL_POWER_DOWN_OFFSET) |
        1 * (1 << CSR_LMS7002M_CONTROL_TX_ENABLE_OFFSET)  |
        1 * (1 << CSR_LMS7002M_CONTROL_RX_ENABLE_OFFSET)  |
        0 * (1 << CSR_LMS7002M_CONTROL_TX_RX_LOOPBACK_ENABLE_OFFSET)
    );

    //Enable DMA Synchronizer
    #ifdef CSR_PCIE_DMA0_SYNCHRONIZER_ENABLE_ADDR
    litepcie_writel(_fd, CSR_PCIE_DMA0_SYNCHRONIZER_ENABLE_ADDR, 0b10);
    #endif

    // reset other FPGA peripherals
    writeSetting("FPGA_DMA_LOOPBACK_ENABLE", "FALSE");
    writeSetting("FPGA_TX_PATTERN", "0");
    writeSetting("FPGA_RX_PATTERN", "0");
    writeSetting("FPGA_RX_DELAY", "16");
    writeSetting("FPGA_TX_DELAY", "16");

    // set clock to Reference Clock Source
    if (args.count("clock") == 0) {
        this->setClockSource("internal");
    } else {
        std::string clock = args.at("clock");
        this->setClockSource(clock);
    }

    SoapySDR::logf(SOAPY_SDR_INFO, "LMS7002M info: revision %d, version %d",
        lms7Device->ReadParam(LMS7_REV),
        lms7Device->ReadParam(LMS7_VER)
    );

    const auto devInfo = lms7Device->GetInfo();
    //quick summary
    SoapySDR::logf(SOAPY_SDR_INFO, "Device name: %s", devInfo->deviceName);
    SoapySDR::logf(SOAPY_SDR_INFO, "Reference: %g MHz", lms7Device->GetClockFreq(LMS_CLOCK_REF)/1e6);


    lms7Device->Init();


    //enable all channels
    for (size_t channel = 0; channel < lms7Device->GetNumChannels(); channel++)
    {
        lms7Device->EnableChannel(true, channel, true);
        lms7Device->EnableChannel(false, channel, true);
    }

    //disable use of value cache automatically
    //or specify args[enableCache] == 1 to enable
    bool cacheEnable = false;
    if (args.count("cacheCalibrations"))
    {
        SoapySDR::logf(SOAPY_SDR_INFO, "'cacheCalibrations' setting is deprecated use 'enableCache' instead", devInfo->deviceName);
        if (std::stoi(args.at("cacheCalibrations")))
            cacheEnable = true;
    }
    else
        cacheEnable = args.count("enableCache") && std::stoi(args.at("enableCache"))!= 0;

    SoapySDR::logf(SOAPY_SDR_INFO, "LMS7002M register cache: %s", cacheEnable?"Enabled":"Disabled");
    lms7Device->EnableCache(cacheEnable);

    //give all RFICs a default state
    for (size_t channel = 0; channel < lms7Device->GetNumChannels(); channel++)
    {
        this->setGain(SOAPY_SDR_RX, channel, 32);
        this->setGain(SOAPY_SDR_TX, channel, 0);
    }
    mChannels[SOAPY_SDR_RX].resize(lms7Device->GetNumChannels());
    mChannels[SOAPY_SDR_TX].resize(lms7Device->GetNumChannels());
    _channelsToCal.clear();
    activeStreams.clear();

    // setup LMS7002M
    _lms = LMS7002M_create(litepcie_interface_transact, &_fd);
/*
    LMS7002M_reset(_lms);
    LMS7002M_set_spi_mode(_lms, 4);

    // read info register
    LMS7002M_regs_spi_read(_lms, 0x002f);
    SoapySDR::logf(SOAPY_SDR_INFO, "LMS7002M info: revision %d, version %d",
                   LMS7002M_regs(_lms)->reg_0x002f_rev,
                   LMS7002M_regs(_lms)->reg_0x002f_ver);

    // configure data port directions and data clock rates
    LMS7002M_configure_lml_port(_lms, LMS_PORT2, LMS_TX, 1);
    LMS7002M_configure_lml_port(_lms, LMS_PORT1, LMS_RX, 1);

    // enable components
    LMS7002M_afe_enable(_lms, LMS_TX, LMS_CHA, true);
    LMS7002M_afe_enable(_lms, LMS_TX, LMS_CHB, true);
    LMS7002M_afe_enable(_lms, LMS_RX, LMS_CHA, true);
    LMS7002M_afe_enable(_lms, LMS_RX, LMS_CHB, true);
    LMS7002M_rxtsp_enable(_lms, LMS_CHA, true);
    LMS7002M_rxtsp_enable(_lms, LMS_CHB, true);
    LMS7002M_txtsp_enable(_lms, LMS_CHA, true);
    LMS7002M_txtsp_enable(_lms, LMS_CHB, true);
    LMS7002M_rbb_enable(_lms, LMS_CHA, true);
    LMS7002M_rbb_enable(_lms, LMS_CHB, true);
    LMS7002M_tbb_enable(_lms, LMS_CHA, true);
    LMS7002M_tbb_enable(_lms, LMS_CHB, true);
    LMS7002M_rfe_enable(_lms, LMS_CHA, true);
    LMS7002M_rfe_enable(_lms, LMS_CHB, true);
    LMS7002M_trf_enable(_lms, LMS_CHA, true);
    LMS7002M_trf_enable(_lms, LMS_CHB, true);
    LMS7002M_sxx_enable(_lms, LMS_RX, true);
    LMS7002M_sxx_enable(_lms, LMS_TX, true);

    // XTRX-specific configuration
    LMS7002M_ldo_enable(_lms, true, LMS7002M_LDO_ALL);
    LMS7002M_xbuf_share_tx(_lms, true);

    // some defaults to avoid throwing
    _cachedSampleRates[SOAPY_SDR_RX] = 1e6;
    _cachedSampleRates[SOAPY_SDR_TX] = 1e6;
    for (size_t i = 0; i < 2; i++) {
        _cachedFreqValues[SOAPY_SDR_RX][i]["RF"] = 1e9;
        _cachedFreqValues[SOAPY_SDR_TX][i]["RF"] = 1e9;
        _cachedFreqValues[SOAPY_SDR_RX][i]["BB"] = 0;
        _cachedFreqValues[SOAPY_SDR_TX][i]["BB"] = 0;
        this->setAntenna(SOAPY_SDR_RX, i, "LNAW");
        this->setAntenna(SOAPY_SDR_TX, i, "BAND1");

        // Use the same default gains as LimeSDR
        // LimeSuiteGUI lists these as:
        //   RFE page:
        //     LNA: Gmax (maps to 30dB)
        //     Loopback: Gmax-40 (not listed here)
        //     TIA: Gmax-3 (maps to 9dB)
        //   RBB page:
        //     PGA Gain: 6dB
        //   TRF page:
        //     TXPAD gain control: 0
        this->setGain(SOAPY_SDR_RX, i, "LNA", 30.0);
        this->setGain(SOAPY_SDR_RX, i, "TIA", 9.0);
        this->setGain(SOAPY_SDR_RX, i, "PGA", 6.0);
        this->setGain(SOAPY_SDR_TX, i, "PAD", 0.0);

        _cachedFilterBws[SOAPY_SDR_RX][i] = 10e6;
        _cachedFilterBws[SOAPY_SDR_TX][i] = 10e6;
        this->setIQBalance(SOAPY_SDR_RX, i, std::polar(1.0, 0.0));
        this->setIQBalance(SOAPY_SDR_TX, i, std::polar(1.0, 0.0));
    }
*/

//    if (_lms == NULL)
//        throw std::runtime_error(
//            "SoapyXTRX(): failed to LMS7002M_create()");

    // turn the clocks on (tested frequencies: 61.44MHz, 80MHz, 122.88MHz)
//    this->setMasterClockRate(80.0e6);

    // set-up the DMA
    checked_ioctl(_fd, LITEPCIE_IOCTL_MMAP_DMA_INFO, &_dma_mmap_info);
    _dma_target = TargetDevice::CPU;
    if (args.count("device") != 0) {
        if (args.at("device") == "CPU")
            _dma_target = TargetDevice::CPU;
        else if (args.at("device") == "GPU")
            _dma_target = TargetDevice::GPU;
        else
            throw std::runtime_error("invalid device");
    }
    switch (_dma_target) {
    case TargetDevice::CPU:
        dma_init_cpu(_fd);
        _dma_buf = NULL;
        break;
    case TargetDevice::GPU:
#ifdef CUDA
        size_t dma_buffer_total_size =
            _dma_mmap_info.dma_tx_buf_count * _dma_mmap_info.dma_tx_buf_size +
            _dma_mmap_info.dma_rx_buf_count * _dma_mmap_info.dma_rx_buf_size;
        checked_cuda_call(
            cuMemAlloc((CUdeviceptr *)&_dma_buf, dma_buffer_total_size));

        unsigned int flag = 1;
        checked_cuda_call(cuPointerSetAttribute(
            &flag, CU_POINTER_ATTRIBUTE_SYNC_MEMOPS, (CUdeviceptr)_dma_buf));

        dma_init_gpu(_fd, _dma_buf, dma_buffer_total_size);
#else
        throw std::runtime_error("GPU DMA not supported");
#endif
    }

    // NOTE: if initialization misses a setting/register, try experimenting in
    //       LimeGUI and loading that register dump here
    if (args.count("ini") != 0) {
        if (LMS7002M_load_ini(_lms, args.at("ini").c_str())){
            SoapySDR::log(SOAPY_SDR_ERROR, "SoapyXTRX configuration load failed");
            throw std::runtime_error("failed to load XTRX configuration");
        } else {
            SoapySDR::logf(SOAPY_SDR_INFO, "SoapyXTRX configuration loaded from: %s", args.at("ini").c_str());

        }
    }

    SoapySDR::log(SOAPY_SDR_INFO, "SoapyXTRX initialization complete");
}

SoapyXTRX::~SoapyXTRX(void) {
    SoapySDR::log(SOAPY_SDR_INFO, "Power down and cleanup");
    if (_rx_stream.opened) {
        litepcie_release_dma(_fd, 0, 1);

            munmap(_rx_stream.buf, _dma_mmap_info.dma_rx_buf_size *
                                    _dma_mmap_info.dma_rx_buf_count);
        _rx_stream.opened = false;
    }
    if (_tx_stream.opened) {
        // release the DMA engine
        litepcie_release_dma(_fd, 1, 0);

        munmap(_tx_stream.buf, _dma_mmap_info.dma_tx_buf_size *
                                   _dma_mmap_info.dma_tx_buf_count);
        _tx_stream.opened = false;
    }
    for (size_t channel = 0; channel < lms7Device->GetNumChannels(); channel++)
    {
        lms7Device->EnableChannel(true, channel, false);
        lms7Device->EnableChannel(false, channel, false);
    }
    delete lms7Device;

//    LMS7002M_destroy(_lms);
    close(_fd);
}


/***********************************************************************
 * Identification API
 **********************************************************************/

SoapySDR::Kwargs SoapyXTRX::getHardwareInfo(void) const {
    SoapySDR::Kwargs args;

    char fpga_identification[256];
    for (int i = 0; i < 256; i++)
        fpga_identification[i] =
            litepcie_readl(_fd, CSR_IDENTIFIER_MEM_BASE + 4 * i);
    args["identification"] = std::string(fpga_identification);

    return args;
}


/*******************************************************************
 * Antenna API
 ******************************************************************/

std::vector<std::string> SoapyXTRX::listAntennas(const int direction, const size_t /*channel*/) const
{
    return lms7Device->GetPathNames(direction == SOAPY_SDR_TX);
}

void SoapyXTRX::setAntenna(const int direction, const size_t channel,
                           const std::string &name) {
    std::unique_lock<std::recursive_mutex> lock(_accessMutex);

    SoapySDR::logf(SOAPY_SDR_DEBUG, "SoapyLMS7::setAntenna(%s, %d, %s)", dirName, int(channel), name.c_str());

    bool tx = direction == SOAPY_SDR_TX;
    std::vector<std::string> nameList = lms7Device->GetPathNames(tx);
    if (tx) {
        int tx_rf_switch = 0;
        if (name == "BAND1") {
            tx_rf_switch = 1;
        }
        else if (name == "BAND2") {
            tx_rf_switch = 0;
        }
        litepcie_writel(_fd, CSR_RF_SWITCHES_TX_ADDR, tx_rf_switch);
    }
    else {
        int rx_rf_switch = 0;
        if (name == "LNAH") {
            rx_rf_switch = 2;
        }
        else if (name == "LNAL") {
            rx_rf_switch = 1;
        }
        else if (name == "LNAW") {
            rx_rf_switch = 0;
        }
        litepcie_writel(_fd, CSR_RF_SWITCHES_RX_ADDR, rx_rf_switch);
    }
    for (unsigned path = 0; path < nameList.size(); path++)
        if (nameList[path] == name)
        {
            lms7Device->SetPath(tx, channel, path);
            _channelsToCal.emplace(direction, channel);
            return;
        }

    throw std::runtime_error("SoapyLMS7::setAntenna(TX, "+name+") - unknown antenna name");
}

std::string SoapyXTRX::getAntenna(const int direction, const size_t channel) const
{
    std::unique_lock<std::recursive_mutex> lock(_accessMutex);
    bool tx = direction == SOAPY_SDR_TX;
    int path = lms7Device->GetPath(tx,channel);
    if (path < 0)
        return "";

    std::vector<std::string> nameList = lms7Device->GetPathNames(tx);
    return (unsigned)path < nameList.size() ? nameList[path] : "";
}


/*******************************************************************
 * Frontend corrections API
 ******************************************************************/

bool SoapyXTRX::hasDCOffsetMode(const int direction, const size_t /*channel*/) const
{
    return (direction == SOAPY_SDR_RX);
}

void SoapyXTRX::setDCOffsetMode(const int direction, const size_t channel, const bool automatic)
{
    std::unique_lock<std::recursive_mutex> lock(_accessMutex);
    if (direction == SOAPY_SDR_RX)
        lms7Device->WriteParam(LMS7param(DC_BYP_RXTSP),automatic == 0, channel);
}

bool SoapyXTRX::getDCOffsetMode(const int direction, const size_t channel) const
{
    std::unique_lock<std::recursive_mutex> lock(_accessMutex);
    if (direction == SOAPY_SDR_RX)
        return lms7Device->ReadParam(LMS7param(DC_BYP_RXTSP),channel) == 0;
    return false;
}

bool SoapyXTRX::hasDCOffset(const int /*direction*/, const size_t /*channel*/) const
{
    return true;
}

void SoapyXTRX::setDCOffset(const int direction, const size_t channel, const std::complex<double> &offset)
{
    std::unique_lock<std::recursive_mutex> lock(_accessMutex);
    const auto lmsDir = (direction == SOAPY_SDR_TX)?LMS7002M::Tx:LMS7002M::Rx;
    auto rfic = lms7Device->GetLMS(channel/2);
    rfic->Modify_SPI_Reg_bits(LMS7param(MAC),(channel%2)+1);
    rfic->SetDCOffset(lmsDir, offset.real(), offset.imag());
}

std::complex<double> SoapyXTRX::getDCOffset(const int direction, const size_t channel) const
{
    double I = 0.0, Q = 0.0;
    const auto lmsDir = (direction == SOAPY_SDR_TX)?LMS7002M::Tx:LMS7002M::Rx;
    std::unique_lock<std::recursive_mutex> lock(_accessMutex);
    auto rfic = lms7Device->GetLMS(channel/2);
    rfic->Modify_SPI_Reg_bits(LMS7param(MAC),(channel%2)+1);
    rfic->GetDCOffset(lmsDir, I, Q);
    return std::complex<double>(I, Q);
}

bool SoapyXTRX::hasIQBalance(const int /*direction*/, const size_t /*channel*/) const
{
    return true;
}

void SoapyXTRX::setIQBalance(const int direction, const size_t channel, const std::complex<double> &balance)
{
    const auto lmsDir = (direction == SOAPY_SDR_TX)?LMS7002M::Tx:LMS7002M::Rx;
    double gain = std::abs(balance);
    double gainI = 1.0; if (gain < 1.0) gainI = gain/1.0;
    double gainQ = 1.0; if (gain > 1.0) gainQ = 1.0/gain;
    std::unique_lock<std::recursive_mutex> lock(_accessMutex);
    auto rfic = lms7Device->GetLMS(channel/2);
    rfic->Modify_SPI_Reg_bits(LMS7param(MAC),(channel%2)+1);
    rfic->SetIQBalance(lmsDir, std::arg(balance), gainI, gainQ);
}

std::complex<double> SoapyXTRX::getIQBalance(const int direction, const size_t channel) const
{
    const auto lmsDir = (direction == SOAPY_SDR_TX)?LMS7002M::Tx:LMS7002M::Rx;
    double phase, gainI, gainQ;
    std::unique_lock<std::recursive_mutex> lock(_accessMutex);
    auto rfic = lms7Device->GetLMS(channel/2);
    rfic->Modify_SPI_Reg_bits(LMS7param(MAC),(channel%2)+1);
    rfic->GetIQBalance(lmsDir, phase, gainI, gainQ);
    return (gainI/gainQ)*std::polar(1.0, phase);
}


/*******************************************************************
 * Gain API
 ******************************************************************/

std::vector<std::string> SoapyXTRX::listGains(const int direction, const size_t /*channel*/) const
{
    std::vector<std::string> gains;
    if (direction == SOAPY_SDR_RX)
    {
        gains.push_back("TIA");
        gains.push_back("LNA");
        gains.push_back("PGA");
    }
    if (direction == SOAPY_SDR_TX)
    {
        gains.push_back("PAD");
        gains.push_back("IAMP");
    }
    return gains;
}

void SoapyXTRX::setGain(const int direction, const size_t channel, const double value)
{
    std::unique_lock<std::recursive_mutex> lock(_accessMutex);
    SoapySDR::logf(SOAPY_SDR_DEBUG, "SoapyXTRX::setGain(%s, %d, %g dB)", dirName, int(channel), value);
    lms7Device->SetGain(direction==SOAPY_SDR_TX,channel,value);
    SoapySDR::logf(SOAPY_SDR_DEBUG, "Actual %s[%d] gain %g dB", dirName, int(channel), this->getGain(direction, channel));
}

double SoapyXTRX::getGain(const int direction, const size_t channel) const
{
    std::unique_lock<std::recursive_mutex> lock(_accessMutex);
    return lms7Device->GetGain(direction==SOAPY_SDR_TX, channel);
}

void SoapyXTRX::setGain(const int direction, const size_t channel, const std::string &name, const double value)
{
    std::unique_lock<std::recursive_mutex> lock(_accessMutex);
    SoapySDR::logf(SOAPY_SDR_DEBUG, "SoapyXTRX::setGain(%s, %d, %s, %g dB)", dirName, int(channel), name.c_str(), value);

    lms7Device->SetGain(direction==SOAPY_SDR_TX, channel, value, name);

    SoapySDR::logf(SOAPY_SDR_DEBUG, "Actual %s%s[%d] gain %g dB", dirName, name.c_str(), int(channel), this->getGain(direction, channel, name));
}

double SoapyXTRX::getGain(const int direction, const size_t channel, const std::string &name) const
{
    std::unique_lock<std::recursive_mutex> lock(_accessMutex);
    return lms7Device->GetGain(direction==SOAPY_SDR_TX, channel, name);
}

SoapySDR::Range SoapyXTRX::getGainRange(const int direction, const size_t channel) const
{
    auto range = lms7Device->GetGainRange(direction==SOAPY_SDR_TX, channel);
    return SoapySDR::Range(range.min, range.max);
}

SoapySDR::Range SoapyXTRX::getGainRange(const int direction, const size_t channel, const std::string &name) const
{
    auto range = lms7Device->GetGainRange(direction==SOAPY_SDR_TX, channel, name);
    return SoapySDR::Range(range.min, range.max);
}


/*******************************************************************
 * Frequency API
 ******************************************************************/


SoapySDR::ArgInfoList SoapyXTRX::getFrequencyArgsInfo(const int direction, const size_t channel) const
{
    return SoapySDR::Device::getFrequencyArgsInfo(direction, channel);
}

void SoapyXTRX::setFrequency(int direction, size_t channel, double frequency, const SoapySDR::Kwargs &args)
{
    std::unique_lock<std::recursive_mutex> lock(_accessMutex);
    if (lms7Device->SetFrequency(direction == SOAPY_SDR_TX, channel, frequency)!=0)
    {
        SoapySDR::logf(SOAPY_SDR_ERROR, "setFrequency(%s, %d, %g MHz) Failed", dirName, int(channel), frequency/1e6);
        throw std::runtime_error("SoapyXTRX::setFrequency() failed");
    }
    mChannels[bool(direction)].at(channel).freq = frequency;
    if (setBBLPF(direction, channel, mChannels[direction].at(channel).bw)!= 0)
        SoapySDR::logf(SOAPY_SDR_ERROR, "setBBLPF(%s, %d, RF, %g MHz) Failed", dirName, int(channel), mChannels[direction].at(channel).bw/1e6);
}

void SoapyXTRX::setFrequency(const int direction, const size_t channel, const std::string &name, const double frequency, const SoapySDR::Kwargs &args)
{
    std::unique_lock<std::recursive_mutex> lock(_accessMutex);
    SoapySDR::logf(SOAPY_SDR_DEBUG, "SoapyXTRX::setFrequency(%s, %d, %s, %g MHz)", dirName, int(channel), name.c_str(), frequency/1e6);
    bool isTx = direction == SOAPY_SDR_TX;
    if (name == "RF")
    {
        const auto clkId = (direction == SOAPY_SDR_TX)? LMS_CLOCK_SXT : LMS_CLOCK_SXR;
        if (lms7Device->SetClockFreq(clkId, frequency, channel)!=0)
        {
            SoapySDR::logf(SOAPY_SDR_ERROR, "setFrequency(%s, %d, RF, %g MHz) Failed", dirName, int(channel), frequency/1e6);
            throw std::runtime_error("SoapyXTRX::setFrequency(RF) failed");
        }
        mChannels[bool(direction)].at(channel).freq = frequency;

        if (setBBLPF(direction, channel, mChannels[direction].at(channel).bw)!= 0)
            SoapySDR::logf(SOAPY_SDR_ERROR, "setBBLPF(%s, %d, RF, %g MHz) Failed", dirName, int(channel), mChannels[direction].at(channel).bw/1e6);
        _channelsToCal.emplace(direction, channel);
        return;
    }

    if (name == "BB")
    {
        lms7Device->SetNCOFreq(isTx, channel, 0, direction == SOAPY_SDR_TX ? frequency : -frequency);
        return;
    }

    throw std::runtime_error("SoapyLMS7::setFrequency("+name+") unknown name");
}

double SoapyXTRX::getFrequency(const int direction, const size_t channel, const std::string &name) const
{
    std::unique_lock<std::recursive_mutex> lock(_accessMutex);

    if (name == "RF")
    {
        const auto clkId = (direction == SOAPY_SDR_TX)? LMS_CLOCK_SXT : LMS_CLOCK_SXR;
        return lms7Device->GetClockFreq(clkId,channel);
    }

    if (name == "BB")
    {
        const bool isTx = (direction == SOAPY_SDR_TX);
        double freq = lms7Device->GetNCOFreq(isTx, channel, 0);
        return isTx ? freq : -freq;
    }

    throw std::runtime_error("SoapyXTRX::getFrequency("+name+") unknown name");
}

double SoapyXTRX::getFrequency(const int direction, const size_t channel) const
{
    std::unique_lock<std::recursive_mutex> lock(_accessMutex);
    return lms7Device->GetFrequency(direction == SOAPY_SDR_TX, channel);
}

std::vector<std::string> SoapyXTRX::listFrequencies(const int /*direction*/, const size_t /*channel*/) const
{
    std::vector<std::string> opts;
    opts.push_back("RF");
    opts.push_back("BB");
    return opts;
}

SoapySDR::RangeList SoapyXTRX::getFrequencyRange(const int direction, const size_t channel, const std::string &name) const
{
    SoapySDR::RangeList ranges;
    if (name == "RF")
    {
        ranges.push_back(SoapySDR::Range(30e6, 3.8e9));
    }
    if (name == "BB")
    {
        std::unique_lock<std::recursive_mutex> lock(_accessMutex);
        const auto lmsDir = (direction == SOAPY_SDR_TX)?LMS_CLOCK_TXTSP:LMS_CLOCK_RXTSP;
        const double dspRate = lms7Device->GetClockFreq(lmsDir,channel);
        ranges.push_back(SoapySDR::Range(-dspRate/2, dspRate/2));
    }
    return ranges;
}

SoapySDR::RangeList SoapyXTRX::getFrequencyRange(const int direction, const size_t channel) const
{
    SoapySDR::RangeList ranges;
    ranges.push_back(SoapySDR::Range(0.0, 3.8e9));
    return ranges;
}

void SoapyXTRX::setSampleRate(const int direction, const size_t channel, const double rate)
{
    std::unique_lock<std::recursive_mutex> lock(_accessMutex);
    auto streams = activeStreams;
    for (auto s : streams)
        deactivateStream(s);

    SoapySDR::logf(SOAPY_SDR_DEBUG, "setSampleRate(%s, %d, %g MHz)", dirName, int(channel), rate/1e6);
    auto ret = lms7Device->SetRate(direction == SOAPY_SDR_TX, rate, oversampling);

    //if bandwidth is not explicitly selected set it to sample rate
    if (mChannels[bool(direction)].at(channel).bw < 0)
    {
        lms_range_t range;
        LMS_GetLPFBWRange(lms7Device, direction == SOAPY_SDR_TX, &range);
        double bw = rate < range.min ? range.min : rate;
        bw = bw < range.max ? bw : range.max;
        setBBLPF(direction, channel, bw);
    }

    for (auto s : streams)
        activateStream(s);
    if (ret != 0)
    {
        SoapySDR::logf(SOAPY_SDR_ERROR, "setSampleRate(%s, %d, %g MHz) Failed", dirName, int(channel), rate/1e6);
        throw std::runtime_error("SoapyLMS7::setSampleRate() failed");
    }
    sampleRate[bool(direction)] = rate;
    return;
}

double SoapyXTRX::getSampleRate(const int direction, const size_t channel) const
{
    std::unique_lock<std::recursive_mutex> lock(_accessMutex);

    return lms7Device->GetRate((direction == SOAPY_SDR_TX),channel);
}

std::vector<double> SoapyXTRX::listSampleRates(const int /*direction*/,
                                               const size_t) const {
    const double baseRate = this->getTSPRate();
    std::vector<double> rates;
    // from baseRate/32 to baseRate/2
    for (int i = 5; i >= 1; i--) {
        rates.push_back(baseRate / (1 << i));
    }
    return rates;
}

std::vector<std::string> SoapyXTRX::getStreamFormats(const int /*direction*/,
                                                     const size_t /*channel*/) const
{
    std::vector<std::string> formats;
    formats.push_back(SOAPY_SDR_CS16);
    return formats;
}

/*******************************************************************
 * BW filter API
 ******************************************************************/

int SoapyXTRX::setBBLPF(bool direction, size_t channel, double bw)
{
    if (bw < 0)
        return 0;
    double frequency = mChannels[direction].at(channel).freq;
    if (frequency > 0 && frequency < 30e6)
    {
        bw += 2*(30e6 - frequency);
        bw = bw > 60e6 ? 60e6 : bw;
    }
    if (std::abs(bw-mChannels[direction].at(channel).rf_bw)>10e3)
    {
        SoapySDR::logf(SOAPY_SDR_DEBUG, "lms7Device->SetLPF(%s, %d, %g MHz)", dirName, int(channel), bw/1e6);
        if (lms7Device->SetLPF(direction==SOAPY_SDR_TX, channel, true, bw) == 0)
            mChannels[direction].at(channel).rf_bw = bw;
        else
            return -1;
    }
    return 0;
}

void SoapyXTRX::setBandwidth(const int direction, const size_t channel, const double bw)
{
    if (bw == 0.0) return; //special ignore value

    std::unique_lock<std::recursive_mutex> lock(_accessMutex);
    SoapySDR::logf(SOAPY_SDR_DEBUG, "SoapyLMS7::setBandwidth(%s, %d, %g MHz)", dirName, int(channel), bw/1e6);

    if (setBBLPF(direction, channel, bw)!=0)
    {
        SoapySDR::logf(SOAPY_SDR_ERROR, "setBBLPF(%s, %d, %g MHz) Failed", dirName, int(channel), bw/1e6);
        throw std::runtime_error("setBandwidth() failed");
    }

    mChannels[bool(direction)].at(channel).bw = bw;
    _channelsToCal.emplace(direction, channel);
}

double SoapyXTRX::getBandwidth(const int direction, const size_t channel) const
{
    std::unique_lock<std::recursive_mutex> lock(_accessMutex);
    return mChannels[bool(direction)].at(channel).bw;
}

SoapySDR::RangeList SoapyXTRX::getBandwidthRange(const int direction, const size_t channel) const
{
    SoapySDR::RangeList bws;

    if (direction == SOAPY_SDR_RX)
    {
        lms_range_t range;
        LMS_GetLPFBWRange(lms7Device, LMS_CH_RX, &range);
        bws.push_back(SoapySDR::Range(range.min, range.max));
    }
    if (direction == SOAPY_SDR_TX)
    {
        bws.push_back(SoapySDR::Range(5e6, 40e6));
        bws.push_back(SoapySDR::Range(50e6, 130e6));
    }

    return bws;
}

/*
std::vector<double> SoapyXTRX::listBandwidths(const int direction,
                                              const size_t) const {
    std::vector<double> bws;

    if (direction == SOAPY_SDR_RX) {
        bws.push_back(1.4e6);
        bws.push_back(3.0e6);
        bws.push_back(5.0e6);
        bws.push_back(10.0e6);
        bws.push_back(15.0e6);
        bws.push_back(20.0e6);
        bws.push_back(37.0e6);
        bws.push_back(66.0e6);
        bws.push_back(108.0e6);
    }
    if (direction == SOAPY_SDR_TX) {
        bws.push_back(2.4e6);
        bws.push_back(2.74e6);
        bws.push_back(5.5e6);
        bws.push_back(8.2e6);
        bws.push_back(11.0e6);
        bws.push_back(18.5e6);
        bws.push_back(38.0e6);
        bws.push_back(54.0e6);
    }

    return bws;
}
*/


/*******************************************************************
 * Clocking API
 ******************************************************************/

double SoapyXTRX::getTSPRate() const {
    return _masterClockRate / 4;
}

/*
void SoapyXTRX::setMasterClockRate(const double rate) {
    std::lock_guard<std::mutex> lock(_mutex);

    int ret =
        LMS7002M_set_data_clock(_lms, _refClockRate, rate, &_masterClockRate);
    if (ret != 0) {
        SoapySDR::logf(SOAPY_SDR_ERROR, "LMS7002M_set_data_clock(%f MHz) -> %d",
                       rate / 1e6, ret);
        throw std::runtime_error("XTRX fail LMS7002M_set_data_clock()");
    }
    SoapySDR::logf(SOAPY_SDR_TRACE, "LMS7002M_set_data_clock(%f MHz) -> %f MHz",
                   rate / 1e6, _masterClockRate / 1e6);
}
*/

double SoapyXTRX::getMasterClockRate(void) const
{
    std::unique_lock<std::recursive_mutex> lock(_accessMutex);
    return lms7Device->GetClockFreq(LMS_CLOCK_CGEN);
}

/*!
 * Set the reference clock rate of the device.
 * \param rate the clock rate in Hz
 */
void SoapyXTRX::setReferenceClockRate(const double rate) {
    lms7Device->SetClockFreq(LMS_CLOCK_REF, rate);
}

/*!
 * Get the reference clock rate of the device.
 * \return the clock rate in Hz
 */
double SoapyXTRX::getReferenceClockRate(void) const {
    return lms7Device->GetClockFreq(LMS_CLOCK_REF);
}

/*!
 * Get the range of available reference clock rates.
 * \return a list of clock rate ranges in Hz
 */
SoapySDR::RangeList SoapyXTRX::getReferenceClockRates(void) const {
    SoapySDR::RangeList ranges;
    // Really whatever you want to try...
    ranges.push_back(SoapySDR::Range(25e6, 27e6));
    return ranges;
}



/*!
 * Get the list of available clock sources.
 * \return a list of clock source names
 */
std::vector<std::string> SoapyXTRX::listClockSources(void) const {
    std::vector<std::string> sources;
    sources.push_back("internal");
    sources.push_back("external");
    sources.push_back("external+pps");
    return sources;
}

/*!
 * Set the clock source on the device
 * \param source the name of a clock source
 */
void SoapyXTRX::setClockSource(const std::string &source) {
    int control = litepcie_readl(_fd, CSR_VCTCXO_CONTROL_ADDR);
    control &= ~(1 << CSR_VCTCXO_CONTROL_SEL_OFFSET);

    if (source == "external" || source == "external+pps") {
        control |= 1 << CSR_VCTCXO_CONTROL_SEL_OFFSET;
        setReferenceClockRate(19.2e6);
    } else if (source == "internal") {
        setReferenceClockRate(26e6);
    } else {
        throw std::runtime_error("setClockSource(" + source + ") invalid");
    }

    litepcie_writel(_fd, CSR_VCTCXO_CONTROL_ADDR, control);

    #ifdef CSR_PCIE_DMA0_SYNCHRONIZER_BYPASS_ADDR
    if (source == "external+pps") {
        litepcie_writel(_fd, CSR_PCIE_DMA0_SYNCHRONIZER_BYPASS_ADDR, 0);
    } else {
        litepcie_writel(_fd, CSR_PCIE_DMA0_SYNCHRONIZER_BYPASS_ADDR, 1);
    }
    #endif

    _clockSource = source;
}

/*!
 * Get the clock source of the device
 * \return the name of a clock source
 */
std::string SoapyXTRX::getClockSource(void) const {
    return _clockSource;
}

/*******************************************************************
 * Clocking API
 ******************************************************************/

std::vector<std::string> SoapyXTRX::listSensors(void) const {
    std::vector<std::string> sensors;
#ifdef CSR_XADC_BASE
    sensors.push_back("xadc_temp");
    sensors.push_back("xadc_vccint");
    sensors.push_back("xadc_vccaux");
    sensors.push_back("xadc_vccbram");
#endif
    sensors.push_back("tmp108_temp");
    return sensors;
}

SoapySDR::ArgInfo SoapyXTRX::getSensorInfo(const std::string &key) const {
    SoapySDR::ArgInfo info;

    std::size_t dash = key.find("_");
    if (dash < std::string::npos) {
        std::string deviceStr = key.substr(0, dash);
        std::string sensorStr = key.substr(dash + 1);

#ifdef CSR_XADC_BASE
        if (deviceStr == "xadc") {
            if (sensorStr == "temp") {
                info.key = "temp";
                info.value = "0.0";
                info.units = "C";
                info.description = "FPGA temperature";
                info.type = SoapySDR::ArgInfo::FLOAT;
            } else if (sensorStr == "vccint") {
                info.key = "vccint";
                info.value = "0.0";
                info.units = "V";
                info.description = "FPGA internal supply voltage";
                info.type = SoapySDR::ArgInfo::FLOAT;
            } else if (sensorStr == "vccaux") {
                info.key = "vccaux";
                info.value = "0.0";
                info.units = "V";
                info.description = "FPGA auxiliary supply voltage";
                info.type = SoapySDR::ArgInfo::FLOAT;
            } else if (sensorStr == "vccbram") {
                info.key = "vccbram";
                info.value = "0.0";
                info.units = "V";
                info.description = "FPGA supply voltage for block RAM memories";
                info.type = SoapySDR::ArgInfo::FLOAT;
            } else {
                throw std::runtime_error("SoapyXTRX::getSensorInfo(" + key +
                                         ") unknown sensor");
            }
            return info;
        }
#endif
        throw std::runtime_error("SoapyXTRX::getSensorInfo(" + key +
                                 ") unknown device");
    }
    throw std::runtime_error("SoapyXTRX::getSensorInfo(" + key +
                             ") unknown key");
}

std::string SoapyXTRX::readSensor(const std::string &key) const {
    std::string sensorValue;

    std::size_t dash = key.find("_");
    if (dash < std::string::npos) {
        std::string deviceStr = key.substr(0, dash);
        std::string sensorStr = key.substr(dash + 1);

#ifdef CSR_XADC_BASE
        if (deviceStr == "xadc") {
            if (sensorStr == "temp") {
                sensorValue = std::to_string(
                    (double)litepcie_readl(_fd, CSR_XADC_TEMPERATURE_ADDR) *
                        503.975 / 4096 -
                    273.15);
            } else if (sensorStr == "vccint") {
                sensorValue = std::to_string(
                    (double)litepcie_readl(_fd, CSR_XADC_VCCINT_ADDR) / 4096 *
                    3);
            } else if (sensorStr == "vccaux") {
                sensorValue = std::to_string(
                    (double)litepcie_readl(_fd, CSR_XADC_VCCAUX_ADDR) / 4096 *
                    3);
            } else if (sensorStr == "vccbram") {
                sensorValue = std::to_string(
                    (double)litepcie_readl(_fd, CSR_XADC_VCCBRAM_ADDR) / 4096 *
                    3);
            } else {
                throw std::runtime_error("SoapyXTRX::getSensorInfo(" + key +
                                         ") unknown sensor");
            }
            return sensorValue;
        } else if (deviceStr == "tmp108") {
            if (sensorStr == "temp") {
                unsigned int temp;
                unsigned char dat[2];
                i2c1_read(TMP108_I2C_ADDR, 0x00, dat, 2, true);
                temp = (dat[0] << 4) | (dat[1] >> 4);
                temp = (62500*temp)/1000000; /* 0.0625°C/count */
                sensorValue = std::to_string(temp);
            } else {
                throw std::runtime_error("SoapyXTRX::getSensorInfo(" + key +
                                         ") unknown sensor");
            }
            return sensorValue;
        }
#endif
        throw std::runtime_error("SoapyXTRX::getSensorInfo(" + key +
                                 ") unknown device");
    }
    throw std::runtime_error("SoapyXTRX::getSensorInfo(" + key +
                             ") unknown key");
}


/*******************************************************************
 * Register API
 ******************************************************************/

std::vector<std::string> SoapyXTRX::listRegisterInterfaces(void) const {
    std::vector<std::string> interfaces;
    interfaces.push_back("LMS7002M");
    interfaces.push_back("LitePCI");
    return interfaces;
}

void SoapyXTRX::writeMACregister(const unsigned int A, const unsigned int B) {
    // Get current value of 0x0020
    uint16_t val = this->readRegister(0x0020);
    // Insert our A and B enables
    val = (val & 0xfffc) | ((B & 0x1) << 1) | (A & 0x1);
    this->writeRegister(0x0020, val);
}

void SoapyXTRX::writeRegister(const unsigned addr, const unsigned value) {
    LMS7002M_spi_write(_lms, addr, value);
}

unsigned SoapyXTRX::readRegister(const unsigned addr) const {
    return LMS7002M_spi_read(_lms, addr);
}



void SoapyXTRX::writeRegister(const std::string &name, const unsigned addr, const unsigned value) {
    if (name == "LMS7002M") {
        LMS7002M_spi_write(_lms, addr, value);
    } else if (name == "LitePCI") {
        litepcie_writel(_fd, addr, value);
    } else
        throw std::runtime_error("SoapyXTRX::writeRegister(" + name + ") unknown register");
}

unsigned SoapyXTRX::readRegister(const std::string &name, const unsigned addr) const {
    if (name == "LMS7002M") {
        return LMS7002M_spi_read(_lms, addr);
    } else if (name == "LitePCI") {
        return litepcie_readl(_fd, addr);
    } else
        throw std::runtime_error("SoapyXTRX::readRegister(" + name + ") unknown register");
}



/*******************************************************************
 * Settings API
 ******************************************************************/

std::string SoapyXTRX::readSetting(const std::string &key) const
{
    SoapySDR::logf(SOAPY_SDR_DEBUG, "SoapyXTRX::readSetting(%s)", key.c_str());

    if (key == "FPGA_TX_RX_LOOPBACK_ENABLE") {
        uint32_t control = litepcie_readl(_fd, CSR_LMS7002M_CONTROL_ADDR);
        control &= 1 << CSR_LMS7002M_CONTROL_TX_RX_LOOPBACK_ENABLE_OFFSET;
        return control ? "TRUE" : "FALSE";
    } else if (key == "FPGA_TX_PATTERN") {
        uint32_t control = litepcie_readl(_fd, CSR_LMS7002M_TX_PATTERN_CONTROL_ADDR);
        control &= 1 << CSR_LMS7002M_TX_PATTERN_CONTROL_ENABLE_OFFSET;
        return control ? "1" : "0";
    } else if (key == "FPGA_RX_PATTERN") {
        uint32_t control = litepcie_readl(_fd, CSR_LMS7002M_RX_PATTERN_CONTROL_ADDR);
        control &= 1 << CSR_LMS7002M_RX_PATTERN_CONTROL_ENABLE_OFFSET;
        return control ? "1" : "0";
    } else if (key == "FPGA_RX_PATTERN_ERRORS") {
        uint32_t errors = litepcie_readl(_fd, CSR_LMS7002M_RX_PATTERN_ERRORS_ADDR);
        return std::to_string(errors);
    } else if (key == "FPGA_TX_DELAY") {
        uint32_t reg = litepcie_readl(_fd, CSR_LMS7002M_DELAY_ADDR);
        uint32_t mask = ((uint32_t)(1 << CSR_LMS7002M_DELAY_TX_DELAY_SIZE)-1);
        uint32_t delay = (reg >> CSR_LMS7002M_DELAY_TX_DELAY_OFFSET) & mask;
        return std::to_string(delay);
    } else if (key == "FPGA_RX_DELAY") {
        uint32_t reg = litepcie_readl(_fd, CSR_LMS7002M_DELAY_ADDR);
        uint32_t mask = ((uint32_t)(1 << CSR_LMS7002M_DELAY_RX_DELAY_SIZE)-1);
        uint32_t delay = (reg >> CSR_LMS7002M_DELAY_RX_DELAY_OFFSET) & mask;
        return std::to_string(delay);
    } else if (key == "DMA_BUFFERS") {
        return "RX hw count: " + std::to_string(_rx_stream.hw_count)
                + " RX sw count: " + std::to_string(_rx_stream.sw_count)
                + " RX user count: " + std::to_string(_rx_stream.user_count)
                + " TX hw count: " + std::to_string(_tx_stream.hw_count)
                + " TX sw count: " + std::to_string(_tx_stream.sw_count)
                + " TX user count: " + std::to_string(_tx_stream.user_count);
    } else if (key == "DMA_BUFFER_RX_HW_COUNT") {
        return std::to_string(_rx_stream.hw_count);
    } else if (key == "DMA_BUFFER_RX_SW_COUNT") {
        return std::to_string(_rx_stream.sw_count);
    } else if (key == "DMA_BUFFER_RX_USER_COUNT") {
        return std::to_string(_rx_stream.user_count);
    } else if (key == "DMA_BUFFER_TX_HW_COUNT") {
        return std::to_string(_tx_stream.hw_count);
    } else if (key == "DMA_BUFFER_TX_SW_COUNT") {
        return std::to_string(_tx_stream.sw_count);
    } else if (key == "DMA_BUFFER_TX_USER_COUNT") {
        return std::to_string(_tx_stream.user_count);
    } else
        throw std::runtime_error("SoapyXTRX::readSetting(" + key + ") unknown key");
}

void SoapyXTRX::writeSetting(const std::string &key, const std::string &value) {
    SoapySDR::logf(SOAPY_SDR_DEBUG, "SoapyXTRX::writeSetting(%s, %s)",
                   key.c_str(), value.c_str());

    std::lock_guard<std::mutex> lock(_mutex);

    // undo any changes caused by one of the other keys with these enable calls
    if (key == "RXTSP_ENABLE")
        LMS7002M_rxtsp_enable(_lms, LMS_CHAB, value == "TRUE");
    else if (key == "TXTSP_ENABLE")
        LMS7002M_txtsp_enable(_lms, LMS_CHAB, value == "TRUE");
    else if (key == "RBB_ENABLE")
        LMS7002M_rbb_enable(_lms, LMS_CHAB, value == "TRUE");
    else if (key == "TBB_ENABLE")
        LMS7002M_tbb_enable(_lms, LMS_CHAB, value == "TRUE");
    else if (key == "TRF_ENABLE_LOOPBACK")
        LMS7002M_trf_enable_loopback(_lms, LMS_CHAB, value == "TRUE");
    else if (key == "CGEN")
        lms7Device->SetClockFreq(LMS_CLOCK_REF, std::stod(value));
    else if (key == "RXTSP_TSG_CONST") {
        const int ampl = std::stoi(value);
        LMS7002M_rxtsp_tsg_const(_lms, LMS_CHAB, ampl, 0);
    } else if (key == "TXTSP_TSG_CONST") {
        const int ampl = std::stoi(value);
        LMS7002M_txtsp_tsg_const(_lms, LMS_CHAB, ampl, 0);
    } else if (key == "TBB_ENABLE_LOOPBACK") {
        SoapySDR::log(SOAPY_SDR_DEBUG, "Setting TBB loopback");
        int path = 0;
        if (value == "LB_DISCONNECTED")
            path = LMS7002M_TBB_LB_DISCONNECTED;
        else if (value == "LB_DAC_CURRENT")
            path = LMS7002M_TBB_LB_DAC_CURRENT;
        else if (value == "LB_LB_LADDER")
            path = LMS7002M_TBB_LB_LB_LADDER;
        else if (value == "LB_MAIN_TBB")
            path = LMS7002M_TBB_LB_MAIN_TBB;
        else
            throw std::runtime_error("SoapyXTRX::writeSetting(" + key + ", " +
                                     value + ") unknown value");
        LMS7002M_tbb_enable_loopback(_lms, LMS_CHAB, path, false);
    } else if (key == "TBB_SET_PATH") {
        int path = 0;
        if (value == "TBB_BYP")
            path = LMS7002M_TBB_BYP;
        else if (value == "TBB_S5")
            path = LMS7002M_TBB_S5;
        else if (value == "TBB_LAD")
            path = LMS7002M_TBB_LAD;
        else if (value == "TBB_LBF")
            path = LMS7002M_TBB_LBF;
        else if (value == "TBB_HBF")
            path = LMS7002M_TBB_HBF;
        else
            throw std::runtime_error("SoapyXTRX::writeSetting(" + key + ", " +
                                     value + ") unknown value");
        LMS7002M_tbb_set_path(_lms, LMS_CHAB, path);
    } else if (key == "RBB_SET_PATH") {
        int path = 0;
        if (value == "BYP")
            path = LMS7002M_RBB_BYP;
        else if (value == "LBF")
            path = LMS7002M_RBB_LBF;
        else if (value == "HBF")
            path = LMS7002M_RBB_HBF;
        else if (value == "LB_BYP")
            path = LMS7002M_RBB_LB_BYP;
        else if (value == "LB_LBF")
            path = LMS7002M_RBB_LB_LBF;
        else if (value == "LB_HBF")
            path = LMS7002M_RBB_LB_HBF;
        else if (value == "PDET")
            path = LMS7002M_RBB_PDET;
        else
            throw std::runtime_error("SoapyXTRX::writeSetting(" + key + ", " +
                                     value + ") unknown value");
        LMS7002M_rbb_set_path(_lms, LMS_CHAB, path);
    } else if (key == "LOOPBACK_ENABLE") {
        SoapySDR::log(SOAPY_SDR_DEBUG, "Setting Digital Loopback");
        if (value == "TRUE") {
//            LMS7002M_setup_digital_loopback(_lms);
            lms7Device->SetDigitalLoopback();
        } else if (value == "FALSE") {
            LMS7002M_configure_lml_port(_lms, LMS_PORT2, LMS_TX, 1);
            LMS7002M_configure_lml_port(_lms, LMS_PORT1, LMS_RX, 1);
        } else
            throw std::runtime_error("SoapyXTRX::writeSetting(" + key + ", " +
                                     value + ") unknown value");
    } else if (key == "LOOPBACK_ENABLE_LFSR") {
        SoapySDR::log(SOAPY_SDR_DEBUG, "Setting LFSR Loopback");
        if (value == "TRUE") {
            LMS7002M_setup_digital_loopback_lfsr(_lms);
        } else if (value == "FALSE") {
            LMS7002M_configure_lml_port(_lms, LMS_PORT2, LMS_TX, 1);
            LMS7002M_configure_lml_port(_lms, LMS_PORT1, LMS_RX, 1);
        } else
            throw std::runtime_error("SoapyXTRX::writeSetting(" + key + ", " +
                                     value + ") unknown value");
    } else if (key == "TRF_ENABLE_LOOPBACK") {
        LMS7002M_trf_enable_loopback(_lms, LMS_CHAB, value == "TRUE");
    } else if (key == "RESET_RX_FIFO") {
        LMS7002M_reset_lml_fifo(_lms, LMS_RX);
    } else if (key == "FPGA_TX_RX_LOOPBACK_ENABLE") {
        SoapySDR::log(SOAPY_SDR_DEBUG, "Setting FPGA TX-RX Loopback");
        uint32_t control = litepcie_readl(_fd, CSR_LMS7002M_CONTROL_ADDR);
        control &= ~(1 << CSR_LMS7002M_CONTROL_TX_RX_LOOPBACK_ENABLE_OFFSET);
        if (value == "TRUE") {
            control |= (1 << CSR_LMS7002M_CONTROL_TX_RX_LOOPBACK_ENABLE_OFFSET);
        } else if (value != "FALSE")
            throw std::runtime_error("SoapyXTRX::writeSetting(" + key + ", " +
                                     value + ") unknown value");
        litepcie_writel(_fd, CSR_LMS7002M_CONTROL_ADDR, control);
    } else if (key == "FPGA_DMA_LOOPBACK_ENABLE") {
        SoapySDR::log(SOAPY_SDR_DEBUG, "Setting FPGA DMA Loopback");
        if (value == "TRUE")
             dma_set_loopback(_fd, true);
        else if (value == "FALSE")
             dma_set_loopback(_fd, false);
        else
            throw std::runtime_error("SoapyXTRX::writeSetting(" + key + ", " +
                                     value + ") unknown value");

    } else if (key == "FPGA_TX_PATTERN") {
        SoapySDR::log(SOAPY_SDR_DEBUG, "Setting FPGA TX pattern");
        uint32_t control = litepcie_readl(_fd, CSR_LMS7002M_TX_PATTERN_CONTROL_ADDR);
        control &= ~(1 << CSR_LMS7002M_TX_PATTERN_CONTROL_ENABLE_OFFSET);
        if (value == "1") {
            control |= 1 << CSR_LMS7002M_TX_PATTERN_CONTROL_ENABLE_OFFSET;
        } else if (value != "0")
            throw std::runtime_error("SoapyXTRX::writeSetting(" + key + ", " +
                                     value + ") unknown value");
        litepcie_writel(_fd, CSR_LMS7002M_TX_PATTERN_CONTROL_ADDR, control);
    } else if (key == "FPGA_RX_PATTERN") {
        SoapySDR::log(SOAPY_SDR_DEBUG, "Setting FPGA RX pattern");
        uint32_t control = litepcie_readl(_fd, CSR_LMS7002M_RX_PATTERN_CONTROL_ADDR);
        control &= ~(1 << CSR_LMS7002M_RX_PATTERN_CONTROL_ENABLE_OFFSET);
        if (value == "1") {
            control |= 1 << CSR_LMS7002M_RX_PATTERN_CONTROL_ENABLE_OFFSET;
        } else if (value != "0")
            throw std::runtime_error("SoapyXTRX::writeSetting(" + key + ", " +
                                     value + ") unknown value");
        litepcie_writel(_fd, CSR_LMS7002M_RX_PATTERN_CONTROL_ADDR, control);
    } else if (key == "FPGA_TX_DELAY") {
        int delay = std::stoi(value);
        if (delay < 0 || delay > 31)
            throw std::runtime_error("SoapyXTRX::writeSetting(" + key + ", " +
                                     value + ") invalid value");
        uint32_t reg = litepcie_readl(_fd, CSR_LMS7002M_DELAY_ADDR);
        uint32_t mask = ((uint32_t)(1 << CSR_LMS7002M_DELAY_TX_DELAY_SIZE)-1) << CSR_LMS7002M_DELAY_TX_DELAY_OFFSET;
        litepcie_writel(_fd, CSR_LMS7002M_DELAY_ADDR,
                        (reg & ~mask) | (delay << CSR_LMS7002M_DELAY_TX_DELAY_OFFSET));
    } else if (key == "FPGA_RX_DELAY") {
        int delay = std::stoi(value);
        if (delay < 0 || delay > 31)
            throw std::runtime_error("SoapyXTRX::writeSetting(" + key + ", " +
                                     value + ") invalid value");
        uint32_t reg = litepcie_readl(_fd, CSR_LMS7002M_DELAY_ADDR);
        uint32_t mask = ((uint32_t)(1 << CSR_LMS7002M_DELAY_RX_DELAY_SIZE)-1) << CSR_LMS7002M_DELAY_RX_DELAY_OFFSET;
        litepcie_writel(_fd, CSR_LMS7002M_DELAY_ADDR,
                        (reg & ~mask) | (delay << CSR_LMS7002M_DELAY_RX_DELAY_OFFSET));
    } else if (key == "CALIBRATE_TX")
    {
        for (size_t channel = 0; channel < lms7Device->GetNumChannels(); channel++)
        {
            this->writeSetting(SOAPY_SDR_TX, channel, "CALIBRATE_TX", value);
        }
    } else if (key == "CALIBRATE_RX")
    {
        for (size_t channel = 0; channel < lms7Device->GetNumChannels(); channel++)
        {
            this->writeSetting(SOAPY_SDR_RX, channel, "CALIBRATE_RX", value);
        }
    } else if (key == "DUMP_INI") {
        LMS7002M_dump_ini(_lms, value.c_str());
    } else if (key == "RXTSP_TONE") {
        LMS7002M_rxtsp_tsg_tone_div(_lms, LMS_CHAB, std::stoi(value));
    } else if (key == "TXTSP_TONE") {
        LMS7002M_txtsp_tsg_tone_div(_lms, LMS_CHAB, std::stoi(value));
    } else if (key == "RXTSP_ENABLE") {
        LMS7002M_rxtsp_enable(_lms, LMS_CHAB, value == "TRUE");
    } else if (key == "TXTSP_ENABLE") {
        LMS7002M_txtsp_enable(_lms, LMS_CHAB, value == "TRUE");
    } else if (key == "GPS_ENABLE") {
        if (value == "TRUE") {
            SoapySDR::log(SOAPY_SDR_DEBUG, "Enabling GPS");
            litepcie_writel(_fd, CSR_GPS_CONTROL_ADDR, 0 * (1 << CSR_GPS_CONTROL_ENABLE_OFFSET));
        } else if (value == "FALSE") {
            SoapySDR::log(SOAPY_SDR_DEBUG, "Disabling GPS");
            litepcie_writel(_fd, CSR_GPS_CONTROL_ADDR, 1 * (1 << CSR_GPS_CONTROL_ENABLE_OFFSET));
        } else {
            throw std::runtime_error("SoapyXTRX::writeSetting(" + key + ", " +
                                     value + ") unknown value");
        }
    } else if (key == "DAC_SET") {
        vctcxo_dac_set(std::stoi(value));
    } else if (key == "LITEX_DUMP_INI") {
        dump_litex_regs(value);
    } else if (key == "SET_REGISTER") {
        std::stringstream ssin(value);
        std::string register_addr_string;
        std::string register_value_string;
        if(ssin.good()) {
            ssin >> register_addr_string;
        }
        if(ssin.good()) {
            ssin >> register_value_string;
        }
        size_t register_addr = std::stoul(register_addr_string, nullptr, 16);
        size_t register_value = std::stoul(register_value_string, nullptr, 16);
        SoapySDR::logf(SOAPY_SDR_DEBUG, "writeRegister(0x%04x, 0x%04x)", register_addr, register_value);
        writeRegister(register_addr, register_value);
    } else if (key == "READ_REGISTER") {
        size_t register_addr = std::stoul(value, nullptr, 16);
        size_t register_value = readRegister(register_addr);
        SoapySDR::logf(SOAPY_SDR_DEBUG, "readRegister(0x%04x) : 0x%04x", register_addr, register_value);
    } else
        throw std::runtime_error("SoapyXTRX::writeSetting(" + key + ", " +
                                 value + ") unknown key");
}

void SoapyXTRX::writeSetting(const int direction, const size_t channel, const std::string &key, const std::string &value) {
    std::unique_lock<std::recursive_mutex> lock(_accessMutex);
    const bool isTx = (direction == SOAPY_SDR_TX);
    
    if (key == "DC_OFFSET_WINDOW") {
        LMS7002M_rxtsp_set_dc_correction_window(_lms, ch2LMS(channel), std::stoi(value));
    } else if ((key == "CALIBRATE_TX" or key == "CALIBRATE_EXTERNAL_TX") or (isTx and (key == "CALIBRATE" or key == "CALIBRATE_EXTERNAL")))
    {
        unsigned int is_external = (key == "CALIBRATE_EXTERNAL_TX" or key == "CALIBRATE_EXTERNAL");
        double bw = std::stof(value);
        SoapySDR::logf(SOAPY_SDR_INFO, "Calibrate Tx %f", bw);
        if (lms7Device->Calibrate(true, channel, bw, is_external)!=0)
            throw std::runtime_error(lime::GetLastErrorMessage());
        _channelsToCal.erase(std::make_pair(direction, channel));
        mChannels[direction].at(channel).cal_bw = bw;
    } else if ((key == "CALIBRATE_RX" or key == "CALIBRATE_EXTERNAL_RX") or (not isTx and (key == "CALIBRATE" or key == "CALIBRATE_EXTERNAL")))
    {
        unsigned int is_external = (key == "CALIBRATE_EXTERNAL_RX" or key == "CALIBRATE_EXTERNAL");
        double bw = std::stof(value);
        SoapySDR::logf(SOAPY_SDR_INFO, "CalibrateRx %f", bw);
        if (lms7Device->Calibrate(false, channel, bw, is_external)!=0)
            throw std::runtime_error(lime::GetLastErrorMessage());
        _channelsToCal.erase(std::make_pair(direction, channel));
        mChannels[direction].at(channel).cal_bw = bw;
    } else
        throw std::runtime_error("SoapyXTRX::writeChannelSetting(" + key + ", " +
                                 value + ") unknown key");
}

std::string SoapyXTRX::readSetting(const int direction, const size_t channel, const std::string &key) const {
    if (key == "DC_OFFSET_WINDOW" && direction == SOAPY_SDR_RX) {
//        return std::to_string(LMS7002M_rxtsp_get_dc_correction_window(_lms, ch2LMS(channel)));
    } else
        throw std::runtime_error("SoapyXTRX::readChannelSetting(" + key + ") unknown key");
}



void SoapyXTRX::writeI2C(const int addr, const std::string &data){


}

std::string SoapyXTRX::readI2C(const int addr, const size_t numBytes){
    unsigned char data[numBytes];
    i2c0_read(addr&0xff, addr, data, numBytes, true);
    return std::string((char*)data, numBytes);
}


std::vector<std::string> SoapyXTRX::listUARTs(void) const {
    std::vector<std::string> interfaces;
    interfaces.push_back("GPS");
    interfaces.push_back("LiteX");
    return interfaces;
}

void SoapyXTRX::writeUART(const std::string &which, const std::string &data) {
    if (which == "GPS") {
        throw std::runtime_error("SoapyXTRX::writeUART(" + which + ") GPS does not support write");
    } else if (which == "LiteX") {
        for (size_t i = 0; i < data.length(); i++) {
            litepcie_writel(_fd, CSR_UART_RXTX_ADDR, data[i]);
        }
    } else
        throw std::runtime_error("SoapyXTRX::writeUART(" + which + ") unknown interface");
}

std::string SoapyXTRX::readUART(const std::string &which, const long timeoutUs = 100000) const {
    std::string ret_str = "";
    if (which == "GPS" || which == "LiteX") {
        auto tstart = std::chrono::high_resolution_clock::now();
        while (true) {
            char c;
            if (which == "GPS") {
                if (litepcie_readl(_fd, CSR_GPS_UART_RXEMPTY_ADDR) == 0) {
                    c = litepcie_readl(_fd, CSR_GPS_UART_RXTX_ADDR);
                    ret_str.push_back(c);
                }
            } else { // which == "LiteX"
                if (litepcie_readl(_fd, CSR_UART_RXEMPTY_ADDR) == 0) {
                    c = litepcie_readl(_fd, CSR_UART_RXTX_ADDR);
                    ret_str.push_back(c);
                }
            }
            auto elapsed = std::chrono::high_resolution_clock::now() - tstart;
            long long elapsedUs = std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count();
            if (elapsedUs > timeoutUs || c == '\n' || c == '\r') {
                break;
            }
        }
        return ret_str;
    } else {
        throw std::runtime_error("SoapyXTRX::readUART(" + which + ") unknown interface");
    }
}

/*******************************************************************
 * Device Utils
 ******************************************************************/
int SoapyXTRX::board_get_revision(void) {
    /* Get board revision from SPI DACs:
       - XTRX Rev4 is equipped with a MCP4725.
       - XTRX Rev5 is equipped with a DAC60501.
       The DAC60501 has the particularity of only accepting write commands,
       so we detect MCP4725 presence (and thus Rev4 revision) by doing a
       I2C read to the MCP4725 I2C address.
    */

    /* Check MCP4725 presence */
    int has_mcp4725;
    i2c1_start();
    has_mcp4725 = i2c1_transmit_byte(I2C1_ADDR_RD(MCP4725_I2C_ADDR));
    i2c1_stop();

    if (has_mcp4725) {
        dac_addr = MCP4725_I2C_ADDR;
        return 4;
    } else {
        dac_addr = DAC60501_I2C_ADDR;
        return 5;
    }
}

void SoapyXTRX::vctcxo_dac_set(int value) {
    unsigned char cmd;
    unsigned char dat[2];
    bool ret;

    value = value & 0xfff; /* 12-bit full range clamp */

    /* Rev4 is equipped with a MCP4725 */
    if (board_revision == 4) {
        cmd = (0b0000 << 4) | (value >> 8);
        dat[0] = (value & 0xff);
        i2c1_write(MCP4725_I2C_ADDR, cmd, dat, 1);
        /* Rev5 is equipped with a DAC60501 */
    } else {
        // Bottom four bits are ignored on DAC60501
        value <<= 4;
        dat[0] = (value & 0xff00) >> 8;
        dat[1] = value & 0xff;
        cmd = 0x08;
        ret = i2c1_write(DAC60501_I2C_ADDR, cmd, dat, 2);
        if (!ret) {
            printf("DAC write failed, err: %d\n", ret);
        }
    }
}

/*
Dump the LiteX registers in an INI format similar to the one used by limesuite
*/
void SoapyXTRX::dump_litex_regs(std::string filename) {
    FILE *f = fopen(filename.c_str(), "w");
    if (!f) {
        printf("Failed to open file %s)", filename.c_str());
        return;
    }
    struct csr_block {
        uint32_t addr;
        std::string name;
        uint32_t length;
    };

    std::vector<csr_block> reg_addrs = {
        {CSR_VCTCXO_BASE, "VCTCXO", 0x08},
        {CSR_RF_SWITCHES_BASE, "RF_SWITCHES", 0x04},
        {CSR_LMS7002M_BASE, "LMS", 0x2c},
    };
    fprintf(f, "[FILE INFO]\ntype=litex csr configuration\nversion=1.0\n");

    for (auto csr : reg_addrs)
    {
        fprintf(f, "[%s]\n", csr.name.c_str());
        for (uint32_t i = 0; i < csr.length; i=i+4) {
            uint32_t addr = csr.addr + i;
            uint32_t val = litepcie_readl(_fd, addr);
            fprintf(f, "0x%02x=0x%08x\n", addr, val);
        }
    }
    fclose(f);
}

void * SoapyXTRX::getNativeDeviceHandle(void) const {
    return getLMS7Handle();
}

void * SoapyXTRX::getLMS7Handle() const {
    return this->_lms;
}

/***********************************************************************
 * Find available devices
 **********************************************************************/

std::string getXTRXIdentification(int fd) {
    char fpga_identification[256];
    for (int i = 0; i < 256; i ++)
        fpga_identification[i] = litepcie_readl(fd, CSR_IDENTIFIER_MEM_BASE + 4 * i);
    return std::string(&fpga_identification[0]);
}

std::string getXTRXSerial(int fd) {
    char serial[32];
    snprintf(serial, 32, "%x%08x",
                litepcie_readl(fd, CSR_DNA_ID_ADDR + 4 * 0),
                litepcie_readl(fd, CSR_DNA_ID_ADDR + 4 * 1));
    return std::string(&serial[0]);
}

std::vector<SoapySDR::Kwargs> findXTRX(const SoapySDR::Kwargs &args) {
    std::vector<SoapySDR::Kwargs> discovered;
    if (args.count("path") != 0) {
        // respect user choice
        int fd = open(args.at("path").c_str(), O_RDWR);
        if (fd < 0)
            throw std::runtime_error("Invalid device path specified (should be an accessible device node)");

        // gather device info
        SoapySDR::Kwargs dev(args);
        dev["serial"] = getXTRXSerial(fd);
        dev["identification"] = getXTRXIdentification(fd);
        close(fd);

        discovered.push_back(dev);
    } else {
        // find all LitePCIe devices
        for (int i = 0; i < 10; i++) {
            std::string path = "/dev/litepcie" + std::to_string(i);
            int fd = open(path.c_str(), O_RDWR);
            if (fd < 0)
                continue;

            // check the FPGA identification to see if this is an XTRX
            std::string fpga_identification = getXTRXIdentification(fd);
            if (strstr(fpga_identification.c_str(), "LiteX SoC on Fairwaves XTRX") != NULL) {
                // gather device info
                SoapySDR::Kwargs dev(args);
                dev["path"] = path;
                dev["serial"] = getXTRXSerial(fd);
                dev["identification"] = &fpga_identification[0];
                close(fd);

                // filter by serial if specified
                if (args.count("serial") != 0) {
                    // filter on serial number
                    if (args.at("serial") != dev["serial"])
                        continue;
                }

                discovered.push_back(dev);
            }
        }
    }

    return discovered;
}


/***********************************************************************
 * Make device instance
 **********************************************************************/

SoapySDR::Device *makeXTRX(const SoapySDR::Kwargs &args) {
    return new SoapyXTRX(argsToHandle(args), args);
}


/***********************************************************************
 * Registration
 **********************************************************************/

static SoapySDR::Registry registerXTRX("XTRXLime", &findXTRX, &makeXTRX,
                                       SOAPY_SDR_ABI_VERSION);
