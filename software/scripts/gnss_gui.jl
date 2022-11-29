# Replace SoapyLMS7_jll with whatever SoapySDR driver that you need
#using SoapyRTLSDR_jll
using SoapySDR
using GNSSReceiver, GNSSSignals, Unitful
using Tracking

# You'll might want to run it twice for optimal performance.
gnss_receiver_gui(;
    system = GPSL1(),
    sampling_freq = 2e6u"Hz",
    acquisition_time = 4u"ms", # A longer time increases the SNR for satellite acquisition, but also increases the computational load. Must be longer than 1ms
    run_time = 40u"s",
    dev_args = first(Devices(serial="18c5241b88485c")),
    num_ants = Tracking.NumAnts(2) # Number of antenna channels
)

gnss_receiver_gui(;
    system = GPSL1(),
    sampling_freq = 2e6u"Hz",
    acquisition_time = 4u"ms", # A longer time increases the SNR for satellite acquisition, but also increases the computational load. Must be longer than 1ms
    run_time = 120u"s",
    dev_args = first(Devices(serial="18c5241b88485c")),
    num_ants = Tracking.NumAnts(2) # Number of antenna channels
)