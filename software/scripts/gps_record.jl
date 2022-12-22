using SoapySDR, SoapySDRRecorder, Unitful, XTRX, Base.Threads

@assert Threads.nthreads() >= 2

# Channel confuguration callback
function configuration(dev, chans)

    #dev[SoapySDR.Setting("biastee")] = "true"

    for rx in chans
        rx.sample_rate = 5u"MHz"
        rx.bandwidth = 5u"MHz"
        rx.frequency = 1_575_420_000u"Hz"
        rx.gain = 100u"dB"
        @show rx
    end

end

function telemetry_callback(dev, chans)
    # this is terribly low level, but for some reason the highlevel API
    # segfaults julia, so we keep it all in C
    @ccall printf("%s\n"::Cstring, SoapySDR.SoapySDRDevice_readSetting(dev.ptr, "DMA_BUFFERS")::Ptr{Cstring})::Cint
end

function csv_header_callback(io)
    # this is terribly low level, but for some reason the highlevel API
    # segfaults julia, so we keep it all in C
    @ccall fprintf(io::Ptr{Cint}, "dma_hw_count,dma_sw_count,dma_user_count,"::Cstring)::Cint
end


function csv_log_callback(io, dev, chans)
    # this is terribly low level, but for some reason the highlevel API
    # segfaults julia, so we keep it all in C
    hw = SoapySDR.SoapySDRDevice_readSetting(dev.ptr, "DMA_BUFFER_RX_HW_COUNT")
    sw = SoapySDR.SoapySDRDevice_readSetting(dev.ptr, "DMA_BUFFER_RX_SW_COUNT")
    user = SoapySDR.SoapySDRDevice_readSetting(dev.ptr, "DMA_BUFFER_RX_USER_COUNT")
    @ccall fprintf(io::Ptr{Cint}, "%s,%s,%s,"::Cstring, hw::Ptr{Cstring}, sw::Ptr{Cstring}, user::Ptr{Cstring})::Cint
end


devs = collect(Devices());
dev1 = Device(only(filter(x -> x["driver"] == "XTRXLime" && x["serial"] == "121c444ea8c85c", devs)));
dev2 = Device(only(filter(x -> x["driver"] == "XTRXLime" && x["serial"] == "30c5241b884854", devs)));


synchro_dev1 = dev1[(SoapySDR.Register("LitePCI"),XTRX.CSRs.CSR_SYNCHRO_CONTROL_ADDR)] 
synchro_dev2 = dev2[(SoapySDR.Register("LitePCI"),XTRX.CSRs.CSR_SYNCHRO_CONTROL_ADDR)] 
@show synchro_dev1, synchro_dev2
dev1[(SoapySDR.Register("LitePCI"),XTRX.CSRs.CSR_SYNCHRO_CONTROL_ADDR)] = synchro_dev1 | (2 << XTRX.CSRs.CSR_SYNCHRO_CONTROL_INT_SOURCE_OFFSET | 2 << XTRX.CSRs.CSR_SYNCHRO_CONTROL_OUT_SOURCE_OFFSET)
dev2[(SoapySDR.Register("LitePCI"),XTRX.CSRs.CSR_SYNCHRO_CONTROL_ADDR)] = synchro_dev2 | (2 << XTRX.CSRs.CSR_SYNCHRO_CONTROL_INT_SOURCE_OFFSET | 2 << XTRX.CSRs.CSR_SYNCHRO_CONTROL_OUT_SOURCE_OFFSET)

format = dev1.rx[1].native_stream_format
fullscale = dev1.tx[1].fullscale
dev1.clock_source = "external+pps"
dev2.clock_source = "external+pps"

GC.gc()

GC.enable(false)

t1 = Threads.@spawn SoapySDRRecorder.record("/mnt/data_disk/gps_record13_dev1", device=dev1, channel_configuration=configuration,
    telemetry_callback=telemetry_callback, csv_log = true, timer_display=false, csv_log_callback=csv_log_callback, csv_header_callback=csv_header_callback, timeout=0,
    compress=false, compression_level=1)

t2 = Threads.@spawn SoapySDRRecorder.record("/mnt/data_disk/gps_record13_dev2", device=dev2, channel_configuration=configuration,
    telemetry_callback=telemetry_callback, csv_log = true, timer_display=false, csv_log_callback=csv_log_callback, csv_header_callback=csv_header_callback, timeout=0,
    compress=false, compression_level=1)

wait(t1)
wait(t2)