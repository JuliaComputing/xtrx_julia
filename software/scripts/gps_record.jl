using SoapySDR, SoapySDRRecorder, Unitful, XTRX, Base.Threads

synchro = false

# Channel confuguration callback
function configuration(dev, chans)

    #dev[SoapySDR.Setting("biastee")] = "true"
    for tx in dev.tx
        tx.sample_rate = 5u"MHz"
        tx.bandwidth = 5u"MHz"
        tx.frequency = 1_575_420_000u"Hz"
    end 

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

recorder_tasks = []

dev_list = []


for dev_id in ("12cc5241b88485c", "30c5241b884854", "121c444ea8c85c")
    push!(dev_list, (Device(only(filter(x -> x["driver"] == "XTRXLime" && x["serial"] == dev_id, devs))), dev_id))
end

for (dev1, dev_id) in dev_list
    if synchro
        synchro_dev1 = dev1[(SoapySDR.Register("LitePCI"),XTRX.CSRs.CSR_SYNCHRO_CONTROL_ADDR)] 
        dev1[(SoapySDR.Register("LitePCI"),XTRX.CSRs.CSR_SYNCHRO_CONTROL_ADDR)] = synchro_dev1 | (2 << XTRX.CSRs.CSR_SYNCHRO_CONTROL_INT_SOURCE_OFFSET | 2 << XTRX.CSRs.CSR_SYNCHRO_CONTROL_OUT_SOURCE_OFFSET)
        dev1.clock_source = "external+pps"
    else
        dev1.clock_source = "internal"
    end

    t = Threads.@spawn SoapySDRRecorder.record("/mnt/data_disk/gps_record_$(dev_id)", device=dev1, channel_configuration=configuration,
        telemetry_callback=telemetry_callback, csv_log = true, timer_display=false, csv_log_callback=csv_log_callback, csv_header_callback=csv_header_callback, timeout=100000000,
        compress=true, compression_level=3, initial_buffers=4096)

    #sleep(10)
    push!(recorder_tasks, t)
end

wait(recorder_tasks[1])