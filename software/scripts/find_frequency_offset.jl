ENV["GKSwstype"] = "100"

using SoapySDR,
    Unitful,
    DSP,
    LibSigflow,
    Statistics,
    LinearAlgebra,
    Plots,
    FFTW,
    Acquisition,
    GNSSSignals,
    OrderedCollections,
    Tracking,
    GNSSReceiver,
    PositionVelocityTime

function estimate_frequency_offset_roughly(;
    sample_rate = 5e6u"Hz",
    run_time = 4u"s",
)
    time_for_acquisition = 4u"ms"
    reshunk_size = Int(upreferred(sample_rate * time_for_acquisition))

    device_kwargs = Dict{Symbol,Any}()
    if chomp(String(read(`hostname`))) == "pathfinder"
        device_kwargs[:driver] = "XTRX"
        device_kwargs[:serial] = "12cc5241b88485c"
    end
    device_kwargs[:driver] = "XTRXLime"
    gpsl1 = GPSL1()

    measurement = Device(first(Devices(;device_kwargs...))) do dev

        format = dev.rx[1].native_stream_format

        # Setup transmitter parameters
        ct = dev.tx[1]
        ct.bandwidth = sample_rate
        ct.sample_rate = sample_rate

        # Setup receive parameters
        for cr in dev.rx
            cr.antenna = :LNAW
            cr.bandwidth = sample_rate
            cr.sample_rate = sample_rate
            cr.gain = 100u"dB"
            cr.gain_mode = false
            cr.frequency = get_center_frequency(gpsl1)
        end

        for cr in dev.rx
            cr[SoapySDR.Setting("CALIBRATE")] = "5e6"
        end

        stream_rx = SoapySDR.Stream(format, dev.rx)

        num_total_samples = Int(upreferred(sample_rate * run_time))

        # RX reads the buffers in
        samples_channel = stream_data(stream_rx, num_total_samples; leadin_buffers=0)

        reshunked_channel = rechunk(samples_channel, reshunk_size)

        measurement, spawn_event = collect_single_chunk_at(
            reshunked_channel,
            counter_threshold = Int(upreferred(run_time / time_for_acquisition)) - 100
        ) # 100 * time_for_acquisition before end

        wait(spawn_event)

        measurement
    end
    frequency_offsets_to_test = -15u"kHz":2.5u"kHz":15u"kHz"
    num_frequency_offsets = length(frequency_offsets_to_test)
    num_sats = zeros(num_frequency_offsets)
    for i in 1:num_frequency_offsets
        acq_res = coarse_fine_acquire(gpsl1, measurement[:,1], sample_rate, 1:32, interm_freq = frequency_offsets_to_test[i])
        acq_res_valid = filter(x -> x.CN0 > 43, acq_res)
        num_sats[i] = length(acq_res_valid)
    end
    max_sats, max_sats_idx = findmax(num_sats)
    # If multiple offsets result into the same number of satellites, pick the middle one
    counter = 0
    for i = max_sats_idx + 1:num_frequency_offsets
        num_sats[i] < max_sats && break
        counter += 1
    end
    best_frequency_offset_idx = max_sats_idx + counter >> 1
    @info "Your offset is probably around $(frequency_offsets_to_test[best_frequency_offset_idx]). I found $max_sats satellites using this frequency offstet." OrderedDict(frequency_offsets_to_test .=> num_sats)
    upreferred(frequency_offsets_to_test[best_frequency_offset_idx] / get_center_frequency(gpsl1))
end

function estimate_frequency_offset(;
    system = GPSL1(),
    sampling_freq = 5e6u"Hz",
    acquisition_time = 4u"ms", # A longer time increases the SNR for satellite acquisition, but also increases the computational load. Must be longer than 1ms
    num_ants = NumAnts(2),
    dev_args = first(Devices()),
    gain::Union{Nothing, <:Unitful.Gain} = nothing
)
    clock_drift = estimate_frequency_offset_roughly(;sample_rate = sampling_freq)

    num_samples_acquisition = Int(upreferred(sampling_freq * acquisition_time))

    found_offset_event = Base.Event()

    Device(dev_args) do dev

#        dac_val = UInt16(1)*0x111
#        dev[SoapySDR.Setting("DAC_SET")] = string(dac_val)
        adjustment = -clock_drift * sampling_freq
        ct = dev.tx[1]
        ct.bandwidth = sampling_freq - adjustment
        ct.sample_rate = sampling_freq - adjustment

        for crx in dev.rx
            crx.antenna = :LNAW
            crx.sample_rate = sampling_freq - adjustment
            crx.bandwidth = sampling_freq - adjustment
            if isnothing(gain)
                crx.gain_mode = true
            else
                crx.gain = gain
            end
            crx.frequency = get_center_frequency(system)
        end

        stream = SoapySDR.Stream(first(dev.rx).native_stream_format, dev.rx)

        # Getting samples in chunks of `mtu`
        data_stream = stream_data(stream, found_offset_event)

        # Satellite acquisition takes about 1s to process on a recent laptop
        # Let's take a buffer length of 5s to be on the safe side
        buffer_length = 5u"s"
        buffered_stream = membuffer(data_stream, ceil(Int, buffer_length * sampling_freq / stream.mtu))

        # Resizing the chunks to acquisition length
        reshunked_stream = rechunk(buffered_stream, num_samples_acquisition)

        # Performing GNSS acquisition and tracking
        data_channel = receive(
            reshunked_stream,
            system,
            sampling_freq;
            num_ants,
            num_samples = num_samples_acquisition,
            interm_freq = clock_drift * get_center_frequency(system)
        )

        data_channel1, data_channel2 = GNSSReceiver.tee(data_channel)

        gui_channel = get_gui_data_channel(data_channel1)

        precise_clock_drift = 0.0
        search_for_pvt_event = Base.errormonitor(Threads.@spawn begin 
            GNSSReceiver.consume_channel(data_channel2) do receiver_data
                # Got a PVT solution?
                if receiver_data.pvt.time_correction != 0
                    precise_clock_drift = clock_drift + receiver_data.pvt.relative_clock_drift
                    @info get_LLA(receiver_data.pvt) precise_clock_drift receiver_data.pvt.relative_clock_drift clock_drift
                    notify(found_offset_event)
                end
            end
        end)

        # Display the GUI and block
        GNSSReceiver.gui(gui_channel)
        wait(search_for_pvt_event)
        precise_clock_drift
    end
end