ENV["GKSwstype"] = "100"

using SoapySDR, Unitful, DSP, LibSigflow, Statistics, LinearAlgebra, Plots, FFTW, Acquisition, GNSSSignals

function debug_signal(;
    frequency = 1575.42u"MHz",
    sample_rate = 5e6u"Hz",
    run_time = 5u"s",
)
    num_samples_to_track = Int(upreferred(sample_rate * 4u"ms"))

    device_kwargs = Dict{Symbol,Any}()
    if chomp(String(read(`hostname`))) == "pathfinder"
        device_kwargs[:driver] = "XTRX"
        device_kwargs[:serial] = "12cc5241b88485c"
    end
    device_kwargs[:driver] = "XTRXLime"

    Device(first(Devices(;device_kwargs...))) do dev

        format = dev.rx[1].native_stream_format

        # Setup transmitter parameters
        ct = dev.tx[1]
        ct.bandwidth = sample_rate
#        ct.frequency = frequency
        ct.sample_rate = sample_rate
#        ct.gain = 50u"dB"
#        ct.gain_mode = false

        # Setup receive parameters
        for cr in dev.rx
            cr.antenna = :LNAW
            cr.bandwidth = sample_rate
            cr.sample_rate = sample_rate
            cr.gain = 100u"dB"
            cr.gain_mode = false
            cr.frequency = frequency
        end
        display(dev.rx)

        for cr in dev.rx
            cr[SoapySDR.Setting("CALIBRATE")] = "5e6"
        end

        stream_rx = SoapySDR.Stream(format, dev.rx)

        num_total_samples = Int(upreferred(sample_rate * run_time))

        # RX reads the buffers in
        samples_channel = stream_data(stream_rx, num_total_samples; leadin_buffers=0)

        reshunked_channel = rechunk(samples_channel, num_samples_to_track)

        measurement, spawn_event = collect_single_chunk_at(reshunked_channel, counter_threshold = 1000) # After 100 ms

        wait(spawn_event)

        measurement
    end
end

function debug_dac(;
    frequency = 1575.42u"MHz",
    sample_rate = 5e6u"Hz",
    run_time = 3u"s",
    dac_iteration = 0
)
    num_samples_to_track = Int(upreferred(sample_rate * 4u"ms"))

    device_kwargs = Dict{Symbol,Any}()
    if chomp(String(read(`hostname`))) == "pathfinder"
        device_kwargs[:driver] = "XTRX"
        device_kwargs[:serial] = "12cc5241b88485c"
    end
    device_kwargs[:driver] = "XTRXLime"

    Device(first(Devices(;device_kwargs...))) do dev

        dac_val = UInt16(dac_iteration)*0x111
        dev[SoapySDR.Setting("DAC_SET")] = string(dac_val)
        format = dev.rx[1].native_stream_format

        # Setup transmitter parameters
        ct = dev.tx[1]
        ct.bandwidth = sample_rate
#        ct.frequency = frequency
        ct.sample_rate = sample_rate
#        ct.gain = 50u"dB"
#        ct.gain_mode = false

        # Setup receive parameters
        for cr in dev.rx
            cr.antenna = :LNAW
            cr.bandwidth = sample_rate
            cr.sample_rate = sample_rate
            cr.gain = 100u"dB"
            cr.gain_mode = false
            cr.frequency = frequency
        end

        for cr in dev.rx
            cr[SoapySDR.Setting("CALIBRATE")] = "5e6"
        end

        stream_rx = SoapySDR.Stream(format, dev.rx)

        num_total_samples = Int(upreferred(sample_rate * run_time))

        # RX reads the buffers in
        samples_channel = stream_data(stream_rx, num_total_samples; leadin_buffers=0)

        reshunked_channel = rechunk(samples_channel, num_samples_to_track)

        measurement, spawn_event = collect_single_chunk_at(reshunked_channel, counter_threshold = 100) # After 100 ms

        wait(spawn_event)

        measurement
    end
end

function try_debug_dac(dac_iteration, max_tries)
    for i = 1:max_tries
        try
            measurement = debug_dac(dac_iteration = i)
            return measurement
        catch
            @warn "Something failed... retry"
        end
    end
end

function debug_dacs()
    gpsl1 = GPSL1()
    num_dacs = 16
    num_sats_found = zeros(num_dacs)
    acq_ress1 = Any[]
    acq_ress2 = Any[]
    for i = 0:num_dacs - 1
        measurement = try_debug_dac(i, 5)
        acq_res1 = Acquisition.coarse_fine_acquire(gpsl1, measurement[:,1], 5e6u"Hz", 1:32, interm_freq = 11.75u"kHz")
        acq_res2 = Acquisition.coarse_fine_acquire(gpsl1, measurement[:,2], 5e6u"Hz", 1:32, interm_freq = 11.75u"kHz")
        push!(acq_ress1, acq_res1)
        push!(acq_ress2, acq_res2)
        curr_num_sats_found = count(res -> res.CN0 > 43, acq_res1) + count(res -> res.CN0 > 43, acq_res2)
        num_sats_found[i + 1] = curr_num_sats_found
    end
    for i = 0:num_dacs - 1
        @info "Debug DAC" i num_sats_found[i + 1]
    end
    acq_ress1, acq_ress2
end

function main()
    sample_rate = 5e6u"Hz"
    measurement = debug_signal(sample_rate = sample_rate)
    plot_data(measurement, sample_rate)
end

function plot_data(measurement, sample_rate, suffix = "png")
    for i = 1:size(measurement, 2)
        p = periodogram(measurement[:,i], fs = sample_rate.val)
        pl = plot(fftshift(freq(p) / 1e6), fftshift(10 * log10.(power(p))), ylabel = "Power (dB)", xlabel = "Frequency (MHz)")
        savefig(pl, "debug/spectogram_channel$i.$suffix")
        pl = plot(hcat(real.(measurement[:,i]), imag.(measurement[:,i])), ylabel = "Amplitude", xlabel = "Samples", label = ["Real" "Imag"])
        savefig(pl, "debug/measurement_channel$i.$suffix")
    end
end

isinteractive() || main()