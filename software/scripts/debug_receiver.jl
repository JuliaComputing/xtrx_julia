ENV["GKSwstype"] = "100"

using SoapySDR, Unitful, DSP, LibSigflow, Statistics, LinearAlgebra, Plots, FFTW

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

        measurement = collect_single_chunk_at(reshunked_channel, counter_threshold = 1000) # After 100 ms

        sleep(10)

        measurement
    end
end

function main()
    sample_rate = 5e6u"Hz"
    measurement = debug_signal(sample_rate = sample_rate)
    for i = 1:size(measurement, 2)
        p = periodogram(measurement[:,i], fs = sample_rate.val)
        pl = plot(fftshift(freq(p) / 1e6), fftshift(10 * log10.(power(p))), ylabel = "Power (dB)", xlabel = "Frequency (MHz)")
        savefig(pl, "debug/spectogram_channel$i.png")
        pl = plot(hcat(real.(measurement[:,i]), imag.(measurement[:,i])), ylabel = "Amplitude", xlabel = "Samples", label = ["Real" "Imag"])
        savefig(pl, "debug/measurement_channel$i.png")
    end
end

isinteractive() || main()