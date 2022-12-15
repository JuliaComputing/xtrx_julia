#ENV["SOAPY_SDR_LOG_LEVEL"] = "DEBUG"

ENV["GKSwstype"] = "100"

using SoapySDR, Printf, Unitful, DSP, LibSigflow, SoapyBladeRF_jll, LibSigGUI, Statistics
include("./xtrx_debugging.jl")

if Threads.nthreads() < 2
    error("This script must be run with multiple threads!")
end

function eval_gain_and_phase_mism(;
    frequency = 1575.42u"MHz",
    sample_rate = 6u"MHz",
    baseband_frequency = 0.6u"MHz",
    run_time = 10u"s"
)

    device_kwargs = Dict{Symbol,Any}()
    if chomp(String(read(`hostname`))) == "pathfinder"
        device_kwargs[:driver] = "XTRX"
        device_kwargs[:serial] = "12cc5241b88485c"
    end

    Device(first(Devices(;device_kwargs...))) do dev

        format = dev.rx[1].native_stream_format
        fullscale = dev.tx[1].fullscale

        # Setup transmitter parameters
        ct = dev.tx[1]
        ct.bandwidth = sample_rate
        ct.frequency = frequency
        ct.sample_rate = sample_rate
        ct.gain = 30u"dB"
        ct.gain_mode = false

        # Setup receive parameters
        for cr in dev.rx
            cr.bandwidth = sample_rate
            cr.frequency = frequency
            cr.sample_rate = sample_rate
            # Gain does not seem to have an effect with BladeRF
            # Even if gain_mode is set to false
            # cr.gain = gain
            cr.gain_mode = true
        end

        stream_rx = SoapySDR.Stream(format, dev.rx)

        stream_tx = SoapySDR.Stream(format, dev.tx)

        num_samples = stream_tx.mtu
        num_total_samples = Int(upreferred(sample_rate * run_time))
        sample_range = 0:num_samples - 1
        signals = hcat(
            cis.(2π * sample_range * baseband_frequency / sample_rate) .* fullscale ./ 3,
            zeros(ComplexF64, num_samples)
        )

        # Construct streams
        phase = 0.0
        transmitted_samples = 0
        tx_go = Base.Event()
        c_tx = generate_stream(num_samples, stream_tx.nchannels; T=format) do buff
            if transmitted_samples > num_total_samples
                return false
            end
            copyto!(buff, format.(round.(signals .* cis(phase))))
            phase = mod2pi(2π * num_samples * baseband_frequency / sample_rate + phase)
            transmitted_samples += num_samples
            return true
        end
        t_tx = stream_data(stream_tx, tripwire(c_tx, tx_go))

        # RX reads the buffers in, and pushes them onto `iq_data`
        samples_channel = flowgate(stream_data(stream_rx, num_total_samples; leadin_buffers=0), tx_go)

        reshunked_channel = rechunk(samples_channel, 10000)

#        periodograms = calc_periodograms(reshunked_channel, sampling_freq = upreferred(sample_rate / 1u"Hz"))
#        plot_periodograms(periodograms; fig)

#
        filter_coeffs = digitalfilter(DSP.Filters.ComplexBandpass(
            upreferred((baseband_frequency - 0.1u"MHz") / 1u"Hz"),
            upreferred((baseband_frequency + 0.1u"MHz") / 1u"Hz");
            fs = upreferred(sample_rate / 1u"Hz")),
            FIRWindow(hanning(256), scale = false)
        )
        filtered_stream = streaming_filter(reshunked_channel, filter_coeffs)
#        periodograms = calc_periodograms(filtered_stream, sampling_freq = upreferred(sample_rate / 1u"Hz"))
#        plot_periodograms(periodograms; fig)


        downconverted_signal = self_downconvert(filtered_stream)
#        periodograms = calc_periodograms(downconverted_signal, sampling_freq = upreferred(sample_rate / 1u"Hz"))
#        plot_periodograms(periodograms; fig)

        float_signal = complex2float(x -> angle(x) * 180 / π, downconverted_signal)
        mean_phase_signal = reduce(mean, float_signal)

        reshunked_mean_phase = rechunk(mean_phase_signal, 2000)
 
        mean_phase_data = collect_buffers(reshunked_mean_phase)

#        concat_means_signal = append_vectors(mean_phase_signal)

#        plot_signal(concat_means_signal; fig, ylabel = "Phase mismatch (deg)")

        # Ensure that we're done transmitting as well.
        # This should always be the case, but best to be sure.
        wait(t_tx)
        mean_phase_data
    end
end