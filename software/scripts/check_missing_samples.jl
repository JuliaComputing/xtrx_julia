#ENV["SOAPY_SDR_LOG_LEVEL"] = "DEBUG"

ENV["GKSwstype"] = "100"

using SoapySDR, Printf, Unitful, DSP, LibSigflow, LibSigGUI, Statistics,
    GNSSSignals, Tracking, Acquisition, FFTW, LinearAlgebra
include("./xtrx_debugging.jl")

if Threads.nthreads() < 2
    error("This script must be run with multiple threads!")
end

struct TrackData
    CN0::Float64
    sample_shift::Float64
end

function update_code_phase(
    system::AbstractGNSS,
    num_samples,
    code_frequency,
    sampling_frequency,
    start_code_phase,
)
    code_length = get_code_length(system)
    mod(code_frequency * num_samples / sampling_frequency + start_code_phase, code_length)
end

function correlate!(
    correlation_result,
    signal,
    signal_baseband_freq_domain,
    code_freq_baseband_freq_domain,
    code_baseband,
    fft_plan,
    code_freq_domain,
)
    mul!(signal_baseband_freq_domain, fft_plan, signal)
    code_freq_baseband_freq_domain .= code_freq_domain .* conj.(signal_baseband_freq_domain)
    ldiv!(code_baseband, fft_plan, code_freq_baseband_freq_domain)
    correlation_result .= abs2.(code_baseband)
    correlation_result
end

function correlate_channel(in::MatrixSizedChannel{T}, system, sampling_freq, prn) where {T <: Number}
    spawn_channel_thread(;T = Float64, num_samples = 1, num_antenna_channels = 1) do out
        code = gen_code(in.num_samples, system, prn, sampling_freq)
        signal_baseband_freq_domain = Vector{ComplexF32}(undef, in.num_samples)
        code_baseband = similar(signal_baseband_freq_domain)
        correlation_result = Vector{Float32}(undef, in.num_samples)
        code_freq_baseband_freq_domain = similar(signal_baseband_freq_domain)
        fft_plan = plan_fft(signal_baseband_freq_domain)
        code_freq_domain = fft_plan * code
        consume_channel(in) do signals
#            signalsf32 = ComplexF32.(signals)
#            sample_shifts = map(eachcol(signalsf32)) do signal

                correlation_result = correlate!(
                    correlation_result,
                    ComplexF32.(signals[:,1]),
                    signal_baseband_freq_domain,
                    code_freq_baseband_freq_domain,
                    code_baseband,
                    fft_plan,
                    code_freq_domain,
                )
                signal_noise_power, index = findmax(correlation_result)
                sample_shifts = index - 1
#            end
            push!(out, reshape([sample_shifts], 1, 1))
        end
    end
end

function track_gnss_signal(in::MatrixSizedChannel{T}, system, sampling_freq, prn) where {T <: Number}
    spawn_channel_thread(;T = TrackData, in.num_antenna_channels) do out
        track_states = TrackingState[]
        signal_found = false
        counter = 0
        start_code_phases = zeros(in.num_antenna_channels)
        consume_channel(in) do signals
            if !signal_found
                track_states = map(eachcol(signals)) do signal
                    acq_res = coarse_fine_acquire(
                        system,
                        signal,
                        sampling_freq,
                        prn;
                        max_doppler = 500u"Hz",
                    )
                    if acq_res.CN0 < 45
                        @warn "Signal not found, yet. Searching..."
                    else
                        @info "Signal found"
                        signal_found = true
                    end
                    TrackingState(acq_res)
                end
            end
            if signal_found
                track_results = map(eachcol(signals), track_states) do signal, track_state
                    track(signal, track_state, sampling_freq)
                end
                track_states = get_state.(track_results)
                est_code_phase = mod.(get_code_phase.(track_results), get_code_length(system))
                if counter > 1
                    #push!(out, 10 * log10.(linear.(get_cn0.(track_results)) ./ 1u"Hz"))
                    track_data = TrackData.(
                        10 * log10.(linear.(get_cn0.(track_results)) ./ 1u"Hz"),
                        (est_code_phase - start_code_phases) * sampling_freq / get_code_frequency(system)
                    )
                    push!(out, track_data)
                else
                    counter += 1
                    start_code_phases = est_code_phase
                end
            end
        end
    end
end

function plot_track_data(in::VectorSizedChannel; fig, sample_rate, num_samples_to_track)
    cn0_points_foreach_channel = [Observable(Point2f0.([-1.0, 1.0], [0.0, 0.0])) for _ = 1:in.num_antenna_channels]
    sample_shift_points_foreach_channel = [Observable(Point2f0.([-1.0, 1.0], [0.0, 0.0])) for _ = 1:in.num_antenna_channels]
    ax_offset = length(fig.content)
    axs = map(cn0_points_foreach_channel, sample_shift_points_foreach_channel, 1:in.num_antenna_channels) do cn0_points, sample_shift_points, idx

        ax2 = Axis(fig[idx + ax_offset, 1], yticklabelcolor = :red, yaxisposition = :right, ylabel = "CN0")
        ax1 = Axis(fig[idx + ax_offset, 1], yticklabelcolor = :blue, xlabel = "Time (min)", ylabel = "Sample shifts", title = "Channel $idx")
        hidespines!(ax2)
        hidexdecorations!(ax2)

        lines!(ax2, cn0_points, color = :red)
        lines!(ax1, sample_shift_points, color = :blue)
        return (ax1, ax2)
    end

    consume_channel(in) do track_datas
#        if isopen(fig.scene) # Does not work for WGLMakie
            foreach(track_datas, cn0_points_foreach_channel, sample_shift_points_foreach_channel, axs) do track_data, cn0_points, sample_shift_points, ax
                xs = (0:length(track_data) - 1) * num_samples_to_track / upreferred(sample_rate * 60u"s")
                cn0_points[] = Point2f0.(xs, map(x -> x.CN0, track_data))
                sample_shift_points[] = Point2f0.(xs, map(x -> x.sample_shift, track_data))
                autolimits!(ax[1])
                autolimits!(ax[2])
            end
#        end
    end
end

#=
function eval_missing_samples(;
    frequency = 1565.42u"MHz",
    sample_rate = 4e6u"Hz",
    gnss_system = GPSL1()
#    gain = 60u"dB",
)
    num_samples_to_track = Int(upreferred(sample_rate * 4u"ms"))

    Device(first(Devices())) do dev

        fig, close_stream_event = open_and_display_figure()
        format = dev.rx[1].native_stream_format
        fullscale = dev.tx[1].fullscale

        # Setup transmitter parameters
        ct = dev.tx[1]
        ct.bandwidth = sample_rate
        ct.frequency = frequency
        ct.sample_rate = sample_rate
        ct.gain = 20u"dB"
        ct.gain_mode = false

        # Setup receive parameters
        for cr in dev.rx
            cr.bandwidth = sample_rate
            cr.frequency = frequency
            cr.sample_rate = sample_rate
            # Gain does not seem to have an effect with BladeRF
            # Even if gain_mode is set to false
            cr.gain = 0u"dB"
            cr.gain_mode = false
        end

        sat_prn = 34
        code_frequency = get_code_frequency(gnss_system)

        stream_rx = SoapySDR.Stream(ComplexF32, dev.rx)

        stream_tx = SoapySDR.Stream(format, dev.tx)

        num_samples = stream_tx.mtu
        signals = zeros(num_samples, stream_tx.nchannels)
#        num_total_samples = Int(upreferred(sample_rate * 2000u"ms"))

        # Construct streams
        phase = 0.0
        tx_go = Base.Event()
        transmitted_samples = 0
        c_tx = generate_stream(num_samples, stream_tx.nchannels; T=format) do buff
            if close_stream_event.set
#            if transmitted_samples > num_total_samples
                return false
            end
            signals[:, 1] = gen_code(num_samples, gnss_system, sat_prn, sample_rate, code_frequency, phase) .* fullscale ./ 3
            copyto!(buff, format.(round.(signals)))
            phase = update_code_phase(gnss_system, num_samples, code_frequency, sample_rate, phase)
            transmitted_samples += num_samples
            return true
        end
        t_tx = stream_data(stream_tx, tripwire(c_tx, tx_go))

        # RX reads the buffers in, and pushes them onto `iq_data`
        samples_channel = flowgate(stream_data(stream_rx, close_stream_event; leadin_buffers=0), tx_go)
#        samples_channel = flowgate(stream_data(stream_rx, num_total_samples; leadin_buffers=0), tx_go)

#        iq_data = collect_buffers(samples_channel)

        reshunked_channel = rechunk(samples_channel, num_samples_to_track)

#        periodograms = calc_periodograms(reshunked_channel, sampling_freq = upreferred(sample_rate / 1u"Hz"))
#        plot_periodograms(periodograms; fig)

#        float_signal = complex2float(real, reshunked_channel)
#        plot_signal(float_signal; fig)  

        cn0_stream = track_gnss_signal(reshunked_channel, gnss_system, sample_rate, sat_prn)

        concat_cn0s = append_vectors(cn0_stream)

        plot_track_data(concat_cn0s; fig, sample_rate, num_samples_to_track)


        # Ensure that we're done transmitting as well.
        # This should always be the case, but best to be sure.
        wait(t_tx)
#        iq_data, signals
    end
end
=#
function eval_missing_samples1(;
    frequency = 1565.42u"MHz",
    sample_rate = 5e6u"Hz",
    rx_sample_rate = 2e6u"Hz",
    gnss_system = GPSL1()
#    gain = 60u"dB",
)
    num_samples_to_track = Int(upreferred(rx_sample_rate * 1u"ms"))

    Device(first(Devices())) do dev

#        fig, close_stream_event = open_and_display_figure(use_button = false)
        format = dev.rx[1].native_stream_format
        fullscale = dev.tx[1].fullscale

        # Setup transmitter parameters
        ct = dev.tx[1]
        ct.bandwidth = sample_rate
        ct.frequency = frequency
        ct.sample_rate = sample_rate
        ct.gain = 20u"dB"
        ct.gain_mode = false

        # Setup receive parameters
        for cr in dev.rx
            cr.bandwidth = rx_sample_rate
            cr.frequency = frequency
            cr.sample_rate = rx_sample_rate
            # Gain does not seem to have an effect with BladeRF
            # Even if gain_mode is set to false
            cr.gain = 0u"dB"
            cr.gain_mode = false
            println("Rx bandwidth: ", cr.bandwidth)
            println("Rx sample_rate: ", cr.sample_rate)
        end

        sat_prn = 34
        code_frequency = get_code_frequency(gnss_system)

        stream_rx = SoapySDR.Stream(format, dev.rx)

        stream_tx = SoapySDR.Stream(format, dev.tx)

        num_samples = stream_tx.mtu
        signals = zeros(num_samples, stream_tx.nchannels)
        num_total_samples = Int(upreferred(sample_rate * 20000u"ms"))
        rx_num_total_samples = Int(upreferred(rx_sample_rate * 20000u"ms"))

        # Construct streams
        phase = 0.0
        tx_go = Base.Event()
        transmitted_samples = 0
        c_tx = generate_stream(num_samples, stream_tx.nchannels; T=format) do buff
#            if close_stream_event.set
            if transmitted_samples > num_total_samples
                return false
            end
            signals[:, 1] = gen_code(num_samples, gnss_system, sat_prn, sample_rate, code_frequency, phase) .* fullscale ./ 3
            copyto!(buff, format.(round.(signals)))
            phase = update_code_phase(gnss_system, num_samples, code_frequency, sample_rate, phase)
            transmitted_samples += num_samples
            return true
        end
        t_tx = stream_data(stream_tx, tripwire(c_tx, tx_go))

        # RX reads the buffers in, and pushes them onto `iq_data`
#        samples_channel = flowgate(stream_data(stream_rx, close_stream_event; leadin_buffers=0), tx_go)
        samples_channel = flowgate(stream_data(stream_rx, rx_num_total_samples; leadin_buffers=0), tx_go)

#        iq_data = collect_buffers(samples_channel)

        reshunked_channel = rechunk(samples_channel, num_samples_to_track)

#        periodograms = calc_periodograms(reshunked_channel, sampling_freq = upreferred(rx_sample_rate / 1u"Hz"))
#        plot_periodograms(periodograms; fig)

#        float_signal = complex2float(real, reshunked_channel)
#        plot_signal(float_signal; fig)  

        sample_shift_stream = correlate_channel(reshunked_channel, gnss_system, rx_sample_rate, sat_prn)

#        plot_signal(diffed_samples_shifts; fig, ylabel = "Sample shifts")     
        iq_data = collect_buffers(sample_shift_stream)

        # Ensure that we're done transmitting as well.
        # This should always be the case, but best to be sure.
        wait(t_tx)
        iq_data
    end
end