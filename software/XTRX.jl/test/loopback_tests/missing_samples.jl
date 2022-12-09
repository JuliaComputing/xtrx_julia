module MissingSamplesTest

# Don't let GR segfault
ENV["GKSwstype"] = "100"

# Force very verbose driver
#ENV["SOAPY_SDR_LOG_LEVEL"]="DEBUG"

using SoapySDR, Unitful, LibSigflow, Statistics, GNSSSignals, FFTW, LinearAlgebra, XTRX, Printf, Test, Plots

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
    spawn_channel_thread(;T = Float64, num_samples = 1, in.num_antenna_channels) do out
        code = gen_code(in.num_samples, system, prn, sampling_freq)
        signal_baseband_freq_domain = Vector{ComplexF32}(undef, in.num_samples)
        code_baseband = similar(signal_baseband_freq_domain)
        correlation_result = Vector{Float32}(undef, in.num_samples)
        code_freq_baseband_freq_domain = similar(signal_baseband_freq_domain)
        fft_plan = plan_fft(signal_baseband_freq_domain)
        code_freq_domain = fft_plan * code
        consume_channel(in) do signals
            signalsf32 = ComplexF32.(signals)
            sample_shifts = map(eachcol(signalsf32)) do signal
                correlation_result = correlate!(
                    correlation_result,
                    signal,
                    signal_baseband_freq_domain,
                    code_freq_baseband_freq_domain,
                    code_baseband,
                    fft_plan,
                    code_freq_domain,
                )
                signal_noise_power, index = findmax(correlation_result)
                index - 1
            end
            push!(out, reshape(sample_shifts, 1, in.num_antenna_channels))
        end
    end
end

function eval_missing_samples(;
    frequency = 1565.42u"MHz",
    samplerate = 5e6u"Hz",
    run_time = 10u"s",
    gnss_system = GPSL1(),
    count_dma_buffers = false,
    loopback_mode = XTRX.NoLoopback,
    out_dir::String="missing_samples"
)
    num_samples_to_track = Int(upreferred(samplerate * 1u"ms"))

    device_kwargs = Dict{Symbol,Any}()
    if chomp(String(read(`hostname`))) == "pathfinder"
        device_kwargs[:driver] = "XTRX"
        device_kwargs[:serial] = "12cc5241b88485c"
    end

    config = XTRXTestConfig(;
        device = device_kwargs,
        loopback_mode,
        rx_params = XTRX.default_rx_parameters(;samplerate, frequency),
        tx_params = XTRX.default_tx_parameters(;samplerate, frequency),
    )

    dma_buffers = (
        rx_hw_count = Int[],
        rx_sw_count = Int[],
        rx_user_count = Int[],
        tx_hw_count = Int[],
        tx_sw_count = Int[],
        tx_user_count = Int[],
    )

    # We'll store the received data here
    local exemplary_measurement, missing_samples_data
    # Try 10 times to get results without underflows / overflows
    max_tries = 10
    counter = 0
    for i in 1:max_tries
#        run(`./test/reset.sh -r`)
        xflow_stats = XTRX.run_test(config) do dev, s_tx, s_rx

            format = first(dev.rx).native_stream_format
            fullscale = first(dev.tx).fullscale

            sat_prn = 34
            code_frequency = get_code_frequency(gnss_system)

            num_samples = s_tx.mtu
            signals = zeros(num_samples, s_tx.nchannels)
            num_total_samples = Int(upreferred(samplerate * run_time))

            # Construct streams
            phase = 0.0
            tx_go = Base.Event()
            transmitted_samples = 0
            c_tx = generate_stream(num_samples, s_tx.nchannels; T=format) do buff
                if transmitted_samples > num_total_samples
                    return false
                end
                code = gen_code(num_samples, gnss_system, sat_prn, samplerate, code_frequency, phase) .* fullscale ./ 3
                signals[:, 1] = code
                signals[:, 2] = code
                copyto!(buff, format.(round.(signals)))
                phase = update_code_phase(gnss_system, num_samples, code_frequency, samplerate, phase)
                transmitted_samples += num_samples
                if count_dma_buffers
                    push!(dma_buffers.rx_hw_count, parse(Int, dev[SoapySDR.Setting("DMA_BUFFER_RX_HW_COUNT")]))
                    push!(dma_buffers.rx_sw_count, parse(Int, dev[SoapySDR.Setting("DMA_BUFFER_RX_SW_COUNT")]))
                    push!(dma_buffers.rx_user_count, parse(Int, dev[SoapySDR.Setting("DMA_BUFFER_RX_USER_COUNT")]))
                    push!(dma_buffers.tx_hw_count, parse(Int, dev[SoapySDR.Setting("DMA_BUFFER_TX_HW_COUNT")]))
                    push!(dma_buffers.tx_sw_count, parse(Int, dev[SoapySDR.Setting("DMA_BUFFER_TX_SW_COUNT")]))
                    push!(dma_buffers.tx_user_count, parse(Int, dev[SoapySDR.Setting("DMA_BUFFER_TX_USER_COUNT")]))
                end
                return true
            end
            t_tx = stream_data(s_tx, tripwire(c_tx, tx_go))

            samples_channel = flowgate(stream_data(s_rx, num_total_samples; leadin_buffers=0), tx_go)

            reshunked_channel = rechunk(samples_channel, num_samples_to_track)

            data_channel1, data_channel2 = tee(reshunked_channel)

            exemplary_measurement = collect_single_chunk_at(data_channel1, counter_threshold = 1000) # After 1000 ms

            sample_shift_stream = correlate_channel(data_channel2, gnss_system, samplerate, sat_prn)

            reshunked_sample_shifts = rechunk(sample_shift_stream, 2000)
    
            missing_samples_data = collect_buffers(reshunked_sample_shifts)
            wait(t_tx)
        end

        xflow_stats["overflows"] == 0 && xflow_stats["underflows"] == 0 && break
        counter += 1
    end

    failed_max_tries = counter == max_tries
    failed_samples_shifts = any(missing_samples_data[10:end,:] .!= missing_samples_data[end])
    failed = failed_max_tries || failed_samples_shifts

    filename = string(
        out_dir, "/",
        failed ? "FAIL_" : "",
        "missing_samples_",
        @sprintf("s%.1fMHz_", Float64(uconvert(u"MHz", samplerate).val)),
        @sprintf("f%.3fGHz_", Float64(uconvert(u"GHz", frequency).val)),
        loopback_mode,
    )

    data_range = 1000:1200
    p = plot(hcat(real.(exemplary_measurement[data_range, :]), imag.(exemplary_measurement[data_range, :]));
        ylabel = "Amplitude",
        xlabel = "Samples",
        label = ["Real A" "Real B" "Imag A" "Imag B"],
        title= "Exemplary measurement"
    )
    savefig(p, "$(filename)-exemplary_measurement.png")
    p = plot(missing_samples_data;
        ylabel = "Sample shifts",
        xlabel = "Time (ms)",
        label = ["Channel A" "Channel B"],
        title= "Sample Shifts"
    )
    savefig(p, "$(filename)-sample_shifts.png")

    if failed_max_tries
        @error("Buffer overflowed or underflowed too many times",
            max_tries,
            xflow_stats["overflows"],
            xflow_stats["underflows"]
        )
    end

    if failed_samples_shifts
        @error("There are sample shifts in your data",
            counter,
        )
    end

    return !failed
end

function collect_single_chunk_at(in::MatrixSizedChannel{T}; counter_threshold::Int = 1000) where {T <: Number}
    buffs = Matrix{T}(undef, in.num_samples, in.num_antenna_channels)
    counter = 0
    Base.errormonitor(Threads.@spawn begin 
        consume_channel(in) do buff
            if counter == counter_threshold
                buffs .= buff
            end
            counter += 1
        end
    end)
    return buffs
end

function run_tests()
    @testset "Sample shifts" begin
        out_dir = "samples_shifts"
        rm(out_dir; force=true, recursive=true)
        mkpath(out_dir)
        for loopback_mode in (XTRX.NoLoopback,)#(XTRX.DigitalLoopback, XTRX.TBBLoopback, XTRX.TRFLoopback)
            @testset "$(string(loopback_mode))" begin
                for frequency in (1u"GHz",),
                    samplerate in (5e6u"Hz",)# 4u"MHz", 10u"MHz", 20u"MHz")
                    # Run the test, failures will result in a picture upload
                    @test eval_missing_samples(;frequency, samplerate, loopback_mode, out_dir)
                end
            end
        end
    end
end

end # module

isinteractive() || MissingSamplesTest.run_tests()