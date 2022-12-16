#ENV["SOAPY_SDR_LOG_LEVEL"] = "DEBUG"

ENV["GKSwstype"] = "100"

using SoapySDR, Printf, Unitful, DSP, LibSigflow, Statistics,
    GNSSSignals, FFTW, LinearAlgebra, XTRX
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
    sample_rate = 5e6u"Hz",
    run_time = 10u"s",
    gnss_system = GPSL1(),
    count_dma_buffers = false
)
    num_samples_to_track = Int(upreferred(sample_rate * 1u"ms"))

    device_kwargs = Dict{Symbol,Any}()
    if chomp(String(read(`hostname`))) == "pathfinder"
        device_kwargs[:driver] = "XTRX"
        device_kwargs[:serial] = "12cc5241b88485c"
    end
    device_kwargs[:driver] = "XTRXLime"

    Device(first(Devices(;device_kwargs...))) do dev

        format = dev.rx[1].native_stream_format
        fullscale = dev.tx[1].fullscale

        # Setup transmitter parameters
        ct = dev.tx[1]
        ct.bandwidth = sample_rate
        ct.frequency = frequency
        ct.sample_rate = sample_rate
        ct.gain = 50u"dB"
        ct.gain_mode = false

        # Setup receive parameters
        for cr in dev.rx
            cr.bandwidth = sample_rate
            cr.frequency = frequency
            cr.sample_rate = sample_rate
            cr.gain = 80u"dB"
            cr.gain_mode = false
        end

        dma_buffers = (
            rx_hw_count = Int[],
            rx_sw_count = Int[],
            rx_user_count = Int[],
            tx_hw_count = Int[],
            tx_sw_count = Int[],
            tx_user_count = Int[],
        )

        sat_prn = 34
        code_frequency = get_code_frequency(gnss_system)

        stream_rx = SoapySDR.Stream(format, dev.rx)

        stream_tx = SoapySDR.Stream(format, dev.tx)

        num_samples = stream_tx.mtu
        signals = zeros(num_samples, stream_tx.nchannels)
        num_total_samples = Int(upreferred(sample_rate * run_time))

        # Construct streams
        phase = 0.0
        tx_go = Base.Event()
        transmitted_samples = 0
        c_tx = generate_stream(num_samples, stream_tx.nchannels; T=format) do buff
            if transmitted_samples > num_total_samples
                return false
            end
            code = gen_code(num_samples, gnss_system, sat_prn, sample_rate, code_frequency, phase) .* fullscale ./ 3
#            signals[:, 1] = code
            signals[:, 2] = code
            copyto!(buff, format.(round.(signals)))
            phase = update_code_phase(gnss_system, num_samples, code_frequency, sample_rate, phase)
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
        t_tx = stream_data(stream_tx, tripwire(c_tx, tx_go))

        # RX reads the buffers in, and pushes them onto `iq_data`
        samples_channel = flowgate(stream_data(stream_rx, num_total_samples; leadin_buffers=0), tx_go)

        reshunked_channel = rechunk(samples_channel, num_samples_to_track)

        data_channel1, data_channel2 = tee(reshunked_channel)

        measurement = collect_single_chunk_at(data_channel1, counter_threshold = 1000) # After 1000 ms

        sample_shift_stream = correlate_channel(data_channel2, gnss_system, sample_rate, sat_prn)

        reshunked_sample_shifts = rechunk(sample_shift_stream, 2000)
 
        missing_samples_data = collect_buffers(reshunked_sample_shifts)

        # Ensure that we're done transmitting as well.
        # This should always be the case, but best to be sure.
        wait(t_tx)
#        missing_samples_data, dma_buffers
        measurement, missing_samples_data, dma_buffers
    end
end

function eval_missing_samples_over_multiple_devices(;
    frequency = 1565.42u"MHz",
    sample_rate = 5e6u"Hz",
    run_time = 10u"s",
    gnss_system = GPSL1(),
    count_dma_buffers = false
)
    num_samples_to_track = Int(upreferred(sample_rate * 1u"ms"))

    # Do not run Devices twice
    devs = collect(Devices())
    dev1 = Device(only(filter(x -> x["driver"] == "XTRXLime" && x["serial"] == "4c1c440ea8c85c", devs)))
    dev2 = Device(only(filter(x -> x["driver"] == "XTRXLime" && x["serial"] == "12cc5241b88485c", devs)))

    try
        synchro_dev1 = dev1[(SoapySDR.Register("LitePCI"),XTRX.CSRs.CSR_SYNCHRO_CONTROL_ADDR)] 
        synchro_dev2 = dev2[(SoapySDR.Register("LitePCI"),XTRX.CSRs.CSR_SYNCHRO_CONTROL_ADDR)] 
        @show synchro_dev1, synchro_dev2
        dev1[(SoapySDR.Register("LitePCI"),XTRX.CSRs.CSR_SYNCHRO_CONTROL_ADDR)] = synchro_dev1 | (2 << XTRX.CSRs.CSR_SYNCHRO_CONTROL_INT_SOURCE_OFFSET | 2 << XTRX.CSRs.CSR_SYNCHRO_CONTROL_OUT_SOURCE_OFFSET)
        dev2[(SoapySDR.Register("LitePCI"),XTRX.CSRs.CSR_SYNCHRO_CONTROL_ADDR)] = synchro_dev2 | (2 << XTRX.CSRs.CSR_SYNCHRO_CONTROL_INT_SOURCE_OFFSET | 2 << XTRX.CSRs.CSR_SYNCHRO_CONTROL_OUT_SOURCE_OFFSET)

        synchro_dev1 = dev1[(SoapySDR.Register("LitePCI"),XTRX.CSRs.CSR_SYNCHRO_CONTROL_ADDR)] 
        synchro_dev2 = dev2[(SoapySDR.Register("LitePCI"),XTRX.CSRs.CSR_SYNCHRO_CONTROL_ADDR)] 
        @show synchro_dev1, synchro_dev2

        format = dev1.rx[1].native_stream_format
        fullscale = dev1.tx[1].fullscale
        dev1.clock_source = "external+pps"
        dev2.clock_source = "external+pps"

        # Setup transmitter parameters
        ct = dev1.tx[1]
        ct.bandwidth = sample_rate
        ct.frequency = frequency
        ct.sample_rate = sample_rate
        ct.gain = 50u"dB"
        ct.gain_mode = false

        # Needed for now
        ct = dev2.tx[1]
        ct.bandwidth = sample_rate
        ct.sample_rate = sample_rate

        # Setup receive parameters
        for cr in dev1.rx
            cr.bandwidth = sample_rate
            cr.frequency = frequency
            cr.sample_rate = sample_rate
            cr.gain = 50u"dB"
            cr.gain_mode = false
        end

        for cr in dev2.rx
            cr.bandwidth = sample_rate
            cr.frequency = frequency
            cr.sample_rate = sample_rate
            cr.gain = 50u"dB"
            cr.gain_mode = false
        end

        stream_tx = SoapySDR.Stream(format, dev1.tx)
        stream_rx1 = SoapySDR.Stream(format, dev1.rx)
        stream_rx2 = SoapySDR.Stream(format, dev2.rx)

        dma_buffers = (
            rx_hw_count = Int[],
            rx_sw_count = Int[],
            rx_user_count = Int[],
            tx_hw_count = Int[],
            tx_sw_count = Int[],
            tx_user_count = Int[],
        )

        sat_prn = 34
        code_frequency = get_code_frequency(gnss_system)

        num_samples = stream_tx.mtu
        signals = zeros(num_samples, stream_tx.nchannels)
        num_total_samples = Int(upreferred(sample_rate * run_time))

        # Construct streams
        phase = 0.0
        tx_go = Base.Event()
        transmitted_samples = 0
        c_tx = generate_stream(num_samples, stream_tx.nchannels; T=format) do buff
            if transmitted_samples > num_total_samples
                return false
            end
            code = gen_code(num_samples, gnss_system, sat_prn, sample_rate, code_frequency, phase) .* fullscale ./ 3
            signals[:, 1] = code
#            signals[:, 2] = code
            copyto!(buff, format.(round.(signals)))
            phase = update_code_phase(gnss_system, num_samples, code_frequency, sample_rate, phase)
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
        t_tx = stream_data(stream_tx, tripwire(c_tx, tx_go))

        # RX reads the buffers in, and pushes them onto `iq_data`
        samples_channel = flowgate(stream_data(stream_rx1, stream_rx2, num_total_samples; leadin_buffers=0), tx_go)

        reshunked_channel = rechunk(samples_channel, num_samples_to_track)

        data_channel1, data_channel2 = tee(reshunked_channel)

        measurement = collect_single_chunk_at(data_channel1, counter_threshold = 1000) # After 1000 ms

        sample_shift_stream = correlate_channel(data_channel2, gnss_system, sample_rate, sat_prn)

        reshunked_sample_shifts = rechunk(sample_shift_stream, 2000)
 
        missing_samples_data = collect_buffers(reshunked_sample_shifts)

        # Ensure that we're done transmitting as well.
        # This should always be the case, but best to be sure.
        wait(t_tx)
#        missing_samples_data, dma_buffers
        measurement, missing_samples_data, dma_buffers
    finally
        finalize(dev1)
        finalize(dev2)
    end
end