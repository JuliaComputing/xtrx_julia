# configure loopback mode in the the XTRX and LMS7002M RF IC, so transmitted
# buffers should appear on the RX side.

# Don't let GR segfault
ENV["GKSwstype"] = "100"
#ENV["SOAPY_SDR_LOG_LEVEL"] = "DEBUG"

using SoapySDR, Printf, Unitful, DSP, LibSigflow, FFTW
include("./xtrx_debugging.jl")

if Threads.nthreads() < 2
    error("This script must be run with multiple threads!")
end

# foreign threads can segfault us when they call back into the logger
#SoapySDR.register_log_handler()

function guess_mode(args)
    if "--lfsr-loopback" in args
        return :lfsr_loopback
    elseif "--digital-loopback" in args
        return :digital_loopback
    elseif "--tbb-loopback" in args
        return :tbb_loopback
    elseif "--trf-loopback" in args
        return :trf_loopback
    else
        return :tx
    end
end

function fpga_loopback_sanity_check(dev)
    # check that the clocks are correctly calibrated by having the FPGA
    # transmit a pattern over the digital loopback and verify the result.
    # if this fails, you might need different TX/RX delays.
    SoapySDR.SoapySDRDevice_writeSetting(dev, "LOOPBACK_ENABLE", "TRUE")
    SoapySDR.SoapySDRDevice_writeSetting(dev, "FPGA_TX_PATTERN", "1")
    SoapySDR.SoapySDRDevice_writeSetting(dev, "FPGA_RX_PATTERN", "1")
    try
        e0 = unsafe_string(SoapySDR.SoapySDRDevice_readSetting(dev, "FPGA_RX_PATTERN_ERRORS"))
        sleep(0.1)
        e1 = unsafe_string(SoapySDR.SoapySDRDevice_readSetting(dev, "FPGA_RX_PATTERN_ERRORS"))
        errors = parse(Int, e1) - parse(Int, e0)
        if errors != 0
            @error "FPGA could not verify digital loopback, clock delays may need calibration!"
        end
    finally
        SoapySDR.SoapySDRDevice_writeSetting(dev, "FPGA_TX_PATTERN", "0")
        SoapySDR.SoapySDRDevice_writeSetting(dev, "FPGA_RX_PATTERN", "0")
        SoapySDR.SoapySDRDevice_writeSetting(dev, "LOOPBACK_ENABLE", "FALSE")
    end
end

function do_txrx(mode::Symbol;
                 sample_rate,
                 register_sets::Vector{<:Pair} = Pair[],
                 dump_inis::Bool = false,
                 skip_sanity_check::Bool = false)
    # If we're running on pathfinder, pick a specific device
    device_kwargs = Dict{Symbol,Any}()
    if chomp(String(read(`hostname`))) == "pathfinder"
        device_kwargs[:driver] = "XTRXLime"
        device_kwargs[:serial] = "12cc5241b88485c"
    end

    Device(first(Devices(;device_kwargs...))) do dev
        # Get some useful parameters
        format = dev.rx[1].native_stream_format
        fullscale = dev.tx[1].fullscale

        frequency = 1575.42u"MHz"

        # Setup transmission/recieve parameters
        for (c_idx, cr) in enumerate(dev.rx)
            cr.bandwidth = sample_rate
            cr.sample_rate = sample_rate
            cr.frequency = frequency
            if mode == :tbb_loopback
                # For TBB loopback, we really don't need to be that loud
#                cr[SoapySDR.GainElement(:LNA)] = 0u"dB"
#                cr[SoapySDR.GainElement(:TIA)] = 0u"dB"
#                cr[SoapySDR.GainElement(:PGA)] = 0u"dB"
                cr.gain = 0u"dB"
            elseif mode == :trf_loopback
                # For :trf_loopback, we need to be a little louder
                cr[SoapySDR.GainElement(:LNA)] = 0u"dB"
                cr[SoapySDR.GainElement(:TIA)] = 0u"dB"
                cr[SoapySDR.GainElement(:PGA)] = 19u"dB"

                # We also need to enable the loopback gain
                cr[SoapySDR.GainElement(:LB_LNA)] = 40u"dB"
            elseif mode == :tx
                # For actual transmission, we need to be a little louder still
#                cr[SoapySDR.GainElement(:LNA)] = 10u"dB"
#                cr[SoapySDR.GainElement(:TIA)] = 12u"dB"
#                cr[SoapySDR.GainElement(:PGA)] = 19u"dB"
                cr.gain = 50u"dB"
            else
                # Default everything to absolute quiet
#                cr[SoapySDR.GainElement(:LNA)] = 0u"dB"
#                cr[SoapySDR.GainElement(:TIA)] = 0u"dB"
#                cr[SoapySDR.GainElement(:PGA)] = -20u"dB"
                cr.gain = 40u"dB"
            end

            # Normally, we'll be receiving from LNAL (since we're at 1.5GHz)
            # but if we're doing a TRF loopback, we need to pull from the
            # appropriate loopback path
            if mode != :trf_loopback
                cr.antenna = :LNAL
            else
                cr.antenna = :LB1
            end
        end

        for ct in dev.tx
            ct.bandwidth = sample_rate
            ct.sample_rate = sample_rate
            ct.frequency = frequency
            if mode ==:tx
                # If we're actually TX'ing and RX'ing, juice it up
                ct.gain = 50u"dB"
            elseif mode == :trf_loopback
                ct.gain = 40u"dB"
            else
                # Otherwise, keep quiet
                ct.gain = 30u"dB"
            end
        end

        # Do a quick FPGA loopback sanity check for these clocking values
        if !skip_sanity_check
            fpga_loopback_sanity_check(dev)
        end

        if mode == :digital_loopback
            SoapySDR.SoapySDRDevice_writeSetting(dev, "LOOPBACK_ENABLE", "TRUE")
        elseif mode == :lfsr_loopback
            SoapySDR.SoapySDRDevice_writeSetting(dev, "LOOPBACK_ENABLE_LFSR", "TRUE")
        elseif mode == :tbb_loopback
            # Enable TBB -> RBB loopback
            SoapySDR.SoapySDRDevice_writeSetting(dev, "TBB_ENABLE_LOOPBACK", "LB_MAIN_TBB")

            # Use low bandwidth filters, and tell the RBB to use the loopback
            SoapySDR.SoapySDRDevice_writeSetting(dev, "TBB_SET_PATH", "TBB_LBF")
            SoapySDR.SoapySDRDevice_writeSetting(dev, "RBB_SET_PATH", "LB_LBF")
        elseif mode == :trf_loopback
            # Enable TRF -> RFE loopback
            SoapySDR.SoapySDRDevice_writeSetting(dev, "TRF_ENABLE_LOOPBACK", "TRUE")

            # Use low bandwidth filters
            SoapySDR.SoapySDRDevice_writeSetting(dev, "TBB_SET_PATH", "TBB_LBF")
            SoapySDR.SoapySDRDevice_writeSetting(dev, "RBB_SET_PATH", "LBF")
        end

        if !isempty(register_sets)
            @info("Applying $(length(register_sets)) register sets")
            for (addr, val) in register_sets
                if val !== nothing
                    write_lms_register(dev, addr, val)
                end
                @info(string("0x", string(addr; base=16)), value=read_lms_register(dev, addr))
            end
        end

        for ct in dev.tx
            ct[SoapySDR.Setting("CALIBRATE")] = "5e6"
        end

        for (c_idx, cr) in enumerate(dev.rx)
            cr[SoapySDR.Setting("CALIBRATE")] = "5e6"
        end

        # Dump an initial INI, showing how the registers are configured here
        if dump_inis
            SoapySDR.SoapySDRDevice_writeSetting(dev, "DUMP_INI", "$(mode).ini")
        end

        # Construct streams
        stream_rx = SoapySDR.Stream(format, dev.rx)
        stream_tx = SoapySDR.Stream(format, dev.tx)

        # We're going to write and read this many buffers:
        if mode == :lfsr_loopback
            # If we're dealing with the LFSR loopback, don't get too many buffers
            # as it takes a long time to plot randomness
            num_buffers = 4
        else
            num_buffers = 12
        end

        # prepare some data to send:
        signal_frequency = 0.75u"MHz"
        num_repeats = 1
        num_channels = Int(length(dev.tx))
        mtu = Int(stream_tx.mtu)
        samples = div(mtu*num_buffers, num_repeats)
        data_tx = zeros(format, samples, num_channels)
        sample_range = 0:samples - 1

        # Create some pretty patterns to plot
        data_tx[:, 1] .= Complex{Int16}.(round.(cis.(2π .* sample_range .* signal_frequency ./ sample_rate) .* (fullscale/3)))
        data_tx[:, 2] .= Complex{Int16}.(round.(cis.(2π .* sample_range .* signal_frequency ./ sample_rate) .* (fullscale/3)))

        # Simple flowgraph for TX: just transmit the same buffer over and over again
        tx_go = Base.Event()
        num_buffs_transmitted = 0
        c_tx = generate_stream(samples, stream_tx.nchannels; T=format) do buff
            if num_buffs_transmitted >= num_repeats
                return false
            end
            copyto!(buff, data_tx)
            num_buffs_transmitted += 1
            return true
        end
        c_tx = rechunk(c_tx, stream_tx.mtu)
        t_tx = stream_data(stream_tx, tripwire(c_tx, tx_go))

        # RX reads the buffers in, and pushes them onto `iq_data`
        c_rx = flowgate(stream_data(stream_rx, mtu*num_buffers; leadin_buffers=0), tx_go)
        iq_data = collect_buffers(c_rx)

        # Ensure that we're done transmitting as well.
        # This should always be the case, but best to be sure.
        wait(t_tx)

        dma_buffers = dev[SoapySDR.Setting("DMA_BUFFERS")]
        sleep(0.1)
        println(dma_buffers)

        return iq_data, data_tx
    end
end


using Plots

# Plot out received signals
function make_txrx_plots(iq_data, data_tx; name::String="data", sample_rate)
#    data_part = 15000:size(iq_data, 1)#(15000:15100) .+ 120000
#    data_part = 6001:6100
#    data_part = (20000:min(20000+15000, size(data_tx,1)))# .+ 120000
    data_part = (20550:20600) .+ 4000
    plt = plot(real.(data_tx[data_part, 1]); label="re(tx[1])", title="$(name) - Real")
    plot!(plt, real.(iq_data)[data_part, 1]; label="re(rx[1])")
    plot!(plt, real.(iq_data)[data_part, 2]; label="re(rx[2])")
    savefig(plt, "plots/$(name)_re.png")

    plt = plot(imag.(data_tx[data_part, 1]); label="im(tx[1])", title="$(name) - Imag")
    plot!(plt, imag.(iq_data)[data_part, 1]; label="im(rx[1])")
    plot!(plt, imag.(iq_data)[data_part, 2]; label="im(rx[2])")
    savefig(plt, "plots/$(name)_im.png")

    fft_data_part = (20000:min(20000+15000, size(data_tx,1)))# .+ 120000

    p = periodogram(data_tx[fft_data_part, 1], fs = upreferred(sample_rate / 1u"Hz"))
    plt = plot(fftshift(freq(p) / 1e6), fftshift(10 * log10.(power(p))); xlabel="Frequency (MHz)", ylabel = "Power (dB)", title="Periodogram $name transmitted")
    savefig(plt, "plots/periodogram_$(name)_transmitted.png")
    p = periodogram(iq_data[fft_data_part, 1], fs = upreferred(sample_rate / 1u"Hz"))
    plt = plot(fftshift(freq(p) / 1e6), fftshift(10 * log10.(power(p))); xlabel="Frequency (MHz)", ylabel = "Power (dB)", title="Periodogram $name channel 1")
    savefig(plt, "plots/periodogram_$(name)_channel_1.png")
    p = periodogram(iq_data[fft_data_part, 2], fs = upreferred(sample_rate / 1u"Hz"))
    plt = plot(fftshift(freq(p) / 1e6), fftshift(10 * log10.(power(p))); xlabel="Frequency (MHz)", ylabel = "Power (dB)", title="Periodogram $name channel 2")
    savefig(plt, "plots/periodogram_$(name)_channel_2.png")
end

function full_loopback_suite(;sample_rate, kwargs...)
    @sync begin
        # First, lfsr loopback
        lfsr_iq, lfsr_tx = do_txrx(:lfsr_loopback; sample_rate, kwargs...)
        t_lfsr_plot = @async make_txrx_plots(lfsr_iq, lfsr_tx; sample_rate, name="lfsr_loopback")

        # Next, digital loopback
        digi_iq, digi_tx = do_txrx(:digital_loopback; sample_rate, kwargs...)
        wait(t_lfsr_plot)
        t_digi_plot = @async make_txrx_plots(digi_iq, digi_tx; sample_rate, name="digital_loopback")

        # Next, TBB loopback
        tbb_iq, tbb_tx = do_txrx(:tbb_loopback; sample_rate, kwargs...)
        wait(t_digi_plot)
        t_tbb_plot = @async make_txrx_plots(tbb_iq, tbb_tx; sample_rate, name="tbb_loopback")

        # Next, TRF loopback
        trf_iq, trf_tx = do_txrx(:trf_loopback; sample_rate, kwargs...)
        wait(t_tbb_plot)
        t_trf_plot = @async make_txrx_plots(trf_iq, trf_tx; sample_rate, name="trf_loopback")

        # Finally, out over the air
        tx_iq, tx_tx = do_txrx(:tx; sample_rate, kwargs...)
        wait(t_trf_plot)
        t_tx_plot = @async make_txrx_plots(tx_iq, tx_tx; sample_rate, name="tx")
    end
end


function main(args::String...)
    mode = guess_mode(args)
    dump_inis = "--dump-inis" in args
    full_suite = "--full" in args
    skip_sanity_check = "--no-sanity-check" in args
    sample_rate = 5u"MHz"

    # You can set this here, but Elliot has changed XTRXDevice.cpp to do this automatically.
    register_sets = Pair[
        #0x00ad => 0x03f3,

        # Force CG_IAMP_TBB to be smaller, to prevent over saturating
        # Note that `0x45xx` is still large enough to saturate, but setting the IAMP
        # lower causes a bunch of noise to leak in for reasons I still don't fully understand.
        # This is probably related to the fact that most transmitters prefer to saturate.
        #0x0108 => 0x558c,
    ]

    if full_suite
        full_loopback_suite(; dump_inis, register_sets, skip_sanity_check, sample_rate)
    else
        iq_data, data_tx = do_txrx(mode; dump_inis, register_sets, skip_sanity_check, sample_rate)
        make_txrx_plots(iq_data, data_tx; name="$(mode)", sample_rate)
    end
end

isinteractive() || main(ARGS...)
