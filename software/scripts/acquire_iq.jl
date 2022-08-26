# configure loopback mode in the the XTRX and LMS7002M RF IC, so transmitted
# buffers should appear on the RX side.

# Don't let GR segfault
ENV["GKSwstype"] = "100"
ENV["SOAPY_SDR_LOG_LEVEL"] = "DEBUG"

using SoapySDR, Printf, Unitful, DSP
include("./libsigflow.jl")

# foreign threads can segfault us when they call back into the logger
#SoapySDR.register_log_handler()


function do_txrx(; digital_loopback::Bool = false,
                   lfsr_loopback::Bool = false,
                   dump_inis::Bool = false,
                   tbb_loopback::Bool = false,
                   trf_loopback::Bool = false,
                   name::String = "acquire_iq")
    # open the first device
    Device(first(Devices())) do dev
        # Get some useful parameters
        format = dev.rx[1].native_stream_format
        fullscale = dev.tx[1].fullscale

        # Setup transmission/recieve parameters
        for (c_idx, cr) in enumerate(dev.rx)
            cr.bandwidth = 20u"MHz"
            cr.frequency = 2.498u"GHz"
            cr.sample_rate = 40u"MHz"

            # Default everything to absolute quiet
            cr[SoapySDR.GainElement(:LNA)] = 0u"dB"
            cr[SoapySDR.GainElement(:TIA)] = 0u"dB"
            cr[SoapySDR.GainElement(:PGA)] = 0u"dB"

            if trf_loopback
                # If we've got a TRF loopback, let's be a little louder
                cr[SoapySDR.GainElement(:TIA)] = 9u"dB"
                cr[SoapySDR.GainElement(:PGA)] = 6u"dB"

                # Despite the `0dB` here, this is actually the loudest it gets
                # Default value is -40dB.
                cr[SoapySDR.GainElement(:LB_LNA)] = 0u"dB"
            else
                # If we're not doing one of those analog loopbacks, we can
                # just default to something reasonable
                cr[SoapySDR.GainElement(:PGA)] = 6u"dB"
            end

            # Normally, we'll be receiving from LNAH (since we're at 2.5GHz)
            # but if we're doing a TRF loopback, we need to pull from the
            # appropriate loopback path
            if !trf_loopback
                cr.antenna = :LNAH
            else
                cr.antenna = Symbol("LB$(c_idx)")
            end
        end

        for ct in dev.tx
            ct.bandwidth = 20u"MHz"
            ct.frequency = 2.498u"GHz"
            ct.sample_rate = 40u"MHz"
            if tbb_loopback
                ct.gain = 0u"dB"
            elseif trf_loopback
                ct.gain = 10u"dB"
            else
                ct.gain = 30u"dB"
            end
        end

        if digital_loopback
            SoapySDR.SoapySDRDevice_writeSetting(dev, "LOOPBACK_ENABLE", "TRUE")
        end
        if lfsr_loopback
            SoapySDR.SoapySDRDevice_writeSetting(dev, "LOOPBACK_ENABLE_LFSR", "TRUE")
        end
        if tbb_loopback
            # Enable TBB -> RBB loopback
            SoapySDR.SoapySDRDevice_writeSetting(dev, "TBB_ENABLE_LOOPBACK", "LB_MAIN_TBB")

            # Disable RxBB and TxBB lowpass filters
            SoapySDR.SoapySDRDevice_writeSetting(dev, "TBB_SET_PATH", "TBB_HBF")
            SoapySDR.SoapySDRDevice_writeSetting(dev, "RBB_SET_PATH", "LB_BYP")

            # Disable RxTSP and TxTSP settings, to cause as little signal disturbance as possible
            SoapySDR.SoapySDRDevice_writeSetting(dev, "RXTSP_ENABLE", "TRUE")
            SoapySDR.SoapySDRDevice_writeSetting(dev, "TXTSP_ENABLE", "TRUE")
        end
        if trf_loopback
            # Enable TRF -> RFE loopback
            SoapySDR.SoapySDRDevice_writeSetting(dev, "TRF_ENABLE_LOOPBACK", "TRUE")
        end


        # Dump an initial INI, showing how the registers are configured here
        if dump_inis
            SoapySDR.SoapySDRDevice_writeSetting(dev, "DUMP_INI", "$(name)_configured.ini")
        end

        # Construct streams
        stream_rx = SoapySDR.Stream(format, dev.rx)
        stream_tx = SoapySDR.Stream(format, dev.tx)

        # the number of buffers each stream has
        wr_nbufs = max(Int(SoapySDR.SoapySDRDevice_getNumDirectAccessBuffers(dev, stream_tx)), 256)
        rd_nbufs = max(Int(SoapySDR.SoapySDRDevice_getNumDirectAccessBuffers(dev, stream_rx)), 256)

        # Let's drop a few of the first buffers, to give our automated
        # balancing algorithms a chance to catch up
        drop_nbufs = 0
        #if !lfsr_loopback
            #drop_nbufs = rd_nbufs
        #end

        # Read 4x as many buffers as we write
        if !lfsr_loopback
            rd_nbufs *= 4
        else
            # If we're dealing with the LFSR loopback, don't get too many buffers
            # as it takes a long time to plot randomness, and don't bother to write anything
            wr_nbufs = 0
            rd_nbufs = 4
        end

        # prepare some data to send:
        rate = 10
        num_channels = Int(length(dev.tx))
        mtu = Int(stream_tx.mtu)
        samples = mtu*wr_nbufs
        t = (1:samples)./samples
        data_tx = zeros(format, samples, num_channels)

        # Create some pretty patterns to plot
        data_tx[:, 1] .= format.(round.(sin.(2π.*t.*rate).*(fullscale/2).*0.95.*DSP.hanning(samples)), 0)

        # We're going to push values onto this list,
        # then concatenate them into a giant matrix at the end
        iq_data = Matrix{format}[]

        SoapySDR.activate!(stream_tx) do; SoapySDR.activate!(stream_rx) do;
            written_buffs = 0
            read_buffs = 0

            # write tx-buffer
            t_write = Threads.@spawn begin
                while written_buffs < wr_nbufs
                    #=
                    buffs = Ptr{format}[C_NULL]
                    err, handle = SoapySDR.SoapySDRDevice_acquireWriteBuffer(dev, stream_tx, buffs, 0)
                    if err == SoapySDR.SOAPY_SDR_TIMEOUT
                        break
                    elseif err == SoapySDR.SOAPY_SDR_UNDERFLOW
                        err = 1 # keep going
                    end
                    @assert err > 0
                    unsafe_copyto!(buffs[1], pointer(data_tx, num_channels*mtu*written_buffs+1), num_channels*mtu)
                    SoapySDR.SoapySDRDevice_releaseWriteBuffer(dev, stream_tx, handle, 1)
                    written_buffs += 1
                    =#
                    buff1 = data_tx[mtu*written_buffs+1:mtu*(written_buffs+1), 1]
                    buff2 = data_tx[mtu*written_buffs+1:mtu*(written_buffs+1), 2]
                    SoapySDR.write(stream_tx, (buff1, buff2); timeout=1u"s")
                    written_buffs += 1
                end
            end

            # Take the opportunity to dump our .ini
            if dump_inis
                SoapySDR.SoapySDRDevice_writeSetting(dev, "DUMP_INI", "acquire_iq_mid_transmission.ini")
            end

            # read/check rx-buffer
            t_read = Threads.@spawn begin
                while read_buffs < (rd_nbufs + drop_nbufs)
                    buff = Matrix{format}(undef, mtu, num_channels)
                    SoapySDR.read!(stream_rx, split_matrix(buff))
                    #=
                    buffs = Ptr{format}[C_NULL]
                    err, handle, flags, timeNs = SoapySDR.SoapySDRDevice_acquireReadBuffer(dev, stream_rx, buffs, 0)

                    if err == SoapySDR.SOAPY_SDR_TIMEOUT
                        continue
                    elseif err == SoapySDR.SOAPY_SDR_OVERFLOW
                        err = mtu # nothing to do, should be the MTU
                    end
                    @assert err > 0
                    =#

                    if (read_buffs > drop_nbufs)
                        #@show buff[400:410, 1]
                        push!(iq_data, buff)
                    end

                    #SoapySDR.SoapySDRDevice_releaseReadBuffer(dev, stream_rx, handle)
                    read_buffs += 1
                end
            end

            wait(t_read)
            wait(t_write)
            sleep(0.1)
        end; end

        # Concatenate into a giant matrix, then sign-extend since it's
        # most like 12-bit signed data hiding in a 16-bit buffer:
        iq_data = cat(iq_data...; dims=1)
        if fullscale == 4096
            sign_extend!(iq_data)
        end

        return iq_data, data_tx
    end
end


using Plots

# Plot out received signals
function make_txrx_plots(iq_data, data_tx; name::String="data")
    plt = plot(real.(data_tx[:, 1]); label="re(tx[1])", title="$(name) - Real")
    plot!(plt, real.(iq_data)[:, 1]; label="re(rx[1])")
    plot!(plt, real.(iq_data)[:, 2]; label="re(rx[2])")
    savefig(plt, "$(name)_re.png")

    plt = plot(imag.(data_tx[:, 1]); label="im(tx[1])", title="$(name) - Imag")
    plot!(plt, imag.(iq_data)[:, 1]; label="im(rx[1])")
    plot!(plt, imag.(iq_data)[:, 2]; label="im(rx[2])")
    savefig(plt, "$(name)_im.png")
end

function full_loopback_suite(;kwargs...)
    @sync begin
        # First, lfsr loopback
        lfsr_iq, lfsr_tx = do_txrx(; lfsr_loopback=true, name="lfsr_loopback", kwargs...)
        t_lfsr_plot = @async make_txrx_plots(lfsr_iq, lfsr_tx; name="lfsr_loopback")

        # Next, digital loopback
        digi_iq, digi_tx = do_txrx(; digital_loopback=true, name="digital_loopback", kwargs...)
        wait(t_lfsr_plot)
        t_digi_plot = @async make_txrx_plots(digi_iq, digi_tx; name="digital_loopback")

        # Next, TBB loopback
        tbb_iq, tbb_tx = do_txrx(; tbb_loopback=true, name="tbb_loopback", kwargs...)
        wait(t_digi_plot)
        t_tbb_plot = @async make_txrx_plots(tbb_iq, tbb_tx; name="tbb_loopback")

        # Finally, TRF loopback
        trf_iq, trf_tx = do_txrx(; trf_loopback=true, name="trf_loopback", kwargs...)
        wait(t_tbb_plot)
        t_trf_plot = @async make_txrx_plots(trf_iq, trf_tx; name="trf_loopback")
    end
end


function main(args...)
    lfsr_loopback = "--lfsr-loopback" in args
    digital_loopback = "--digital-loopback" in args
    tbb_loopback = "--tbb-loopback" in args
    trf_loopback = "--trf-loopback" in args
    dump_inis = "--dump-inis" in args
    full_suite = "--full" in args

    if full_suite
        full_loopback_suite(; dump_inis)
    else
        iq_data, data_tx = do_txrx(; lfsr_loopback, digital_loopback, tbb_loopback, dump_inis)
        make_txrx_plots(iq_data, data_tx)
    end
end

isinteractive() || main(ARGS...)
