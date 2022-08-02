# configure loopback mode in the the XTRX and LMS7002M RF IC, so transmitted
# buffers should appear on the RX side.

if !haskey(ENV, "SOAPY_SDR_PLUGIN_PATH") || isempty(ENV["SOAPY_SDR_PLUGIN_PATH"])
    ENV["SOAPY_SDR_PLUGIN_PATH"] = joinpath(@__DIR__, "../soapysdr-xtrx/build")
end

@show ENV["SOAPY_SDR_PLUGIN_PATH"]

using SoapySDR, Printf, Unitful

SoapySDR.register_log_handler()


function dma_test()
    # open the first device
    devs = Devices()
    dev_args = devs[1]
    dev_args["ini"] = joinpath(@__DIR__, "../configs/xtrx_limesdr.ini")
    dev = Device(dev_args)

    #dev.master_clock_rate = 61.44e6
    #@show dev.master_clock_rate

    #SoapySDR.SoapySDRDevice_writeSetting(dev, "LOOPBACK_ENABLE", "TRUE")

    # get the RX and TX channels
    # XX We suspect they are interlaced somehow, so analyize
    chan_rx = dev.rx[1]
    chan_tx = dev.tx[1]
    println(chan_rx)
    println(chan_rx)
    # open RX and TX streams
    format = Complex{Int16} #chan_rx.native_stream_format
    fullscale = chan_tx.fullscale
    stream_rx = SoapySDR.Stream(format, dev.rx)
    stream_tx = SoapySDR.Stream(format, dev.tx)

    @info "Streaming format: $format"

    # the size of every buffer, in bytes
    mtu = SoapySDR.SoapySDRDevice_getStreamMTU(dev, stream_tx)
    wr_sz = SoapySDR.SoapySDRDevice_getStreamMTU(dev, stream_tx) * sizeof(format)
    rd_sz = SoapySDR.SoapySDRDevice_getStreamMTU(dev, stream_rx) * sizeof(format)
    @assert wr_sz == rd_sz

    # the number of buffers each stream has
    wr_nbufs = SoapySDR.SoapySDRDevice_getNumDirectAccessBuffers(dev, stream_tx)
    rd_nbufs = SoapySDR.SoapySDRDevice_getNumDirectAccessBuffers(dev, stream_rx)
    @assert wr_nbufs == rd_nbufs


    # the total size of the stream's buffers, in bytes
    wr_total_sz = wr_sz * wr_nbufs
    rd_total_sz = rd_sz * rd_nbufs
    @info "number of buffers: $(Int(wr_nbufs)), buffer size (bytes): $(Int(wr_sz))"

    # TODO Set Antennas??? GainElements??

    # Setup transmission/recieve parameters
    # XXX: Sometimes this needs to be done twice to not error???
    for cr in dev.rx
        #for ge in cr.gain_elements
        #    cr[ge] = 12u"dB"
        #end
        cr.bandwidth = 1u"MHz" # 200u"kHz"
        @show cr.bandwidth
        cr.frequency = 2.498u"GHz"
        @show cr.frequency
        cr.gain = 0u"dB"
        @show cr.gain
        cr.sample_rate = 2u"MHz"
        @show cr.sample_rate
        println(cr)
    end

    for ct in dev.tx
        for ge in ct.gain_elements
            ct[ge] = -20u"dB"
        end
        ct.bandwidth = 1u"MHz" #2u"MHz"
        @show ct.bandwidth
        ct.frequency = 2.498u"GHz"
        @show ct.frequency
        #ct.gain = 20u"dB"
        #@show ct.gain
        ct.sample_rate = 2u"MHz"
        @show ct.sample_rate
        println(ct)
    end
    # prepare some data to send:
    rate = 20
    samples = mtu*wr_nbufs
    t = (1:div(samples,1))./samples
    data_tx = zeros(format, samples)
    #data_tx[1:2:end] = format.(round.(sin.(2π.*t.*rate).*fullscale/4), 0)
    # Transmit on both real and imaginary as perperdicular signals
    # to ensure we always have a signal on one of I or Q
    #data_tx = convert.(format, round.(cis.(2π.*t.*rate).*fullscale/4))

    iq_data = format[]

    try

        written_buffs = 0
        read_buffs = 0

        SoapySDR.activate!(stream_tx)
        SoapySDR.activate!(stream_rx)

        @info "writing TX"
        # write tx-buffer
        while written_buffs < wr_nbufs
            buffs = Ptr{format}[C_NULL]
            err, handle = SoapySDR.SoapySDRDevice_acquireWriteBuffer(dev, stream_tx, buffs, 0)
            if err == SoapySDR.SOAPY_SDR_TIMEOUT # all buffers are full
                break
            elseif err == SoapySDR.SOAPY_SDR_UNDERFLOW
                err = 1 # keep going
            end
            @assert err > 0
            unsafe_copyto!(buffs[1], pointer(data_tx, mtu*written_buffs+1), mtu)
            SoapySDR.SoapySDRDevice_releaseWriteBuffer(dev, stream_tx, handle, 1)
            written_buffs += 1
        end

        @info "reading RX"
        # read/check rx-buffer
        while read_buffs < rd_nbufs
            buffs = Ptr{format}[C_NULL]
            err, handle, flags, timeNs = SoapySDR.SoapySDRDevice_acquireReadBuffer(dev, stream_rx, buffs, 0)
            if err == SoapySDR.SOAPY_SDR_TIMEOUT
                continue
            elseif err == SoapySDR.SOAPY_SDR_OVERFLOW
                err = 1 # nothing to do, should be the MTU
            end
            @assert err > 0

            arr = unsafe_wrap(Vector{format}, buffs[1], mtu)
            append!(iq_data, copy(arr))

            SoapySDR.SoapySDRDevice_releaseReadBuffer(dev, stream_rx, handle)
            read_buffs += 1
        end
        @assert read_buffs == written_buffs

    finally
        SoapySDR.deactivate!(stream_rx)
        SoapySDR.deactivate!(stream_tx)
    end
    # close everything
    finalize.([stream_rx, stream_tx])
    finalize(dev)

    return iq_data, data_tx
end

iq_data, data_tx = dma_test()

using Plots

function get_bit(word::Unsigned, word_length::Int, bit_number::Int)
    Bool(get_bits(word, word_length, bit_number, 1))
end

function get_bits(word::Unsigned, word_length::Int, start::Int, length::Int)
    Int((word >> (word_length - start - length + 1)) & (UInt(1) << UInt(length) - UInt(1)))
end

"""
e.g. 4 bit twos complement inside 8 bit byte
00001000 = -8
00000111 = 8
becomes:
11111000 = -8

"""
function get_two_complement_num(word::Unsigned, word_length::Int, start::Int, length::Int)
    sign = get_bit(word, word_length, start)
    value = get_bits(word, word_length, start, length)
    if sign
        return -1 << length + value
    else
        return value
    end
end

# correct the data
iq_data_flat = reinterpret(Int16, iq_data)
for idx in 1:length(iq_data_flat)
    if (iq_data_flat[idx] > 2048)
        iq_data_flat[idx] = iq_data_flat[idx] - 4096
    end 
end
iq_data = reinterpret(Complex{Int16}, iq_data_flat)



# Pull out real and imaginary for each channels
# e.g.
# (ch1_r, ch1_i), (ch2_r, ch2_i) , ...

len = length(iq_data)

plt = plot(imag.(iq_data)[2:2:len], label="ch1_i")
plot!(imag.(iq_data)[1:2:len], label="ch2_i")
plot!(real.(iq_data)[2:2:len], label="ch1_r")
plot!(real.(iq_data)[1:2:len], label="ch2_r")
plot!(real.(data_tx)[1:div(len,2)], label="data_tx_r")
plot!(imag.(data_tx)[1:div(len,2)], label="data_tx_i")

savefig("plots/data_$(Int(round(time()))).png")

#display(plt)
