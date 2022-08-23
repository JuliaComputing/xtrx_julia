# configure loopback mode in the the XTRX and LMS7002M RF IC, so transmitted
# buffers should appear on the RX side.

# Make ourselves VERY verbose
#ENV["SOAPY_SDR_LOG_LEVEL"] = "11"

# Tell GR to not segfault
ENV["GKSwstype"] = "100"

using SoapySDR, Printf, Unitful
include("./libsigflow.jl")

const header_len = 8
const test_packet_len = 4080
const header_val = Complex{Int16}(0x0220, 0x0110)

function make_matching_header(buffer::Matrix{Complex{Int16}})
    header = Matrix{Complex{Int16}}(undef, header_len, size(buffer,2))
    for ch_idx in 1:size(buffer,2)
        header[1:header_len, ch_idx] .= header_val .* ch_idx
    end
    return header
end

function fill_test_buffer!(buffer::Matrix{Complex{Int16}}, buffer_idx::Int;
                           data_width::Int = 12,
                           header_len::Int = header_len,
                           header_val::Complex{Int16} = header_val)
    # Each buffer starts with this pattern
    buffer[1:header_len, :] .= make_matching_header(buffer)

    # Then we just fill the rest with the `buffer_idx`:
    max_val = 2^data_width
    #buffer[(header_len+1):end, :] .= Complex{Int16}(buffer_idx % max_val, buffer_idx % max_val)
    buffer[(header_len+1):end, :] .= Complex{Int16}(0,0)
    return buffer
end

function verify_test_buffs(in::Channel{Matrix{T}};
                           header_len::Int = header_len,
                           header_val::T = header_val,
                           test_packet_len::Int = test_packet_len,
                           verbose::Bool = true) where {T}
    # Throw away samples until we match our header
    valid_header = make_matching_header(Matrix{Complex{Int16}}(undef, header_len, 2))
    in = synchronize(in, header_len) do header
        if !all(header[1,1] .== header) && header[1,1] != Complex{Int16}(0,0) && header[end,1] != Complex{Int16}(0,0)
            if all(header[2, :] .== 0)
                @info("That's weird!")
            end
            #@info("Sync try")
            #display(reinterpret(Complex{UInt16}, header))
            #println()
        end
        return all(header .== valid_header)
    end

    # Rechunk to the test packet length, so we should get synchronized packet lengths here
    in = rechunk(in, test_packet_len)

    last_print = time()
    last_buffer_idx = nothing
    good_buffers = 0
    consume_channel(in) do buff
        @info("Looking!", size(buff))
        #display(reinterpret(Complex{UInt16}, buff[1:20, :]))
        #exit(0)

        if !all(buff[1:header_len, :] .== valid_header)
            if verbose
                @error("bad header", buff[1,1], last_buffer_idx)
                display(buff[1:header_len, :])
                println()
            end
            #last_buffer_idx = nothing
            return
        end
        if last_buffer_idx === nothing
            last_buffer_idx = buff[header_len+1, 1].re - 1
            if verbose
                @info("Synchronized", buff[header_len+1, 1].re)
            end
        end
    
        #=
        buffer_idx = (last_buffer_idx + 1)%(2^12)
        for idx in header_len+1:size(buff,1)
            val = Complex{Int16}(buffer_idx, buffer_idx)
            if !all(buff[idx,:] .== val)
                if verbose
                    @error("bad payload", idx, buff[idx,:], buffer_idx)
                end
                last_buffer_idx = nothing
                return
            end
        end
        =#
        last_buffer_idx = (last_buffer_idx + 1)%(2^12)
        good_buffers += 1

        if verbose && time() - last_print > 1
            last_print = time()
            @info("Update", good_buffers, last_buffer_idx)
        end
    end
    return good_buffers
end

function make_test_buffs(;test_packet_len::Int = test_packet_len,
                          num_channels::Int = 1,
                          leadin::Int = 2)
    buff_idx = 0
    return generate_stream(test_packet_len, num_channels; T = Complex{Int16}) do buff
        buff_idx += 1
        if buff_idx <= leadin
            buff[:,:] .= Complex{Int16}(0, 0)
            return true
        end
        #=
        if buff_idx == 1000
            buff[:,:] .= Complex{Int16}(0, 0)
            return true
        end
        =#
        fill_test_buffer!(buff, buff_idx)
        return true
    end
end

function test_digital_loopback()
    # open the first device
    devs = Devices(parse(KWArgs, "driver=lime"))
    Device(devs[1]) do dev
        # open RX and TX streams
        for c in vcat(dev.rx..., dev.tx...)
            c.sample_rate = 2u"MHz"
            c.bandwidth = 2u"MHz"
            c.frequency = 2495u"MHz"
        end
        for c in vcat(dev.rx...)
            # Set RXPGA to a high value so that we can read our filter tuning
            c[SoapySDR.GainElement(:PGA)] = -6u"dB"
        end
        stream_rx = SoapySDR.Stream(Complex{Int16}, dev.rx)
        stream_tx = SoapySDR.Stream(Complex{Int16}, dev.tx)

        # enable digital loopback
        SoapySDR.SoapySDRDevice_writeSetting(dev, "LOOPBACK_ENABLE", "TRUE")

        rx = stream_data(stream_rx, 2^32-1; leadin_buffers=0)
        rx_ready = Base.Event()

        # Stream test buffs out to the device
        tx = make_test_buffs(; test_packet_len=stream_rx.mtu, num_channels=2)
        tx = tripwire(tx, rx_ready; name="tx", verbose=true)
        #tx = log_stream_xfer(tx; title="TX")
        #stream_data(stream_tx, rechunk(tx, stream_tx.mtu))
        stream_data(stream_tx, tx)

        # Verify what we receive
        #rx = log_stream_xfer(rx; title="RX")
        rx = flowgate(rx, rx_ready; name="rx", verbose=true)
        verify_test_buffs(rx)
        #sleep(10)
        buff_idx = 1
        #=
        consume_channel(rx) do buff
            if buff_idx % 1000 == 16
                @info("Got a buff", buff_idx, buff[1,1], buff[2,1], buff[3,1])
            end
            buff_idx += 1
        end
        =#
        sleep(1)
    end
end

#SoapySDR.register_log_handler()
test_digital_loopback()
