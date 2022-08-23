# configure loopback mode in the the XTRX and LMS7002M RF IC, so transmitted
# buffers should appear on the RX side.

# Make ourselves VERY verbose
ENV["SOAPY_SDR_LOG_LEVEL"] = "11"

# Tell GR to not segfault
ENV["GKSwstype"] = "100"

using SoapySDR, Printf, Unitful

SoapySDR.register_log_handler()

function test_rf_loopback()
    # open the first device
    devs = Devices(parse(KWArgs, "driver=lime"))
    Device(devs[1]) do dev
        # get the RX and TX channels
        # XX We suspect they are interlaced somehow, so analyize
        chan_rx = dev.rx[1]
        chan_tx = dev.tx[1]

        chan_tx.gain = 52u"dB"
        chan_tx.sample_rate = 1u"MHz"
        chan_rx.sample_rate = 1u"MHz"

        SoapySDR.SoapySDRDevice_writeSetting(dev, "LB_LNA", "60")

        chan_rx.antenna = SoapySDR.Antenna(:LB1)
        #chan_rx.antenna = SoapySDR.Antenna(:LNAL)

        # open RX and TX streams
        format = chan_rx.native_stream_format
        stream_rx = SoapySDR.Stream(format, chan_rx)
        stream_tx = SoapySDR.Stream(format, chan_tx)

        @info "Streaming format: $format"

        #SoapySDR.SoapySDRDevice_writeSetting(dev, "DUMP_INI", "test20220818_post_config.ini")

        data_tx = rand(format, stream_tx.mtu)
        data_zr = zeros(format, stream_tx.mtu)
        iq_data = format[]
        SoapySDR.activate!(stream_tx) do
            SoapySDR.activate!(stream_rx) do
                tx_go = Base.Event()
                rx_go = Base.Event()
                @sync begin
                    Threads.@spawn begin
                        notify(tx_go)
                        wait(rx_go)
                        for _ in 1:8
                            buff = zeros(format, stream_rx.mtu)
                            read!(stream_rx, (buff,); timeout=1u"s")
                            println("rx!")
                            append!(iq_data, buff)
                        end
                    end

                    Threads.@spawn begin
                        notify(rx_go)
                        wait(tx_go)
                        for buff_idx in 1:8
                            if buff_idx % 2 == 0
                                buff = data_tx
                            else
                                buff = data_zr
                            end
                            println("tx! $(sum(abs.(buff)))")
                            write(stream_tx, (buff,); timeout=1u"s")
                        end
                    end
                end
            end
        end
        return iq_data, data_tx
    end
end

test_rf_loopback()
iq_data, data_tx = test_rf_loopback()

using Plots

begin
    plt = plot(real.(iq_data[10000:end]))
    #plot!(imag.(iq_data)[1:2])
    savefig("data.png")
end

#plot!(real.(data_tx) .+ 40000)
