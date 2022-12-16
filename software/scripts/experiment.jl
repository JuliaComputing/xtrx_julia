ENV["GKSwstype"] = "100"

using SoapySDR,
    Unitful,
    DSP,
    LibSigflow,
    Statistics,
    LinearAlgebra,
    Plots,
    FFTW,
    Acquisition,
    GNSSSignals,
    OrderedCollections,
    Tracking,
    GNSSReceiver,
    PositionVelocityTime,
    StaticArrays,
    JLD2,
    Dates,
    Term

import REPL

function experiment(;
    frequency = 1565.42u"MHz",
    sampling_rate = 5e6u"Hz",
    acquisition_time = 4u"ms", # A longer time increases the SNR for satellite acquisition, but also increases the computational load. Must be longer than 1ms
    num_ants = NumAnts(4),
    write_to_file_every = 5u"s",
    gain::Unitful.Gain = 100u"dB",
    receiver_data_file = "receiver_data_$(now()).jld2",
    measurement_data_file = "measurement",
    clock_drift = 0.0
)
    isfile(receiver_data_file) && throw(ArgumentError("Your file $receiver_data_file for receiver data still exists. Please move or remove it before running this function."))

    # Do not run Devices twice
    devs = collect(Devices())
    dev1 = Device(only(filter(x -> x["driver"] == "XTRXLime" && x["serial"] == "12cc5241b88485c", devs)))
    dev2 = Device(only(filter(x -> x["driver"] == "XTRXLime" && x["serial"] == "30c5241b884854", devs)))
    close_stream_event = Base.Event()

    num_samples_acquisition = Int(upreferred(sampling_rate * acquisition_time))

    try
        format = dev1.rx[1].native_stream_format
        fullscale = dev1.tx[1].fullscale
        dev1.clock_source = "external+pps"
        dev2.clock_source = "external+pps"

        adjusted_sampling_freq = sampling_freq * (1 + clock_drift)
        # Needed for now
        ct1 = dev1.tx[1]
        ct1.bandwidth = adjusted_sampling_freq
        ct1.sample_rate = adjusted_sampling_freq

        # Needed for now
        ct2 = dev2.tx[1]
        ct2.bandwidth = adjusted_sampling_freq
        ct2.sample_rate = adjusted_sampling_freq

        # Setup receive parameters
        for cr in dev1.rx
            cr.bandwidth = adjusted_sampling_freq
            cr.frequency = frequency
            cr.sample_rate = adjusted_sampling_freq
            cr.gain = gain
            cr.gain_mode = false
        end

        for cr in dev2.rx
            cr.bandwidth = adjusted_sampling_freq
            cr.frequency = frequency
            cr.sample_rate = adjusted_sampling_freq
            cr.gain = gain
            cr.gain_mode = false
        end

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

        # RX reads the buffers in, and pushes them onto `iq_data`
        measurement_stream = stream_data(stream_rx1, stream_rx2, close_stream_event; leadin_buffers=0)

        # Satellite acquisition takes about 1s to process on a recent laptop
        # Let's take a buffer length of 5s to be on the safe side
        buffer_length = 15u"s"
        buffered_stream = membuffer(measurement_stream, ceil(Int, buffer_length * sampling_rate / stream_rx1.mtu))

        # Resizing the chunks to acquisition length
        reshunked_stream = rechunk(buffered_stream, num_samples_acquisition)

        if save_measurement
            reshunked_stream1, reshunked_stream2 = tee(reshunked_stream)

            LibSigflow.write_to_file(reshunked_stream1, measurement_data_file)
        end

        # Performing GNSS acquisition and tracking
        data_channel = receive(
            save_measurement ? reshunked_stream2 : reshunked_stream,
            system,
            sampling_rate;
            num_ants,
            num_samples = num_samples_acquisition,
            interm_freq = clock_drift * get_center_frequency(system)
        )

        data_channel1, data_channel2 = GNSSReceiver.tee(data_channel)

        gui_channel = get_gui_data_channel(data_channel1)

        Base.errormonitor(Threads.@spawn begin
            write_counter = 0
            receiver_data_buffer_length = floor(Int, upreferred(write_to_file_every / acquisition_time))
            receiver_data_buffer = Vector{GNSSReceiver.ReceiverDataOfInterest{GNSSReceiver.SatelliteDataOfInterest{StaticArraysCore.SVector{N, ComplexF64}}}}(undef, receiver_data_buffer_length)
            data_counter = 0
            GNSSReceiver.consume_channel(data_channel2) do receiver_data
                receiver_data_buffer[data_counter + 1] = receiver_data
                if data_counter + 1 == receiver_data_buffer_length
                    jldopen(receiver_data_file, "a") do fid
                        fid["$write_counter"] = receiver_data_buffer
                    end
                    write_counter += 1
                end
                data_counter = mod(data_counter + 1, receiver_data_buffer_length)
            end
        end)

        # Display the GUI
        Base.errormonitor(@async GNSSReceiver.gui(gui_channel; construct_gui_panels = make_construct_gui_panels()))

        # Read any input to close
        t = REPL.TerminalMenus.terminal
        REPL.Terminals.raw!(t, true)
        char = read(stdin, Char) 
        REPL.Terminals.raw!(t, false)
        notify(close_stream_event)
        @info("Wait a little until power down...")
        sleep(3)

    catch e
        if isa(e, InterruptException)
            notify(close_stream_event)
            @warn("SIGINT caught, early-exiting...")
            @info("Wait a little until power down...")
            sleep(3)
        else
            rethrow(e)
        end
    finally
        finalize(dev1)
        finalize(dev2)
    end
end

function experiment_single_device(;
    system::AbstractGNSS = GPSL1(),
    sampling_rate = 5e6u"Hz",
    acquisition_time = 4u"ms", # A longer time increases the SNR for satellite acquisition, but also increases the computational load. Must be longer than 1ms
    num_ants::NumAnts{N} = NumAnts(2),
    write_to_file_every = 5u"s",
    gain::Unitful.Gain = 100u"dB",
    receiver_data_file = "receiver_data_$(now()).jld2",
    save_measurement = false,
    measurement_data_file = "measurement",
    clock_drift = 0.0
) where N

    isfile(receiver_data_file) && throw(ArgumentError("Your file $receiver_data_file for receiver data still exists. Please move or remove it before running this function."))

    dev = Device(first(Devices(driver = "XTRXLime")))
    close_stream_event = Base.Event()

    num_samples_acquisition = Int(upreferred(sampling_rate * acquisition_time))

    LibSigflow.reset_xflow_stats()

    try
        format = dev.rx[1].native_stream_format
        fullscale = dev.tx[1].fullscale
#        dev.clock_source = "external+pps"

        adjusted_sampling_freq = sampling_rate * (1 + clock_drift)
         # Needed for now
        ct = dev.tx[1]
        ct.bandwidth = adjusted_sampling_freq
        ct.sample_rate = adjusted_sampling_freq

        # Setup receive parameters
        for cr in dev.rx
            cr.bandwidth = adjusted_sampling_freq
            cr.frequency = get_center_frequency(system)
            cr.sample_rate = adjusted_sampling_freq
            cr.gain = gain
            cr.gain_mode = false
        end

        stream_rx = SoapySDR.Stream(format, dev.rx)

        dma_buffers = (
            rx_hw_count = Int[],
            rx_sw_count = Int[],
            rx_user_count = Int[],
            tx_hw_count = Int[],
            tx_sw_count = Int[],
            tx_user_count = Int[],
        )

        # RX reads the buffers in, and pushes them onto `iq_data`
        measurement_stream = stream_data(stream_rx, close_stream_event; leadin_buffers=0)

        # Satellite acquisition takes about 1s to process on a recent laptop
        # Let's take a buffer length of 5s to be on the safe side
        buffer_length = 15u"s"
        buffered_stream = membuffer(measurement_stream, ceil(Int, buffer_length * sampling_rate / stream_rx.mtu))

        # Resizing the chunks to acquisition length
        reshunked_stream = rechunk(buffered_stream, num_samples_acquisition)

        if save_measurement
            reshunked_stream1, reshunked_stream2 = tee(reshunked_stream)

            LibSigflow.write_to_file(reshunked_stream1, measurement_data_file)
        end

        # Performing GNSS acquisition and tracking
        data_channel = receive(
            save_measurement ? reshunked_stream2 : reshunked_stream,
            system,
            sampling_rate;
            num_ants,
            num_samples = num_samples_acquisition,
            interm_freq = clock_drift * get_center_frequency(system)
        )

        data_channel1, data_channel2 = GNSSReceiver.tee(data_channel)

        gui_channel = get_gui_data_channel(data_channel1)

        Base.errormonitor(Threads.@spawn begin
            write_counter = 0
            receiver_data_buffer_length = floor(Int, upreferred(write_to_file_every / acquisition_time))
            receiver_data_buffer = Vector{GNSSReceiver.ReceiverDataOfInterest{GNSSReceiver.SatelliteDataOfInterest{StaticArraysCore.SVector{N, ComplexF64}}}}(undef, receiver_data_buffer_length)
            data_counter = 0
            GNSSReceiver.consume_channel(data_channel2) do receiver_data
                receiver_data_buffer[data_counter + 1] = receiver_data
                if data_counter + 1 == receiver_data_buffer_length
                    jldopen(receiver_data_file, "a") do fid
                        fid["$write_counter"] = receiver_data_buffer
                    end
                    write_counter += 1
                end
                data_counter = mod(data_counter + 1, receiver_data_buffer_length)
            end
        end)

        # Display the GUI
        Base.errormonitor(@async GNSSReceiver.gui(gui_channel; construct_gui_panels = make_construct_gui_panels_single()))

        # Read any input to close
        t = REPL.TerminalMenus.terminal
        REPL.Terminals.raw!(t, true)
        char = read(stdin, Char) 
        REPL.Terminals.raw!(t, false)
        notify(close_stream_event)
        @info("Wait a little until power down...")
        sleep(3)

    catch e
        if isa(e, InterruptException)
            notify(close_stream_event)
            @warn("SIGINT caught, early-exiting...")
            @info("Wait a little until power down...")
            sleep(3)
        else
            rethrow(e)
        end
    finally
        finalize(dev)
    end
end

function make_construct_gui_panels_single()
    function construct_gui_panels(gui_data, num_dots)
        xflows = LibSigflow.get_xflow_stats()
        panels = GNSSReceiver.construct_gui_panels(gui_data, num_dots)
        panels / Panel("Overflows: $(xflows["overflows"])", fit = true)
    end
end

function make_construct_gui_panels()
    function construct_gui_panels(gui_data, num_dots)
        xflows = LibSigflow.get_xflow_stats()["multi_overflows"]
        panels = GNSSReceiver.construct_gui_panels(gui_data, num_dots)
        panels / Panel("Overflows XTRX 1: $(xflows[1])\nOverflows XTRX 2: $(xflows[2])", fit = true)
    end
end