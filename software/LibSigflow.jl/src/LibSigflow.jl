module LibSigflow

using SoapySDR, Printf, DSP, FFTW, Statistics, StaticArrays

export generate_stream, stream_data, membuffer, tee, flowgate, tripwire, rechunk,
       generate_chirp, stft, absshift, reduce, log_stream_xfer, collect_buffers,
       collect_psd, consume_channel, spawn_channel_thread, streaming_filter,
       write_to_file, stream_channel, calc_periodograms, self_downconvert,
       append_vectors, complex2float, MatrixSizedChannel, VectorSizedChannel,
       AbstractSizedChannel, transform, collect_single_chunk_at

include("sized_channel.jl")

# Helper for turning a matrix into a tuple of views, for use with the SoapySDR API.
function split_matrix(m::AbstractMatrix{T}) where {T <: Number}
    return tuple(collect(view(m, :, idx) for idx in 1:size(m,2))...)
end

_default_verbosity = false
function set_libsigflow_verbose(verbose::Bool)
    global _default_verbosity = verbose
end
const _num_overflows = Ref{Int64}(0)
const _num_underflows = Ref{Int64}(0)

function reset_xflow_stats()
    _num_overflows[] = Int64(0)
    _num_underflows[] = Int64(0)
end

function get_xflow_stats()
    return Dict(
        "overflows" => _num_overflows[],
        "underflows" => _num_underflows[],
    )
end

"""
    spawn_channel_thread(f::Function)

Use this convenience wrapper to invoke `f(out_channel)` on a separate thread, closing
`out_channel` when `f()` finishes.
"""
function spawn_channel_thread(f::Function; T::DataType = ComplexF32,
                              num_samples = nothing, num_antenna_channels = nothing,
                              buffers_in_flight::Int = 0)
    out = select_appropriate_channel(num_samples, num_antenna_channels, T, buffers_in_flight)
    Base.errormonitor(Threads.@spawn begin
        try
            f(out)
        finally
            close(out)
        end
    end)
    return out
end

function select_appropriate_channel(num_samples::Nothing, num_antenna_channels::Nothing, T::DataType, sz)
    Channel{Matrix{T}}(sz)
end
function select_appropriate_channel(num_samples::Nothing, num_antenna_channels::Int, T::DataType, sz)
    VectorSizedChannel{T}(num_antenna_channels, sz)
end
function select_appropriate_channel(num_samples, num_antenna_channels, T::DataType, sz)
    MatrixSizedChannel{T}(num_samples, num_antenna_channels, sz)
end


"""
    membuffer(in, max_size = 16)

Provide some buffering for realtime applications.
"""
function membuffer(in::MatrixSizedChannel{T}, max_size::Int = 16) where {T <: Number}
    spawn_channel_thread(;T, in.num_samples, in.num_antenna_channels, buffers_in_flight=max_size) do out
        consume_channel(in) do buff
            put!(out, buff)
        end
    end
end

"""
    transform(in, func)

Transform data
"""
function transform(in::AbstractSizedChannel{T}, func) where T
    spawn_channel_thread(;T, num_samples = in isa MatrixSizedChannel ? in.num_samples : nothing, in.num_antenna_channels) do out
        consume_channel(in) do buff
            put!(out, func(buff))
        end
    end
end


"""
    generate_stream(gen_buff!::Function, buff_size, num_channels)

Returns a `Channel` that allows multiple buffers to be 
"""
function generate_stream(gen_buff!::Function, num_samples::Integer, num_antenna_channels::Integer;
                         wrapper::Function = (f) -> f(),
                         buffers_in_flight::Integer = 1,
                         T = ComplexF32)
    return spawn_channel_thread(;T, num_samples, num_antenna_channels, buffers_in_flight) do c
        wrapper() do
            buff = Matrix{T}(undef, num_samples, num_antenna_channels)

            # Keep on generating buffers until `gen_buff!()` returns `false`.
            while gen_buff!(buff)
                put!(c, copy(buff))
            end
        end
    end
end
function generate_stream(f::Function, s::SoapySDR.Stream{T}; kwargs...) where {T <: Number}
    return generate_stream(f, s.mtu, s.nchannels; T, kwargs...)
end

"""
    stream_data(s_rx::SoapySDR.Stream, end_condition::Union{Integer,Event})

Returns a `Channel` which will yield buffers of data to be processed of size `s_rx.mtu`.
Starts an asynchronous task that does the reading from the stream, until the requested
number of samples are read, or the given `Event` is notified.
"""
function stream_data(s_rx::SoapySDR.Stream{T}, end_condition::Union{Integer,Base.Event};
                     leadin_buffers::Integer = 16,
                     kwargs...) where {T <: Number}
    # Wrapper to activate/deactivate `s_rx`
    wrapper = (f) -> begin
        buff = Matrix{T}(undef, s_rx.mtu, s_rx.nchannels)

        # Let the stream come online for a bit
        SoapySDR.activate!(s_rx) do
            while leadin_buffers > 0
                read!(s_rx, split_matrix(buff))
                leadin_buffers -= 1
            end

            # Invoke the rest of `generate_stream()`
            f()
        end
    end

    # Read streams until we read the number of samples, or the given event
    # is triggered
    buff_idx = 0
    return generate_stream(s_rx.mtu, s_rx.nchannels; wrapper, T, kwargs...) do buff
        if isa(end_condition, Integer)
            if buff_idx*s_rx.mtu >= end_condition
                return false
            end
        else
            if end_condition.set
                return false
            end
        end

        flags = Ref{Int}(0)
        try
            read!(s_rx, split_matrix(buff); flags, timeout=0.9u"s", throw_error = true)
        catch e
            if e isa SoapySDR.SoapySDRDeviceError
                if e.status == SoapySDR.SOAPY_SDR_OVERFLOW
                    _num_overflows[] += 1
                    print("O")
                elseif e.status == SoapySDR.SOAPY_SDR_TIMEOUT
                    print("Tᵣ")
                else
                    print("Eᵣ")
                end
            else
                rethrow(e)
            end
        end

        buff_idx += 1
        return true
    end
end

"""
    stream_data(s_tx, in::Channel)

Feed data from a `Channel` out onto the airwaves via a given `SoapySDR.Stream`.
We suggest using `rechunk()` to convert to `s_tx.mtu`-sized buffers for maximum
efficiency.
"""
function stream_data(s_tx::SoapySDR.Stream{T}, in::MatrixSizedChannel{T}) where {T <: Number}
    Base.errormonitor(Threads.@spawn begin
        SoapySDR.activate!(s_tx) do
            # Consume channel and spit out into `s_tx`
            consume_channel(in) do buff
                flags = Ref{Int}(0)
                try
                    write(s_tx, split_matrix(buff); flags, timeout=0.9u"s", throw_error = true)
                catch e
                    if e isa SoapySDR.SoapySDRDeviceError
                        if e.status == SoapySDR.SOAPY_SDR_UNDERFLOW
                            _num_underflows[] += 1
                            print("U")
                        elseif e.status == SoapySDR.SOAPY_SDR_TIMEOUT
                            print("Tₜ")
                        else
                            print("Eₜ")
                        end
                    else
                        rethrow(e)
                    end
                end
#                println("Flags: ", bitstring(flags[]))
            end

            # We need to `sleep()` until we're done transmitting,
            # otherwise we `deactivate!()` a little bit too eagerly.
            # Let's just assume we never transmit more than 1s at a time, for now.
            sleep(1)
        end
    end)
end

"""
    stream_data(paths::Vector{String}, in::Channel)

Feed data from a `Channel` out onto files on disk.  Uses raw bit format of
whatever datatype is given.  We suggest encoding the format of the data in
the filename, for example using the filenames:

    seattle_gps-2022-09-02-f1575.42-s5.00-g81-rx1.sc16
    seattle_gps-2022-09-02-f1575.42-s5.00-g81-rx2.sc16

is a succinct way to tell the user the of nature of the data contents, the
date of capture, the frequency, sampling rate, gain, channel and format.
The number of paths given must match the number of channels streaming in.
"""
function stream_data(paths::Vector{<:AbstractString}, in::MatrixSizedChannel{T}) where {T <: Number}
    fds = [open(path, write=true) for path in paths]

    return Base.errormonitor(Threads.@spawn begin
        try
            consume_channel(in) do data
                if size(data, 2) != length(fds)
                    throw(ArgumentError("Data channels $(size(data,2)) must match number of paths given $(length(fds))"))
                end

                # Write these buffers out to disk as fast as we can
                for (idx, fd) in enumerate(fds)
                    write(fd, data[:, idx])
                end
            end
        finally
            # Always close all of our fds
            close.(fds)
        end
    end)
end

"""
    stream_data(paths::Vector{<:AbstractString}, T::DataType; chunk_size)

Read in a set of files as a coherent chunk of channels.  Use `chunk_size`
to set the initial stream buffer chunk size (defaults to a 4K page on disk)
"""
function stream_data(paths::Vector{<:AbstractString}, T::DataType;
                     chunk_size::Int = div(4096, sizeof(T)))
    fds = [open(path, read=true) for path in paths]
    # Ensure that we close everything at the end
    wrapper = (f) -> begin
        try
            f()
        finally
            close.(fds)
        end
    end 

    return generate_stream(chunk_size, length(paths); T) do buff
        for (idx, fd) in enumerate(fds)
            try
                read!(fd, view(buff, :, idx))
            catch e
                # Stop generating as soon as a single file runs out of content.
                if isa(e, EOFError)
                    return false
                end
                rethrow(e)
            end
        end

        return true
    end
end

"""
    generate_test_pattern(pattern_len; num_channels = 1, num_buffers = 1)

Generate a test pattern, used in our test suite.  Always generates buffers with
length equal to `pattern_len`, if you need to change that, use `rechunk`.
Transmits `num_buffers` and then quits.
"""
function generate_test_pattern(pattern_len::Integer; num_channels::Int = 1, num_buffers::Integer = 1, T::DataType = ComplexF32)
    buffs_sent = 0
    return generate_stream(pattern_len, num_channels; T) do buff
        if buffs_sent >= num_buffers
            return false
        end

        for idx in 1:pattern_len
            buff[idx, :] .= T(idx, idx)
        end
        buffs_sent += 1
        return true
    end
end

"""
    generate_chirp(chirp_len; num_channels = 1, num_buffers = 1)

Generate a linear chirp from 0 -> fs/2 over `chirp_len` samples.
Always generates buffers with length equal to `chirp_len`, if you need to
change that, use `rechunk`.  Transmits `num_buffers` and then quits.
"""
function generate_chirp(chirp_len::Integer; num_channels::Integer = 1, num_buffers::Integer = 1, T::DataType = ComplexF32)
    buffs_sent = 0
    return generate_stream(chirp_len, num_channels; T) do buff
        if buffs_sent >= num_buffers
            return false
        end

        for idx in 1:chirp_len
            buff[idx, :] .= sin(idx.^2 * π / (2*chirp_len))
        end
        buffs_sent += 1
        return true
    end
end

"""
    consume_channel(f::Function, c::Channel, args...)

Consumes the given channel, calling `f(data, args...)` where `data` is what is
taken from the given channel.  Returns when the channel closes.
"""
function consume_channel(f::Function, c::AbstractChannel, args...)
    while !isempty(c) || isopen(c)
        local data
        try
            data = take!(c)
        catch e
            if isa(e, InvalidStateException)
                continue
            end
            rethrow(e)
        end
        f(data, args...)
    end
end

"""
    tee(in::Channel)

Returns two channels that synchronously output what comes in from `in`.
"""
function tee(in::MatrixSizedChannel{T}) where {T <: Number}
    out1 = MatrixSizedChannel{T}(in.num_samples, in.num_antenna_channels)
    out2 = MatrixSizedChannel{T}(in.num_samples, in.num_antenna_channels)
    Base.errormonitor(Threads.@spawn begin
        consume_channel(in) do data
            put!(out1, data)
            put!(out2, data)
        end
        close(out1)
        close(out2)
    end)
    return (out1, out2)
end

"""
    rechunk(in::Channel, chunk_size::Int)

Converts a stream of chunks with size A to a stream of chunks with size B.
"""
function rechunk(in::MatrixSizedChannel{T}, chunk_size::Integer) where {T <: Number}
    return spawn_channel_thread(;T, num_samples = chunk_size, in.num_antenna_channels) do out
        chunk_filled = 0
        chunk_idx = 1
        # We'll alternate between filling up these three chunks, then sending
        # them down the channel.  We have three so that we can have:
        # - One that we're modifying,
        # - One that was sent out to a downstream,
        # - One that is being held by an intermediary
        chunks = [
            Matrix{T}(undef, chunk_size, in.num_antenna_channels),
            Matrix{T}(undef, chunk_size, in.num_antenna_channels),
            Matrix{T}(undef, chunk_size, in.num_antenna_channels),
        ]
        consume_channel(in) do data
            # Make the loop type-stable
            data = view(data, 1:size(data, 1), :)

            # Generate chunks until this data is done
            while !isempty(data)

                # How many samples are we going to consume from this buffer?
                samples_wanted = (chunk_size - chunk_filled)
                samples_taken = min(size(data, 1), samples_wanted)

                # Copy as much of `data` as we can into `chunks`
                chunks[chunk_idx][chunk_filled+1:chunk_filled + samples_taken, :] = data[1:samples_taken, :]
                chunk_filled += samples_taken

                # Move our view of `data` forward:
                data = view(data, samples_taken+1:size(data, 1), :)

                # If we filled the chunk completely, then send it off and flip `chunk_idx`:
                if chunk_filled >= chunk_size
                    put!(out, chunks[chunk_idx])
                    chunk_idx = mod1(chunk_idx + 1, length(chunks))
                    chunk_filled = 0
                end
            end
        end
    end
end

"""
    stft(in::Channel)

Stream an FFT of the buffers coming in on `Channel`.  Use `rechunk()` on the
input to force a time window size.  Combine with `reduce` to perform
grouping/reductions across time.
"""
function stft(in::MatrixSizedChannel{T};
              window_function::Function = DSP.hanning) where {T <: Number}
    BUFF = Matrix{T}(undef, in.num_samples, in.num_antenna_channels)
    fft_plan = FFTW.plan_fft(BUFF, 1)
    win = T.(window_function(in.num_samples))
    
    return spawn_channel_thread(;T, in.num_samples, in.num_antenna_channels) do out
        consume_channel(in) do buff

            # Perform the frequency transform
            FFTW.mul!(BUFF, fft_plan, buff .* win)
            put!(out, copy(BUFF))
        end
    end
    return out
end

function absshift(in::MatrixSizedChannel{T}) where {T <: Number}
    # Note this coerces to Float32
    spawn_channel_thread(; T=Float32, in.num_samples, in.num_antenna_channels) do out
        consume_channel(in) do buff
            val = FFTW.fftshift(Float32.(abs.(buff)))
            put!(out, val)
        end
    end
end

"""
    reduce(reductor::Function, in::Channel, reduction_factor::Integer)

Buffers `reduction_factor` buffers together into a vector, then calls
`reductor(buffs)`, pushing the result out onto a `Channel`.
"""
function Base.reduce(reductor::Function, in::MatrixSizedChannel{T}, reduction_factor::Integer; verbose::Bool = _default_verbosity) where {T <: Number}
    spawn_channel_thread(;T, in.num_samples, in.num_antenna_channels) do out
        buff_idx = 1
        acc = Array{T,3}(undef, in.num_samples, in.num_antenna_channels, reduction_factor)
        consume_channel(in) do buff
            acc[:, :, buff_idx] .= buff
            buff_idx += 1
            if buff_idx > reduction_factor
                put!(out, reductor(acc))
                buff_idx = 1
            end
        end
    end
end

"""
    reduce(reductor::Function, in::Channel)

reduces input by `reductor` function.
"""
function Base.reduce(reductor::Function, in::MatrixSizedChannel{T}) where {T <: Number}
    spawn_channel_thread(;T, num_samples = 1, in.num_antenna_channels) do out
        consume_channel(in) do signals
            reduced_signal = map(reductor, eachcol(signals))
            push!(out, collect(reshape(reduced_signal, 1, in.num_antenna_channels)))
        end
    end
end

"""
    complex2float(complex2float_function::Function, in::Channel)

Converts complex number to float
"""
function complex2float(complex2float_function::Function, in::MatrixSizedChannel{Complex{T}}) where {T <: Real}
    spawn_channel_thread(;T, in.num_samples, in.num_antenna_channels) do out
        consume_channel(in) do signals
            push!(out, complex2float_function.(signals))
        end
    end
end

"""
    append_vectors(in::Channel)

Concat vectors to matrix
"""
function append_vectors(in::VectorSizedChannel{T}) where {T}
    spawn_channel_thread(;T = Vector{T}, in.num_antenna_channels) do out
        buffs = [Vector{T}(undef, 0) for _ in 1:in.num_antenna_channels]
        consume_channel(in) do signals
            foreach(buffs, signals) do buff, signal
                push!(buff, signal)
            end
            push!(out, deepcopy(buffs)) # Copy to avoid race condition
        end
    end
end

"""
    collect_buffers(in::Channel)

Consume a channel, storing the buffers, then `cat()`'ing them
into a giant array.  Automatically caps the number of buffers
that can be slapped together at 4000, due to the inefficient
implementation of `cat()` in Julia v1.8 and earlier.
"""
function collect_buffers(in::MatrixSizedChannel{T}; max_buffers::Int = 4000) where {T <: Number}
    buffs = Matrix{T}[]
    consume_channel(in) do buff
        if size(buffs, 1) < max_buffers
            push!(buffs, buff)
        end
    end
    return cat(buffs...; dims=1)
end

"""
    write_to_file(in::Channel, file_path)

Consume a channel and write to file(s). Multiple channels will
be written to different files. The channel number is appended
to the filename.
"""
function write_to_file(in::MatrixSizedChannel{T}, file_path::String) where {T <: Number}
    type_string = string(T)
    streams = [open("$file_path$type_string$i.dat", "w") for i in 1:in.num_antenna_channels]
    try
        consume_channel(in) do buffs
            foreach(eachcol(buffs), streams) do buff, stream
                write(stream, buff)
            end
        end
    finally
        close.(streams)
    end
end

function collect_psd(in::MatrixSizedChannel{T}, freq_size::Integer, buff_size::Integer; accumulation = :max) where {T <: Number}
    # Precaculate our reduction parameters
    reduction_factor = div(buff_size, freq_size)
    if accumulation == :max
        reductor = buffs -> maximum(buffs, dims=3)[:, :, 1]
    elseif accumulation == :mean
        reductor = buffs -> mean(buffs, dims=3)[:, :, 1]
    else
        throw(ArgumentError("Invalid accumulation algorithm '$(accumulation)'"))
    end

    # Reduce the absolute value, fft-shifted, STFT'ed input
    reduced = reduce(reductor,
        absshift(stft(rechunk(in, freq_size))),
        reduction_factor,
    )

    # We'll store our PSD frames here, then concatenate into a giant matrix later
    psd_frames = Matrix{Float32}[]
    consume_channel(reduced) do buff
        push!(psd_frames, buff[:, :])
    end
    return permutedims(cat(psd_frames..., dims=3), (1,3,2))
end

"""
    log_stream_xfer(in::Channel)

Logs messages summarizing our data transfer to stdout.
"""
function log_stream_xfer(in::MatrixSizedChannel{T}; title = "Xfer", print_period = 1.0, α = 0.7, extra_values::Function = () -> (;)) where {T <: Number}
    spawn_channel_thread(;T, in.num_samples, in.num_antenna_channels) do out
        start_time = time()
        last_print = start_time
        total_samples = 0
        buffers = 0
        consume_channel(in) do data
            buffers += 1
            total_samples += size(data,1)

            curr_time = time()
            if curr_time - last_print > print_period
                duration = curr_time - start_time
                samples_per_sec = total_samples/duration
                @info(title,
                    buffers,
                    buffer_size = size(data),
                    total_samples,
                    over_and_underflows = (_num_overflows[], _num_underflows[]),
                    samples_per_sec = @sprintf("%.1f MHz", samples_per_sec/1e6),
                    data_rate = @sprintf("%.1f MB/s", samples_per_sec * sizeof(T)/1e6),
                    duration = @sprintf("%.1f s", duration),
                    extra_values()...,
                )
                last_print = curr_time
            end
            put!(out, data)
        end
        duration = time() - start_time
        samples_per_sec = total_samples/duration
        @info("$(title) - DONE",
            buffers,
            total_samples,
            samples_per_sec = @sprintf("%.1f MHz", samples_per_sec/1e6),
            data_rate = @sprintf("%.1f MB/s", samples_per_sec * sizeof(T)/1e6),
            duration = @sprintf("%.1f s", duration),
        )
    end
end

"""
    flowgate(in::Channel, ctl::Base.Event)

Waits upon `ctl` before passing buffers through; useful for synchronization.
"""
function flowgate(in::MatrixSizedChannel{T}, ctl::Base.Event;
                  name::String = "flowgate", verbose::Bool = _default_verbosity) where {T <: Number}
    spawn_channel_thread(;T, in.num_samples, in.num_antenna_channels) do out
        already_printed = false
        consume_channel(in) do buff
            wait(ctl)
            if verbose && !already_printed
                @info("$(name) triggered", time=time())
                already_printed = true
            end
            put!(out, buff)
        end
    end
end

"""
    tripwire(in::Channel, ctl::Base.Event)

Notifies `ctl` when a buffer passes through.
"""
function tripwire(in::MatrixSizedChannel{T}, ctl::Base.Event;
                  name::String = "tripwire", verbose::Bool = _default_verbosity) where {T <: Number}
    spawn_channel_thread(;T, in.num_samples, in.num_antenna_channels) do out
        already_printed = false
        consume_channel(in) do buff
            notify(ctl)
            if verbose && !already_printed
                @info("$(name) triggered", time=time())
                already_printed = true
            end
            put!(out, buff)
        end
    end
end

# We used to do this in Julia, but now we do it in the soapysdr-xtrx driver.
# Eventually we may do it in the FPGA, or even transmit 24-bit IQ clusters.
function sign_extend!(x::AbstractArray{Complex{Int16}})
    xi = reinterpret(Int16, x)
    for idx in 1:length(xi)
        if xi[idx] >= (1 << 11)
            xi[idx] -= (1 << 12)
        end
    end
    return x
end

# This is basically only useful for `test_pattern`
function un_sign_extend!(x::AbstractArray{Complex{Int16}})
    xi = reinterpret(Int16, x)
    for idx in 1:length(xi)
        if xi[idx] < 0
            xi[idx] += (1 << 12)
        end
    end
    return x
end

function streaming_filter(in::MatrixSizedChannel{T}, filter_coeffs::Vector{K}) where {T, K <: Number}
    spawn_channel_thread(;T=promote_type(T,K), in.num_samples, in.num_antenna_channels) do out
        # Create N different filter state objects,
        # one for each channel we're filtering
        filters = [FIRFilter(filter_coeffs) for _ in 1:in.num_antenna_channels]
        consume_channel(in) do buff
            out_buff = Matrix{promote_type(T,K)}(undef, size(buff)...)
            for ch_idx in 1:size(buff,2)
                buff_slice = view(buff, :, ch_idx)
                out_buff_slice = view(out_buff, :, ch_idx)
                filt!(out_buff_slice, filters[ch_idx], buff_slice)
            end
            put!(out, out_buff)
        end
    end
end

function self_downconvert(in::MatrixSizedChannel{T}, reference_channel = 1) where T <: Number
    spawn_channel_thread(;T, in.num_samples, num_antenna_channels = in.num_antenna_channels - 1) do out
        consume_channel(in) do signals
            other_channels = filter(x -> x != reference_channel, 1:size(signals, 2))
            out_buff = signals[:,other_channels] ./ signals[:,reference_channel]
            put!(out, out_buff)
        end
    end
end

function calc_periodograms(in::MatrixSizedChannel{Complex{T}}; sampling_freq) where T <: Number
    spawn_channel_thread(;
        T = DSP.Periodograms.Periodogram{
            T,
            AbstractFFTs.Frequencies{Float64},
            Vector{T}
        },
        in.num_antenna_channels
    ) do out
        consume_channel(in) do data
            out_buff = map(eachcol(data)) do samples
                periodogram(samples, fs = sampling_freq)
            end
            put!(out, out_buff)
        end
    end
end

function collect_single_chunk_at(in::MatrixSizedChannel{T}; counter_threshold::Int = 1000) where {T <: Number}
    buffs = Matrix{T}(undef, in.num_samples, in.num_antenna_channels)
    counter = 0
    spawn_event = Base.errormonitor(Threads.@spawn begin 
        consume_channel(in) do buff
            if counter == counter_threshold
                buffs .= buff
            end
            counter += 1
        end
    end)
    return buffs, spawn_event
end

end # module LibSigflow
