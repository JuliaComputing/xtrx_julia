# The purpose of this test is to verify that SoapySDR does not drop any samples when
# running on a B200.
# 
# The B200's Rx port is connected to its Tx port (ideally using a cable to avoid
# the usual channel weirdness). I have included 60 dB of attenuation in the loop in order
# to avoid frying the receiver.
#
# The test consists of transmitting a known LFSR sequence, and correlating it with the
# received sequence. We expect the correlation peaks to occur with perfect periodicity. Any
# jumps in the location of the correlation peak would indicate that samples might have
# been dropped.
#
# This test relies on the B200's Tx and Rx portions sharing a single clock, which implies
# they are perfectly carrer- and symbol- synchronized.

if Threads.nthreads() < 2
    error("Must run with greater than 1 thread for full duplex transmission!")
end

using SoapySDR, SoapyUHD_jll
using Unitful
using DSP

"""
    lfsr(taps, degree)

Return a buffer with LFSR sequence specified by `taps` assuming a polynomial
of the given `degree`. The returned buffer takes values in {0.5, -0.5}.
"""
function lfsr(taps, degree)
    # Using x8 + x6 + x5 + x3 + 1 (Fibonacci) with an initial value of all ones.
    # The buffer is normalized to values in {0.5, -0.5} for UHD compatibility.
    lfsr = ones(Bool, degree)
    L = 2^degree - 1
    buff = Vector{Float32}(undef, L)
    for i = 1:L
        out = lfsr[end]
        for t in taps
            out = xor(out, lfsr[t])
        end
        lfsr[2:end] = lfsr[1:end-1]
        lfsr[1] = out
        buff[i] = out ? 1.0 : -1.0
    end
    return buff
end

function lfsr_loopback_test(run_time = 0.1 ;
                            fc       = 1.0u"GHz",
                            fs       = 1u"MHz",
                            rxgain   = 75u"dB",
                            txgain   = 53u"dB",
                            run_sanity_check = false,
                           )
    # get lfsr sequence
    lfsr_sequence = lfsr([3, 5, 6], 8)
    tx_buff = ComplexF32.(lfsr_sequence, lfsr_sequence)
    seqlen = 255

    # number of times the sequence will be transmitted/received
    numseq = ceil(Int, run_time / (seqlen/uconvert(Unitful.NoUnits, fs/1.0u"Hz")))
    # received data is stored here; one vector per iteration
    data_rx_buffs = Vector{ComplexF32}[]
    for _ in 1:numseq
        push!(data_rx_buffs, Vector{ComplexF32}(undef, seqlen))
    end

    # configure radio
    devs = Devices()
    dev = Device(devs[1])
    # receiver
    rx = dev.rx[1]
    rx.sample_rate = fs
    rx.gain = rxgain
    rx.frequency = fc
    rx_stream = SoapySDR.Stream(ComplexF32, [rx])
    # transmitter
    tx = dev.tx[1]
    tx.sample_rate = fs
    tx.gain = txgain
    tx.frequency = fc
    tx_stream = SoapySDR.Stream(ComplexF32, [tx]) # create stream

    # Events to synchronize tx/rx
    tx_ready = Base.Event()
    rx_ready = Base.Event()

    # receiver thread
    rxdone = Threads.@spawn begin
        wait(tx_ready)
        SoapySDR.activate!(rx_stream)
        notify(rx_ready)
        for i in 1:numseq
            read!(rx_stream, (data_rx_buffs[i], ))
        end
        SoapySDR.deactivate!(rx_stream)
        true
    end

    # transmitter thread
    txdone = Threads.@spawn begin
        SoapySDR.activate!(tx_stream) do
            notify(tx_ready)
            wait(rx_ready)
            for i = 1:numseq
                write(tx_stream, (tx_buff, ))
            end
            true
        end
    end

    # wait for tx and rx to finish
    fetch(rxdone)
    fetch(txdone)

    # clip received sequences and separate I and Q components
    rx_sequence = vcat(data_rx_buffs...)[200000:800000]  # trim the vector edges
    rx_I = zeros(Int, length(rx_sequence))
    for i in eachindex(rx_sequence)
        rx_I[i] = real(rx_sequence[i]) > 0 ? 1 : -1
    end
    rx_Q = zeros(Int, length(rx_sequence))
    for i in eachindex(rx_sequence)
        rx_Q[i] = imag(rx_sequence[i]) > 0 ? 1 : -1
    end

    # Calculate the cross-correlation between the transmitted LFSR sequence and the received data.
    # trim the last correlations -- the last one will be incomplete
    cr, ci = abs.(xcorr(rx_I, lfsr_sequence)[1:end-500]), abs.(xcorr(rx_Q, lfsr_sequence)[1:end-200])

    # Find peaks and detect periodicity
    # crude check that result is not too noisy
    M = maximum(cr)
    threshold = ceil(Int, 2*seqlen/3)  # two thirds of the noise-free peak value
    if M < threshold
        error("Received data too noisy; please repeat measurement")
    end
    # find first peak
    isgood(x) = x > threshold
    fi = findfirst(isgood, cr)
    I_OK = all(isgood, cr[fi:seqlen:end])
    if I_OK
        println("No lost samples were detected in I stream.")
    else
        ffail = findfirst(!isgood, cr[fi:seqlen:end])
        println("Skip in periodicity found in I stream, sample $ffail")
    end
    fi = findfirst(isgood, ci)
    Q_OK = all(isgood, ci[fi:seqlen:end])
    if Q_OK
        println("No lost samples were detected in Q stream.")
    else
        ffail = findfirst(!isgood, ci[fi:seqlen:end])
        println("Skip in periodicity found in Q stream, sample $ffail")
    end

    # sanity check: skip/insert one sample on purpose and see if we detect it.
    if run_sanity_check
        println("skipping one sample...")
        rx_I_test = vcat(rx_I[1:300_000], rx_I[300_002:end])
        cr_test = abs.(xcorr(rx_I_test, lfsr_sequence)[1:end-500])
        fi = findfirst(isgood, cr_test)
        I_OK_test_s = all(isgood, cr_test[fi:seqlen:end])
        if I_OK_test_s
            println("    Error: Skipped sample was not detected.")
        else
            println("    Sanity test successful: skipped sample detected.")
        end
        println("inserting one sample...")
        rx_I_test = vcat(rx_I[1:300_000], [1], rx_I[300_001:end])
        cr_test = abs.(xcorr(rx_I_test, lfsr_sequence)[1:end-500])
        fi = findfirst(isgood, cr_test)
        I_OK_test_i = all(isgood, cr_test[fi:seqlen:end])
        if I_OK_test_i
            println("    Error: Inserted sample was not detected.")
        else
            println("    Sanity test successful: inserted sample detected.")
        end
    end

    return I_OK, Q_OK
end
