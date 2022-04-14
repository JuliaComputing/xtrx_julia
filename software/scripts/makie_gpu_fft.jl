using GLMakie, CUDA
using AbstractFFTs

using GLMakie: GLAbstraction

# disable scalar iteration, a performance trap where GPU memory is copied to the CPU
@eval GLMakie.GLAbstraction begin
    function gpu_getindex(b::GLBuffer{T}, range::UnitRange) where T
        error("GLBuffer getindex")
        multiplicator = sizeof(T)
        offset = first(range)-1
        value = Vector{T}(undef, length(range))
        bind(b)
        glGetBufferSubData(b.buffertype, multiplicator*offset, sizeof(value), value)
        bind(b, 0)
        return value
    end
end

function plot(; T=Float32, resolution=(800,600))
    ## dummy data generation

    NVTX.@range "data generation" CUDA.@sync begin
        # parameters of a signal
        Fs = 1000            # Sampling frequency: 1 kHz
        Ts = 1/Fs            # Sampling period
        L = 1500             # Signal duration: 1.5 seconds

        # time vector
        t = CuArray{T}((0:L-1)*Ts)

        # Form a signal
        S = 0.7*sin.(2*pi*50*t) +   # 50 Hz, amplitude 0.7
                sin.(2*pi*120*t)    # 120 Hz, amplitude 1

        # Corrupt the signal
        X = S + 2*CUDA.randn(T, size(t))    # zero-mean white noise, variance 4
    end


    ## initialization
    #
    # this should be only done once, keeping the buffer and resources across frames

    # XXX: we need create a screen, which initializes a GL Context,
    #      so that we can create a GLBuffer before having rendered anything.
    screen = GLMakie.global_gl_screen(resolution, true)

    # get a buffer object and register it with CUDA
    buffer = GLAbstraction.GLBuffer(Point2f, L÷2+1)
    resource = let
        ref = Ref{CUDA.CUgraphicsResource}()
        CUDA.cuGraphicsGLRegisterBuffer(ref, buffer.id,
                                        CUDA.CU_GRAPHICS_MAP_RESOURCE_FLAGS_WRITE_DISCARD)
        ref[]
    end

    # NOTE: Makie's out-of-place API (lines, scatter) performs may iterating operations,
    #       like determining the range of the data, so we use a manual scene instead.
    scene = Scene(; resolution)
    cam2d!(scene)

    # XXX: manually position the cameral (`center!` would iterate data)
    cam = Makie.camera(scene)
    cam.projection[] = Makie.orthographicprojection(
        #= x =# 0f0, Float32(Fs*(L÷2)/L),
        #= y =# 0f0, 1f0,
        #= z =# 0f0, 1f0)


    ## main processing
    #
    # this needs to be done every time we get new data and want to plot it

    NVTX.@range "main" begin
        # process data, generate points
        NVTX.@range "CUDA" begin
            # map OpenGL buffer object for writing from CUDA
            CUDA.cuGraphicsMapResources(1, [resource], stream())

            # get a CuArray object that we can work with
            array = let
                ptr_ref = Ref{CUDA.CUdeviceptr}()
                numbytes_ref = Ref{Csize_t}()
                CUDA.cuGraphicsResourceGetMappedPointer_v2(ptr_ref, numbytes_ref, resource)

                ptr = reinterpret(CuPtr{Point2f}, ptr_ref[])
                len = Int(numbytes_ref[] ÷ sizeof(Point2f))

                unsafe_wrap(CuArray, ptr, len)
            end

            # example processing: compute the FFT
            # NOTE: real applications will want to perform a pre-planned in-place FFT
            Y = fft(X)
            # Compute the two-sided spectrum P2
            P2 = abs.(Y/L)
            # Compute the single-sided spectrum P1 based on P2 and the even-valued signal length L.
            P1 = P2[1:L÷2+1]
            P1[2:end-1] = 2*P1[2:end-1]
            # Define the frequency domain f
            f = Fs.*(0:(L÷2))./L

            # generate points to visualize the single-sided amplitude spectrum P
            broadcast!(array, f, P1) do x, y
                Point2f(x, y)
            end

            # wait for the GPU to finish
            synchronize()

            CUDA.cuGraphicsUnmapResources(1, [resource], stream())
        end

        # generate and render plot
        NVTX.@range "Makie" begin
            # FIXME: `lines!` iterates when computing `valid_vertex`
            #        https://github.com/JuliaPlots/Makie.jl/blob/28e5fc6f130e1f1a96989895f6ccb025e74d6999/GLMakie/src/GLVisualize/visualize/lines.jl#L84-L86=
            #        so we override it here
            lines!(scene, buffer; valid_vertex=GLAbstraction.GLBuffer(ones(Float32, length(buffer))))

            # force everything to render (for benchmarking purposes)
            GLMakie.render_frame(screen, resize_buffers=false)
            GLMakie.glFinish()
        end

    end

    save("plot.png", scene)


    ## clean-up

    CUDA.cuGraphicsUnregisterResource(resource)

    return
end

# TODO:
# - make more of Makie.jl compatible with on-device buffers (only perform iterating calls
#   if `!isa(GPUArray)`), so that we don't have to use low-level rendering functions
# - alternatively, make it possible to pass CuArrays to Makie. this would then allow array
#   operations (like the `map` that `lines!` does) without falling back to scalar iteration.
#   Makie could then also perform the necessary GL Interop API calls itself.
# - should we replace the default renderloop, `set_window_config!(renderloop=myrenderloop)`?
#   https://github.com/JuliaPlots/Makie.jl/blob/36201c7c5f7e6565ce8ba9278ad077929b3fa525/GLMakie/src/rendering.jl#L40=

function main()
    CUDA.allowscalar(false)
    plot()
    return
end

isinteractive() || main()
