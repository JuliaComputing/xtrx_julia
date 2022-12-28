julia --project --threads=auto ./gps_record.jl 121c444ea8c85c --synchro &
sleep 5
julia --project --threads=auto ./gps_record.jl 30c5241b884854 --synchro &
sleep 5
julia --project --threads=auto ./gps_record.jl 12cc5241b88485c --synchro &