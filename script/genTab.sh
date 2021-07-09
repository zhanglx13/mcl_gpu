#! /usr/bin/env bash

# table for interval
gnuplot -e "col=7" process.gp
# table for maxW
gnuplot -e "col=10" process.gp
# table for diffW
gnuplot -e "col=12" process.gp

# table for n iter time
for arch in CPU_jetson GPU_jetson
do
echo "Generating table/${arch}_time.txt"
    rm -f table/${arch}_time.txt
    for result in $arch/*
    do
        num=${result#*/}
        num=${num#*_}
        n=${num%%_*}
        var=${num#*_}
        var=${var%%_*}
        run=${result: -6}
        run=${run:0:2}
        lastL=$(tail -n 1 $result | awk '{print $2, $4}')
        echo "$n  $var  $run $lastL" >> table/${arch}_time.txt
    done
done

# aggregated table for n iter time
for arch in CPU_jetson GPU_jetson
do
    if [ $arch == "CPU_jetson" ]; then
        range=$(seq 100 100 1000)
    else
        range=$(seq 100 100 1000; seq 1024 1024 15360)
    fi
    rm -f table/${arch}_time_agg.txt
    echo "Generating table/${arch}_time_agg.txt"
    for n in $range
    do
        result=$(awk -v p=$n '$1==p {sIter += $4; sTime += $5} END {print sIter /50, sTime/50}' table/${arch}_time.txt)
        split_results=($result)
        printf "%4d  %6.1f  %7.4f\n" $n ${split_results[0]} ${split_results[1]} >> table/${arch}_time_agg.txt
    done
done
