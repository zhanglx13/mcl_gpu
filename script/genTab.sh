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
        IN=${result#*/result_}
        IN=${IN%.txt}
        ## Now IN only contains info we want
        ## Separate the filename by "_"
        ## Check https://stackoverflow.com/a/918931/4080767
        IFS='_' read -ra item <<< "$IN"
        n=${item[0]}
        var=${item[1]}
        mt=${item[2]}
        run=${item[3]}
        lastL=$(tail -n 1 $result | awk '{print $2, $4}')
        echo "$n  $var  $mt  $run  $lastL" >> table/${arch}_time.txt
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
    mt_range=$(seq 1 1 1; seq 2 2 6)
    rm -f table/${arch}_time_agg.txt
    echo "Generating table/${arch}_time_agg.txt"
    for n in ${n_range}
    do
        for mt in ${mt_range}
        do
            result=$(awk -v p=$n -v t=$mt '$1==p && $3==t {sIter += $5; sTime += $6} END {print sIter /30, sTime/30}' table/${arch}_time.txt)
            split_results=($result)
            printf "%4d  %2d %6.1f  %7.4f\n" $n $mt ${split_results[0]} ${split_results[1]} >> table/${arch}_time_agg.txt
        done
    done
done
