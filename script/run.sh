#! /usr/bin/env bash


if [ $# -lt 5 ]; then
    echo "Usage: ./run.sh <arch> <start> <step> <end> <mt>"
    exit 0
fi

arch=$1
start=$2
step=$3
end=$4
mt=$5

##
## experiment for hybrid implementation
##
if [[ "$1" == "hybrid" ]]; then
    for n in `seq 1024 1024 10240; seq 12288 4096 32768`
    do
        for n_cpu in `seq 256 256 1536`
        do
            if (( $(echo "${n_cpu} <= ($n / 2)") )); then
                n_gpu=$(echo "$n - ${n_cpu}" | bc -l)
                for i in `seq 1 1 10`
                do
                    ofilename=$(printf 'result_%d_%d_%02d.txt' $n ${n_cpu} $i)
                    echo "Writing to $ofilename ... n_gpu is ${n_gpu}"
                    roslaunch mcl_gpu mcl.launch viz:=0 delay:=3 cpu_threads:=$mt \
                              max_particles:=$n which_impl:=$arch N_gpu:=${n_gpu} > $ofilename
                    sed -n -i '/iter */p' $ofilename
                done
            fi
        done
    done
    exit
fi

##
## Non-hybrid experiments
##
for n in `seq $start $step $end`
do
    for i in `seq 1 1 10`
    do
        ofilename=$(printf 'result_%d_%02d.txt' $n  $i)
        echo "Writing to $ofilename ..."
        roslaunch mcl_gpu mcl.launch viz:=0 delay:=3 \
                  max_particles:=$n which_impl:=$arch cpu_threads:=$mt > $ofilename
        sed -n -i '/iter */p' $ofilename
    done
done
