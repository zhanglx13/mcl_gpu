#! /usr/bin/env bash

if [ $# -lt 4 ]; then
    echo "Usage: ./run.sh <arch> <start> <step> <end>"
    exit 0
fi

arch=$1
start=$2
step=$3
end=$4

sudo ./chpmod.sh 1

for n in `seq $start $step $end`
do
    for i in `seq 1 1 10`
    do
        ofilename=$(printf 'result_%d_%02d.txt' $n  $i)
        echo "Writing to $ofilename ..."
        roslaunch mcl_gpu mcl.launch viz:=0 delay:=3 \
                  max_particles:=$n which_impl:=$arch cpu_threads:=7 > $ofilename
        sed -n -i '/iter */p' $ofilename
    done
done

sudo ./chpmod.sh 0
