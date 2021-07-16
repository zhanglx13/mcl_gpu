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
    for init_var in 1 3 5
    do
        for mt in 1 2 4 6
        do
            for i in `seq 1 1 10`
            do
                ofilename=$(printf 'result_%d_%s_%d_%02d.txt' $n "$init_var" $mt $i)
                echo "Writing to $ofilename ..."
                roslaunch mcl_gpu mcl.launch viz:=0 delay:=3 max_particles:=$n init_var:=$init_var which_impl:=$arch cpu_threads:=$mt > $ofilename
                sed -n -i '/iter */p' $ofilename
            done
        done
    done
done

sudo ./chpmod.sh 0
