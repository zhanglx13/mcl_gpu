#! /usr/bin/env bash

result_dir=$1

## Convert result_dir to lower case. Require bash 4.0
filename=$(echo "${result_dir,,}")

echo "Processing results in ${result_dir} and writing to ${filename}_timing.txt"

echo "n        res     update   total     expect  MCL" > ${filename}_timing.txt


if [[ "${result_dir}" == "TITAN_GPU" ]]; then
    n_range=`seq 256 256 768; seq 1024 1024 10240; seq 16384 4096 122880`
elif [[ "${result_dir}" == "Jetson_GPU" ]]; then
    n_range=`seq 256 256 768; seq 1024 1024 10240; seq 16384 4096 40960`
elif [[ "${result_dir}" == "TITAN_CPU_MT7" ]] || [[ "${result_dir}" == "TITAN_CPU_1MT7" ]]; then
    n_range=`seq 20 20 200; seq 256 256 3072`
elif [[ "${result_dir}" == "Jetson_CPU_MT5" ]] || [[ "${result_dir}" == "Jetson_CPU_1mt5" ]]; then
    n_range=`seq 20 20 200; seq 256 256 2560`
elif [[ "${result_dir}" == "Jetson_CPU_MT" ]]; then
    n_range=$(echo "256 512 1024 1536 2048 2560 3072")
else
    echo "${result_dir} not recognized"
    exit
fi

for n in ${n_range}
do
    echo "n is $n"
    cnt=0
    acc_res=0
    acc_update=0
    acc_total=0
    acc_expect=0
    acc_MCL=0
    for result in ./${result_dir}/result_${n}_*.txt
    do
        lastL=$(tail -n 1 $result |
        #                      res  update total expect MCL  dis
                    awk '{print $4, $6,    $8,   $11,   $13, $19}')
        lastL_arr=($lastL)
        if (( $(echo "${lastL_arr[5]} < 1" | bc -l ) )); then
            #echo "$result ---> $lastL  YES"
            cnt=$(echo "$cnt + 1.0" | bc)
            acc_res=$(echo "$acc_res + ${lastL_arr[0]}" | bc -l)
            acc_update=$(echo "$acc_update + ${lastL_arr[1]}" | bc -l)
            acc_total=$(echo "$acc_total + ${lastL_arr[2]}" | bc -l)
            acc_expect=$(echo "$acc_expect + ${lastL_arr[3]}" | bc -l)
            acc_MCL=$(echo "$acc_MCL + ${lastL_arr[4]}" | bc -l)
        fi
    done
    res=$(echo "$acc_res / $cnt" | bc -l)
    update=$(echo "$acc_update / $cnt" | bc -l)
    total=$(echo "$acc_total / $cnt" | bc -l)
    expect=$(echo "$acc_expect / $cnt" | bc -l)
    MCL=$(echo "$acc_MCL / $cnt" | bc -l)
    printf "%6d  %7.4f  %7.4f  %7.4f  %7.4f %7.4f\n" $n  $res  $update  $total  $expect  $MCL >> ${filename}_timing.txt
done
