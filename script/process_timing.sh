#! /usr/bin/env bash

result_dir=jetson_CPU_MT5

#echo "n        res     update   total     expect  MCL" > jetson_gpu_timing.txt
echo "n        res     update   total     expect  MCL" > jetson_cpu_mt5_timing.txt

#for n in `seq 256 256 768; seq 1024 1024 10240; seq 12288 4096 40960`
for n in `seq 256 256 2560`
do
    echo "n is $n"
    cnt=0
    acc_res=0
    acc_update=0
    acc_total=0
    acc_expect=0
    acc_MCL=0
    #for result in ./${result_dir}/result_${n}_*.txt
    for result in ./${result_dir}/result_mt5_${n}_*.txt
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
        #else
        #    echo "$result ---> $lastL  NO"
        fi
    done
    res=$(echo "$acc_res / $cnt" | bc -l)
    update=$(echo "$acc_update / $cnt" | bc -l)
    total=$(echo "$acc_total / $cnt" | bc -l)
    expect=$(echo "$acc_expect / $cnt" | bc -l)
    MCL=$(echo "$acc_MCL / $cnt" | bc -l)
    #printf "%6d  %7.4f  %7.4f  %7.4f  %7.4f %7.4f\n" $n  $res  $update  $total  $expect  $MCL >> jetson_gpu_timing.txt
    printf "%6d  %7.4f  %7.4f  %7.4f  %7.4f %7.4f\n" $n  $res  $update  $total  $expect  $MCL >> jetson_cpu_mt5_timing.txt
done
