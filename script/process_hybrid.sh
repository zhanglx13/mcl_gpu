#! /usr/bin/env bash

DIR=$1

## Convert DIR to lower case. Require bash 4.0
dir=$(echo "${DIR,,}")

min_timing_file=${dir}_min_timing.txt
timing_file=${dir}_timing.txt

echo "Processing results in ${DIR} and writing to ${min_timing_file} and ${timing_file}"

echo "  n    n_cpu   res     update   total    expect  MCL" > ${min_timing_file}
echo "  n    n_cpu   res     update   total    expect  MCL" > ${timing_file}

if [[ "${DIR}" == "TITAN_hybrid_1mt" ]]; then
    n_range=`seq 1024 1024 10240; seq 12288 4096 122880`
elif [[ "${DIR}" == "jetson_hybrid_1mt" ]]; then
    n_range=`seq 1024 1024 10240; seq 12288 4096 40960`
fi

for n in ${n_range}
do
    min_MCL=20.0
    min_res=0
    min_update=0
    min_total=0
    min_expect=0
    min_cpu=0
    for n_cpu in `seq 256 256 1536`
    do
        if (( $(echo "${n_cpu} <= ($n / 2)") )); then
            n_gpu=$(echo "$n - ${n_cpu}" | bc -l)

            echo "n is $n || n_cpu is ${n_cpu} || n_gpu is ${n_gpu}"
            cnt=0
            acc_res=0
            acc_update=0
            acc_total=0
            acc_expect=0
            acc_MCL=0

            for i in `seq 1 1 10`
            do
                ofilename=$(printf 'result_%d_%d_%02d.txt' $n ${n_cpu} $i)
                #ls $DIR/$ofilename
                lastL=$(tail -n 1 $DIR/$ofilename |
                #                      res  update total expect MCL  dis
                            awk '{print $4, $6,    $8,   $11,   $13, $19}')
                lastL_arr=($lastL)
                if (( $(echo "${lastL_arr[5]} < 1" | bc -l ) )); then
                    cnt=$(echo "$cnt + 1.0" | bc)
                    acc_res=$(echo "$acc_res + ${lastL_arr[0]}" | bc -l)
                    acc_update=$(echo "$acc_update + ${lastL_arr[1]}" | bc -l)
                    acc_total=$(echo "$acc_total + ${lastL_arr[2]}" | bc -l)
                    acc_expect=$(echo "$acc_expect + ${lastL_arr[3]}" | bc -l)
                    acc_MCL=$(echo "$acc_MCL + ${lastL_arr[4]}" | bc -l)
                fi
            done
            ##
            ## Compute the average of all 10 runs
            ##
            res=$(echo "$acc_res / $cnt" | bc -l)
            update=$(echo "$acc_update / $cnt" | bc -l)
            total=$(echo "$acc_total / $cnt" | bc -l)
            expect=$(echo "$acc_expect / $cnt" | bc -l)
            MCL=$(echo "$acc_MCL / $cnt" | bc -l)

            printf "%6d  %4d  %7.4f  %7.4f  %7.4f  %7.4f %7.4f\n" $n ${n_cpu} ${res}  ${update}  ${total}  ${expect}  ${MCL} >> ${timing_file}

            ##
            ## Save the min MCL value
            ##
            if (( $(echo "$MCL < ${min_MCL}" | bc -l) )); then
                min_cpu=${n_cpu}
                min_MCL=$MCL
                min_res=$res
                min_update=$update
                min_total=$total
                min_expect=$expect
            fi
        fi
    done
    printf "%6d  %4d  %7.4f  %7.4f  %7.4f  %7.4f %7.4f\n" $n ${min_cpu} ${min_res}  ${min_update}  ${min_total}  ${min_expect}  ${min_MCL} >> ${min_timing_file}
done
