##
## Process each result_n_var_8_run.txt and obtain the following information
##
## 1. For each single file, compute the average of column `col`
## 2. For each (n, var) pair, comptue the average and std dev of `col`'s  among all runs
## 3. Write the result to table_arch_colname.txt. Each line corresponds to a single n
##
##
## `col` is given as a parameter when this is invoked as: gnuplot -e "col=7" process.gp
## only 7,10,12 are supported.
## arch = CPU or GPU

col=col
colfun="(log10(column(col)*1e63))"
accval="value(sprintf(\"ave%d_%d_%d\", n+0,var,i))"
if (col == 12) {
    colname="diffW"
} else {
    if (col == 10){
        colname="maxW"
    } else {
        if (col == 7) {
            set key autotitle columnhead
            colname="interval"
            colfun="(column(col))"
        } else {
            print "column number not recognized"
            print "10 for maxW; 12 for diffW; and 7 for interval"
            exit gnuplot
        }
    }
}

arch="GPU"

n_basic=""
do for [n=100:1000:100]{
    n_basic = n_basic . n . " "
}

do for [a=1:words(arch)]{

    if (word(arch,a) eq "GPU"){
        ## GPU extra range
        #n_arr = n_basic
        n_arr=""
        do for [n=512:76800:512]{
            n_arr = n_arr . n . " "
        }
    } else {
        ## CPU extra range
        n_arr=""
        do for [n=256:3072:256]{
            n_arr = n_arr . n . " "
        }
    }

    set print "-"
    print "Generating table/" . word(arch, a) . "_" . colname . ".txt"
    set print "table/" . word(arch, a) . "_" . colname . ".txt"
    do for [n in n_arr]{
        line = sprintf("%4d", n+0)
        do for [varX=1:3:1]{
            var = varX * 2 - 1
            acc=0
            do for [i=1:10:1]{
                stats sprintf("%s/result_%d_%d_%02d.txt", word(arch, a),n+0,var,i) \
                      u @colfun nooutput
                eval sprintf("ave%d_%d_%d=%f", n+0,var,i, STATS_mean)
                x = @accval
                acc = acc + x
            }
            acc = acc / 10
            ## Calculate std dev
            sum = 0
            do for [i=1:10:1]{
                x = @accval
                sum = sum + (x - acc) * (x - acc)
            }
            stddev = sqrt(sum / 10)
            line = line . sprintf("  %5.2f  %f", acc, stddev)
        }
        print line
    }
}