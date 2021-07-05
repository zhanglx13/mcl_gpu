##
## Process each result_n_var_run.txt and obtain the following information
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
accval="value(sprintf(\"ave%d_%d_%d\", n,var,i))"
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

arch="CPU GPU"

do for [a=1:words(arch)]{
    set print "-"
    print "Generating table/" . word(arch, a) . "_" . colname . ".txt"
    set print "table/" . word(arch, a) . "_" . colname . ".txt"
    do for [n=100:1000:100]{
        line = sprintf("%4d", n)
        do for [var=1:5:1]{
            acc=0
            do for [i=1:10:1]{
                stats sprintf("%s/result_%d_%d_%02d.txt", word(arch, a),n,var,i) \
                      u @colfun nooutput
                eval sprintf("ave%d_%d_%d=%f", n,var,i, STATS_mean)
                x = @accval
                acc = acc + x #value(sprintf("ave%d_%d_%d", n,var,i))
            }
            acc = acc / 10
            ## Calculate std dev
            sum = 0
            do for [i=1:10:1]{
                x = @accval #value(sprintf("ave%d_%d_%d", n,var,i))
                sum = sum + (x - acc) * (x - acc)
            }
            stddev = sqrt(sum / 10)
            line = line . sprintf("  %5.2f  %f", acc, stddev)
        }
        print line
    }
    ##
    ## more data for GPU
    ## I do not know a better way to combine the two ranges
    ## so I have to do everything again for the new range
    ##
    if (word(arch,a) eq "GPU"){
        do for [n=1024:15360:512]{
            line = sprintf("%4d", n)
            do for [var=1:5:1]{
                acc=0
                do for [i=1:10:1]{
                    stats sprintf("%s/result_%d_%d_%02d.txt", word(arch, a),n,var,i) \
                          u @colfun nooutput
                    eval sprintf("ave%d_%d_%d=%f", n,var,i, STATS_mean)
                    x = @accval
                    acc = acc + x #value(sprintf("ave%d_%d_%d", n,var,i))
                }
                acc = acc / 10
                ## Calculate std dev
                sum = 0
                do for [i=1:10:1]{
                    x = @accval #value(sprintf("ave%d_%d_%d", n,var,i))
                    sum = sum + (x - acc) * (x - acc)
                }
                stddev = sqrt(sum / 10)
                line = line . sprintf("  %5.2f  %f", acc, stddev)
            }
            print line
        }
    }
}