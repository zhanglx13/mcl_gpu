set t pdfcairo

##
## fig_all_runs_n_colname.pdf
##
## contains all 10 runs when using n particles and init_var=1:5
## each page contains 10 runs for a single init_var value
## Both CPU and GPU results will be plotted
##
## Change colname and col to plot data for different columns in the result*.txt
## maxW --> 10
## diffW --> 12
##
col=col
if (col == 12) {
    colname="diffW"
} else {
    colname="maxW"
}
arch="CPU GPU"
unset key
set grid
set xlabel "iterations"
set ylabel "weight (lg(w*1e63))"
set yrange [0:22]
set ytics 0,2,22
set tics font ",10"

##
## The number of particles are saved in an array as strings
## In this way, we can easily combine different ranges
## Note that we need to do n+0 to convert n from string to int
##
n_arr=""
do for [n=100:1000:100]{
    n_arr = n_arr . n . " "
}

do for [a=1:words(arch)]{
    if (word(arch,a) eq "GPU"){
        do for [n=1024:15360:512]{
            n_arr = n_arr . n . " "
        }
    }
    do for [n in n_arr]{
        set o sprintf("fig/%s_all_runs_%s_%d.pdf", word(arch, a), colname,n+0)
        do for [var=1:5:1]{
            acc=0
            do for [i=1:10:1]{
                stats sprintf("%s/result_%d_%d_%02d.txt", word(arch, a),n+0,var,i) \
                      u (log10(column(col)*1e63)) nooutput
                eval sprintf("ave%d_%d_%d=%f", n+0,var,i, STATS_mean)
                acc = acc + value(sprintf("ave%d_%d_%d", n+0,var,i))
            }
            acc = acc / 10
            sum = 0
            do for [i=1:10:1]{
                x = value(sprintf("ave%d_%d_%d", n+0,var,i))
                sum = sum + (x - acc) * (x - acc)
            }
            stddev = sqrt(sum / 10)
            set title sprintf("n=%d var=%d All Runs ave=%5.2f stddev=%f", n+0, var, acc, stddev)
            plot for [i=1:10:1] sprintf("%s/result_%d_%d_%02d.txt", word(arch, a),n+0,var,i) \
                 u 2:(log10(column(col)*1e63))  w l lw 1
        }
    }
}