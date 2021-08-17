set t pdfcairo


##
## Draw average and stddev of maxW and diffW of all numbers of particles
##
set xlabel "Number of particles"

drawvline(x) = sprintf("set arrow front from %f, graph 0 to %f, graph 1 nohead dt 2 lw 1", x,x)
GPUfilename="sprintf(\"table/GPU_jetson_%s.txt\", word(metric, m))"
CPUfilename="sprintf(\"table/CPU_jetson_%s.txt\", word(metric, m))"

metric="maxW diffW"
do for [m=1:words(metric)]{
    set o sprintf("fig/jetson_avg_stddev_%s.pdf", word(metric, m))
    eval drawvline(2000)
    eval drawvline(4000)
    set ylabel "weight (lg(w*1e63))"
    ##
    ## GPU has data up to 76800. However, CPU has data only around 3072.
    ## It make no sense to draw that much of GPU data as long as we show
    ## the point, i.e. the more the particles, the more stable of the
    ## MCL algorithm.
    ##
    set xrange [0:20000]
    
    set title "Averaged max weight of all runs"
    set key bottom right
    plot  @GPUfilename u 1:2 w l lw 1 lc rgb "red" t "GPU", \
          for [c=2:3:1] @GPUfilename u 1:(column(c*2)) w l lw 1 lc rgb "red" t "", \
          @CPUfilename u 1:2 w l lw 1 lc rgb "blue" t "CPU" ,\
          for [c=2:3:1] @CPUfilename u 1:(column(c*2)) w l lw 1 lc rgb "blue" t ""
    
    set title "stddev of max weight of all runs"
    set ylabel "standard deviation"
    set key top right
    plot  @GPUfilename u 1:3 w l lw 1 lc rgb "red" t "GPU", \
          for [c=2:3:1] @GPUfilename u 1:(column(c*2+1)) w l lw 1 lc rgb "red" t "", \
          @CPUfilename u 1:3 w l lw 1 lc rgb "blue" t "CPU" ,\
          for [c=2:3:1] @CPUfilename u 1:(column(c*2+1)) w l lw 1 lc rgb "blue" t ""
}

##
## If col is not defined, do not draw the following figures to
## save some time.
##
if (!exists("col")) exit

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
arch="CPU_jetson GPU_jetson"
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
n_basic=""
do for [n=100:1000:100]{
    n_basic = n_basic . n . " "
}

do for [a=1:words(arch)]{

    if (word(arch,a) eq "GPU_jetson"){
        ## GPU extra range
        n_arr = n_basic
        do for [n=1024:15360:1024]{
            n_arr = n_arr . n . " "
        }
    } else {
        ## CPU extra range
        n_arr = n_basic
        do for [n=2000:13000:1000]{
            n_arr = n_arr . n . " "
        }
    }

    do for [n in n_arr]{
        set o sprintf("fig/%s_all_runs_%s_%d.pdf", word(arch, a), colname,n+0)
        do for [var=1:3:1]{
            var = varX * 2 - 1
            acc=0
            do for [i=1:10:1]{
                stats sprintf("%s/result_%d_%d_8_%02d.txt", word(arch, a),n+0,var,i) \
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
            plot for [i=1:10:1] sprintf("%s/result_%d_%d_8_%02d.txt", word(arch, a),n+0,var,i) \
                 u 2:(log10(column(col)*1e63))  w l lw 1
        }
    }
}
