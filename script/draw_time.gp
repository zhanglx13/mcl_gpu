set t pdfcairo

set o "fig/jetson_time.pdf"

drawvline(x) = sprintf("set arrow front from %f,0 to %f,10 nohead lw 1", x,x)

eval drawvline(430)
eval drawvline(14000)
eval drawvline(2000)
eval drawvline(4000)
set ytics nomirror
set y2range [0:1]
set xrange [0:16000]
set y2tics 0,0.1,1
set xlabel "Number of particles"
#set y2label "msg lost rate"
set ylabel "elapsed time (ms) per iteration"
plot  10 w l lw 3 lc rgb "red" t "msg period" axis x1y1,\
      "table/GPU_jetson_time_agg.txt" u 1:3 w l lw 3 t "GPU time" axis x1y1, \
      "table/GPU_jetson_time_agg.txt" u 1:((1045.2-$2)/1045.2) w l lw 3 t "GPU lost rate" axis x1y2, \
      "table/CPU_jetson_time_agg.txt" u 1:3 w l lw 3 t "CPU time" axis x1y1, \
      "table/CPU_jetson_time_agg.txt" u 1:((558.2-$2)/558.2) w l lw 3 lc rgb "purple" t "CPU lost rate" axis x1y2
