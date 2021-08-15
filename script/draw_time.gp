set t pdfcairo

set o "fig/MT_time.pdf"

drawvline(x) = sprintf("set arrow front from %f,0 to %f,10 nohead lw 1", x,x)

lrlt="w l lw 2 lc rgb "

eval drawvline(2700)
eval drawvline(62000)
#eval drawvline(2000)
#eval drawvline(4000)
set ytics nomirror
#set y2range [0:1]
set logscale x
set xrange [100:100000]
set yrange [0:25]
set format x "%.0te%T"
set xtics add ("2.7e3" 2700, "6.2e4" 62000)
set grid
#set y2tics 0,0.1,1
set xlabel "Number of particles"
#set y2label "msg lost rate"
set ylabel "elapsed time (ms) per iteration"
plot  10 w l lw 3 lc rgb "red" t "msg period" axis x1y1,\
      "table/GPU_time_agg.txt" u 1:3 w l lw 3 t "GPU time" axis x1y1, \
      "<awk '$2==1 {print $0}' table/CPU_MT_time_agg.txt" u 1:4 w l lw 3 t "CPU mt=1" axis x1y1,\
      "<awk '$2==2 {print $0}' table/CPU_MT_time_agg.txt" u 1:4 w l lw 3 t "CPU mt=2" axis x1y1,\
      "<awk '$2==4 {print $0}' table/CPU_MT_time_agg.txt" u 1:4 w l lw 3 t "CPU mt=4" axis x1y1,\
      "<awk '$2==6 {print $0}' table/CPU_MT_time_agg.txt" u 1:4 w l lw 3 t "CPU mt=6" axis x1y1,\
      "<awk '$2==8 {print $0}' table/CPU_MT_time_agg.txt" u 1:4 w l lw 3 t "CPU mt=8" axis x1y1
      #"table/GPU_time_agg.txt" u 1:((539.3-$2)/539.3) @lrlt "purple" t "GPU lost rate" axis x1y2, \
      "<awk '$2==8 {print $0}' table/CPU_MT_time_agg.txt" u 1:((531.7-$3)/531.7) @lrlt "brown" t "CPU lost rate mt=8" axis x1y2