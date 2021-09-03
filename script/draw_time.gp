set t pdfcairo

set o "fig/TITAN_MT_time.pdf"
lfont="font \",20\""
mfont="font \"Bold,15\""

drawvline(x) = sprintf("set arrow front from %f,0 to %f,10 nohead lw 1", x,x)

lrlt="w l lw 2 lc rgb "

set tics @mfont
set logscale x
set xrange [100:70000]
set yrange [0:25]
set grid
set key @lfont
set xlabel "Number of particles" @lfont
set ylabel "elapsed time (ms)" @lfont
plot  10 w l lw 2 lc rgb "black" t "",\
      "table/GPU_time_agg.txt" u 1:3 w l lw 3 t "GPU" axis x1y1, \
      "<awk '$2==1 {print $0}' table/CPU_MT_time_agg.txt" u 1:4 w l lw 3 t "CPU mt=1" ,\
      "<awk '$2==2 {print $0}' table/CPU_MT_time_agg.txt" u 1:4 w l lw 3 t "CPU mt=2" ,\
      "<awk '$2==4 {print $0}' table/CPU_MT_time_agg.txt" u 1:4 w l lw 3 t "CPU mt=4" ,\
      "<awk '$2==6 {print $0}' table/CPU_MT_time_agg.txt" u 1:4 w l lw 3 t "CPU mt=6" ,\
      "<awk '$2==8 {print $0}' table/CPU_MT_time_agg.txt" u 1:4 w l lw 3 t "CPU mt=8" 

set o "fig/jetson_MT_time.pdf"
set xrange [100: 41000]

plot  10 w l lw 2 lc rgb "black" t "" ,\
      "result_timing/jetson_gpu_timing.txt" u 1:6 w l lw 3 t "GPU" , \
      "<awk '$2==1 {print $0}' table/jetson_cpu_mt_timing.txt" u 1:4 w l lw 3 t "CPU mt=1" ,\
      "<awk '$2==2 {print $0}' table/jetson_cpu_mt_timing.txt" u 1:4 w l lw 3 t "CPU mt=2" ,\
      "<awk '$2==4 {print $0}' table/jetson_cpu_mt_timing.txt" u 1:4 w l lw 3 t "CPU mt=4" ,\
      "<awk '$2==6 {print $0}' table/jetson_cpu_mt_timing.txt" u 1:4 w l lw 3 t "CPU mt=6" 
