set t pdfcairo

set o "fig/jetson_component_timing.pdf"

set logscale x
set xrange[200:42000]
set key top left
plot 10 w l lw 1 lc rgb "red" t "msg period", \
     "jetson_gpu_timing.txt" u 1:3 w l lw 3 lc rgb "blue" t "update-gpu", \
     "jetson_gpu_timing.txt" u 1:6 w l lw 3 lc rgb "purple" t "total-gpu",\
     "jetson_cpu_mt5_timing.txt" u 1:3 w l lw 3 lc rgb "brown" t "update-cpu5", \
     "jetson_cpu_mt5_timing.txt" u 1:6 w l lw 3 lc rgb "black" t "total-cpu5",\
     "jetson_gpu_timing.txt" u 1:2 w l lc rgb "green" t "resampling", \
     "jetson_gpu_timing.txt" u 1:5 w l lc rgb "orange" t "expect", \
     "jetson_cpu_mt5_timing.txt" u 1:2 w l lc rgb "green" t "", \
     "jetson_cpu_mt5_timing.txt" u 1:5 w l lc rgb "orange" t ""

