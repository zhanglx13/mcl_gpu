set t pdfcairo

set o "fig/TITAN_component_timing.pdf"

set logscale x
set xrange[20:130000]
set key top left
plot 10 w l lw 1 lc rgb "red" t "msg period", \
     "gpu_timing.txt" u 1:3 w l lw 3 lc rgb "blue" t "update-gpu", \
     "gpu_timing.txt" u 1:6 w l lw 3 lc rgb "purple" t "total-gpu",\
     "cpu_mt7_timing.txt" u 1:3 w l lw 3 lc rgb "brown" t "update-cpu7", \
     "cpu_mt7_timing.txt" u 1:6 w l lw 3 lc rgb "black" t "total-cpu7",\
     "gpu_timing.txt" u 1:2 w l lc rgb "green" t "resampling", \
     "gpu_timing.txt" u 1:5 w l lc rgb "orange" t "expect", \
     "cpu_mt7_timing.txt" u 1:2 w l lc rgb "green" t "", \
     "cpu_mt7_timing.txt" u 1:5 w l lc rgb "orange" t ""


set o "fig/jetson_component_timing.pdf"

set logscale x
set xrange[20:42000]
set key top left
plot 10 w l lw 1 lc rgb "red" t "msg period", \
     "gpu_jetson_timing.txt" u 1:3 w l lw 3 lc rgb "blue" t "update-gpu", \
     "gpu_jetson_timing.txt" u 1:6 w l lw 3 lc rgb "purple" t "total-gpu",\
     "cpu_jetson_mt5_timing.txt" u 1:3 w l lw 3 lc rgb "brown" t "update-cpu5", \
     "cpu_jetson_mt5_timing.txt" u 1:6 w l lw 3 lc rgb "black" t "total-cpu5",\
     "gpu_jetson_timing.txt" u 1:2 w l lc rgb "green" t "resampling", \
     "gpu_jetson_timing.txt" u 1:5 w l lc rgb "orange" t "expect", \
     "cpu_jetson_mt5_timing.txt" u 1:2 w l lc rgb "green" t "", \
     "cpu_jetson_mt5_timing.txt" u 1:5 w l lc rgb "orange" t "",\
     "hybrid_jetson_min_timing.txt" u 1:4 w l lw 3 lc rgb "grey" t "update-hybrid",\
     "hybrid_jetson_min_timing.txt" u 1:7 w l lw 3 lc rgb "turquoise" t "total-hybrid"


set o "fig/TITAN_jetson_timing.pdf"

set logscale x
set xrange[20:130000]
set key top left
plot 10 w l lw 1 lc rgb "red" t "msg period", \
     "gpu_timing.txt" u 1:6 w l lw 3 lc rgb "blue" t "titan-gpu",\
     "cpu_mt7_timing.txt" u 1:6 w l lw 3 lc rgb "turquoise" t "titan-cpu7",\
     "gpu_jetson_timing.txt" u 1:6 w l lw 3 lc rgb "brown" t "jetson-gpu",\
     "cpu_jetson_mt5_timing.txt" u 1:6 w l lw 3 lc rgb "olive" t "jetson-cpu5",\
     "hybrid_jetson_min_timing.txt" u 1:7 w l lw 3 lc rgb "purple" t "jetson-hybrid"