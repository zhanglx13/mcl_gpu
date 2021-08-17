set t pdfcairo

set o "fig/component_timing.pdf"

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

