set t pdfcairo

lfont="font \",20\""
mfont="font \",15\""

set xlabel "number of particles" @lfont
set ylabel "elapsed time (ms)" @lfont
set tics @mfont


set grid

set o "fig/TITAN_component_timing.pdf"

gpufile="result_timing/titan_gpu_timing.txt"
cpufile="result_timing/titan_cpu_1mt7_timing.txt"
hybridfile="result_timing/titan_hybrid_1mt_min_timing.txt"
set logscale x
set xrange[20:130000]
set yrange [0:12]
set key center left @mfont
plot 10 w l lw 1 lc rgb "black" t "", \
     gpufile u 1:3 w l lw 3 lc rgb "blue" t "update-gpu", \
     gpufile u 1:6 w l lw 3 lc rgb "purple" t "total-gpu",\
     cpufile u 1:3 w l lw 3 lc rgb "brown" t "update-cpu", \
     cpufile u 1:6 w l lw 3 lc rgb "red" t "total-cpu",\
     gpufile u 1:2 w l lc rgb "green" t "resampling", \
     gpufile u 1:5 w l lc rgb "orange" t "expect", \
     cpufile u 1:2 w l lc rgb "green" t "", \
     cpufile u 1:5 w l lc rgb "orange" t ""

set o "fig/TITAN_hybrid_timing.pdf"
set key right bottom @lfont
unset logscale x
plot 10 w l lw 1 lc rgb "black" t "", \
     gpufile u 1:3 w l lw 3 lc rgb "blue" t "gpu", \
     cpufile u 1:3 w l lw 3 lc rgb "green" t "cpu", \
     hybridfile u 1:4 w l lw 3 lc rgb "red" t "hybrid"



set o "fig/jetson_component_timing.pdf"

gpufile="result_timing/jetson_gpu_timing.txt"
cpufile="result_timing/jetson_cpu_1mt5_timing.txt"
hybridfile="result_timing/jetson_hybrid_1mt_min_timing.txt"
set logscale x
set xrange[20:42000]
set key left center @mfont
set title ""
plot 10 w l lw 1 lc rgb "black" t "", \
     gpufile u 1:3 w l lw 3 lc rgb "blue" t "update-gpu", \
     gpufile u 1:6 w l lw 3 lc rgb "purple" t "total-gpu",\
     cpufile u 1:3 w l lw 3 lc rgb "brown" t "update-cpu", \
     cpufile u 1:6 w l lw 3 lc rgb "red" t "total-cpu",\
     gpufile u 1:2 w l lc rgb "green" t "resampling", \
     gpufile u 1:5 w l lc rgb "orange" t "expect", \
     cpufile u 1:2 w l lc rgb "green" t "", \
     cpufile u 1:5 w l lc rgb "orange" t ""

set o "fig/jetson_hybrid_timing.pdf"
set key right bottom @lfont
unset logscale x
plot 10 w l lw 1 lc rgb "black" t "", \
     gpufile u 1:3 w l lw 3 lc rgb "blue" t "gpu", \
     cpufile u 1:3 w l lw 3 lc rgb "green" t "cpu", \
     hybridfile u 1:4 w l lw 3 lc rgb "red" t "hybrid"
