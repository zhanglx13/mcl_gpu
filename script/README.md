This folder contains scripts to run the experiment (`run.sh`), draw figures with
gnuplot (`draw.gp`), and process the results to generate tables (`genTab.sh`).
It also contains experiment results (`CPU/` and `GPU/`), figures (`fig/`), 
and tables (`table/`).

# Run experiments

`run.sh` will execute the following command 

``` shell
roslaunch mcl_gpu mcl.launch viz:=0 duration:=10 max_particles:=$n init_var:=$init_var which_impl:=$arch
```

with different combinations of `n` and `init_var` for 10 times. 
`arch` `start`, `step`, `end` are arguments which should be provided when
invoking `run.sh`.

Note that duration is set to 10 and we are using the default bag settings.
In the future, we might want to parameterize the bag player settings.

## Requirements

Remember to compile the repo. 
The experiment results are obtained with

``` shell
commit 718458ada50930a8b56aabeb0afe5c3069b7d3c8
```

from the progress branch.

# Draw figures

`gnuplot -e 'col=c' draw_error.gp` plots the error of the mcl algorithm.
Figures that are saved in `fig/`.
`c` can be 10 (**maxW**) or 12 (**diffW**). 
Once `c` is decided, the above command will draw figures for both CPU and GPU results.

Output files: `fig/$arch_all_runs_$col_$n.pdf`

$arch can be "CPU" or "GPU". 
$col can be "maxW" or "diffW"
$n is the number of particles

Each file has 5 pages, each of which plots $col over all iterations for all 10 runs.
Each run has an average, and the average and std dev of the 10 averages are printed
as part of the title of the page.

One thing to note is that it is log10(maxW*1e63) and log10(diffW*1e63) that are plotted
since the original data are too small to compare the averages.

## Requirements

Results should be saved in `CPU/` and `GPU/`.

# Process results

`genTab.sh` will generate 5 tables for CPU and 5 tables for GPU.

The first 3 tables,
`table/$arch_maxW.txt`, `table/$arch_diffW.txt`, and `table/$arch_interval.txt`,
correspond to maxW, diffW, and interval columns in the results.
These columns are first averaged.
Then the averages for all 10 runs are collected to compute the mean and stddev.
At last, the mean and stddev for each $init_var are printed on the same line 
for each $n.

These tables show that the more particles and the less the init_var, the better.

The fourth table, `table/$arch_time.txt`, 
contains max iter and time information. extracted from the last
line of each result.

Then this table is aggregated by computing the average for each $n to generate
the last table, i.e. `table/$arch_time_agg.txt`.
This table simply shows the elapsed time of CPU and GPU implementation with
different numbers of particles.
