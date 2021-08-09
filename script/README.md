This folder contains scripts to run the experiment (`run.sh`), draw figures with
gnuplot (`draw.gp`), and process the results to generate tables (`genTab.sh`).
It also contains experiment results (`CPU/` and `GPU/`), figures (`fig/`), 
and tables (`table/`).

# Run experiments

`run.sh` will execute the following command 

``` shell
roslaunch mcl_gpu mcl.launch viz:=0 duration:=5 delay:=3 max_particles:=$n init_var:=$init_var which_impl:=$arch $cpu_threads:=$mt
```

with different combinations of `n`, `init_var`, and `mt` for 10 times. 
`arch` `start`, `step`, `end` are arguments which should be provided when
invoking `run.sh`.

Note that `duration` is set to 5 (default) and `delay` is set to 3. This is to
shorten the time for each experiment.

We are using the default bag settings.
In the future, we might want to parameterize the bag player settings.


# Draw figures

## Weights of `maxW` and `diffW` 

`gnuplot -e 'col=c' draw_error.gp` plots the error of the mcl algorithm.
Figures that are saved in `fig/`.
`c` can be 10 (**maxW**) or 12 (**diffW**). 
Once `c` is decided, the above command will draw figures for `$arch`.

Output files: `fig/$arch8_all_runs_$col_$n.pdf`

- We need to set `$arch` to the folder containing the experiment results.
- `$col` can be "maxW" or "diffW"
- `$n` is the number of particles

Each file has 3 pages, each of which plots `$col` over all iterations for all 10 runs.
Each run has an average, and the average and std dev of the 10 averages are printed
as part of the title of the page.

One thing to note is that it is log10(maxW*1e63) and log10(diffW*1e63) that are plotted
since the original data are too small to compare the averages.
And we only plotted results for `$mt`=8.

## Average and stddev of `maxW` and `diffW`

When `-e "col=c"` is not specified, the average and stddev of `maxW` and `diffW`
are plotted.
This requires `table/$arch8_maxW.txt` and `table/$arch8_diffW.txt` to be generated 
(see section below).

Output files: `fig/avg_stddev_MT8_maxW.pdf` and `fig/avg_stddev_MT8_diffW.pdf`

Each file contains two pages, one plots average and the other plots stddev.
These figures are supposed to show that the more particles, the higher the
average and the lower the stddev until a threshold number of particles is
reached.
However, this is not true for the CPU implementation, in which the average
drops and stddev increases after around 1000 particles.
Because when more particles are used (more than 1000), the interval for each 
iteration gets too large so that the algorithm cannot keep up with the robot.

## Usage

1. We first need to generate the tables for `maxW`, `diffW`, and `interval`.
2. Change `GPUfilename` and `CPUfilename` accordingly.
3. Change `arch`.
4. Change `n_basic` and `n_arr` accordingly.
5. If needed, change **8** in the output name to use another value of `$mt`.

## Time and msg lost rate

`gnuplot draw_time.gp` plots elapsed time of each iteration of the mcl algorithm
with different numbers of particles and message lost rate, which is calculated by
the max iterations of each run.

Output file: `fig/time.pdf` or `fig/MT_time.pdf`

This figure shows that message lost rate starts to be larger than zero and keeps
growing when the elapsed time is larger than the message period (10 ms).
It also shows that the GPU implementation can afford about 10000 particles while
the CPU implementation can only support about 450 particles.


# Process results

`genTab.sh` will generate 5 tables for each `${arch}`.

The first 3 tables,
`table/$arch8_maxW.txt`, `table/$arch8_diffW.txt`, and `table/$arch8_interval.txt`,
correspond to maxW, diffW, and interval columns in the results.
These columns are first averaged.
Then the averages for all 10 runs are collected to compute the mean and stddev.
At last, the mean and stddev for each $init_var are printed on the same line 
for each $n.

For multi-threading settings, we only consider when `$mt=8`, since this generates
the best result on the TITAN machine.

These tables show that the more particles and the less the init_var, the better.

The fourth table, `table/$arch_time.txt`, 
contains max iter and time information. extracted from the last
line of each result.

Then this table is aggregated by computing the average for each $n to generate
the last table, i.e. `table/${arch}_time_agg.txt`.
This table aggregates results of all runs and of all values of `${init_var}`. 
For the current set up, we have 10 runs and 3 values of `${init_var}` (1, 3, and 5).
Therefore, we can see 30 as a hard-coded number in `genTab.sh`. 
In the future, we may parameterize this number.
The table shows max iterations and elapsed time for each combination of `$n` and
`$mt`.

## Usage

Go to `process.gp`

1. Set `arch` to the folder(s) that contain(s) the experiment
results.
2. Set `n_basic` and `n_arr` for each `$arch` according to the experiment result
files in the `$arch` folder.
3. Change **8** to another number if you want the result for other `$mt` values.

Go to `genTab.sh`

1. Set `arch` to the folder(s) that contain(s) the experiment.
2. Set `n_range` and `mt_range` according to the experiment result files
3. Change **30** to another number according to the number of `run`s and number 
of `$mt` values.
