set datafile separator ';'
set xdata time
set timefmt "%Y-%m-%dT%H:%M:%S"

set key autotitle columnhead # use the first line as title
set ylabel "First Y Units" # label for the Y axis
set xlabel 'Time' # label for the X axi

plot `plot.csv` using 1:2 with lines, '' using 2:4 with lines
