# to plot this file, enter:
#    load 'JaguarBackwardsData.p'
# at the gnuplot> prompt
#
#
set terminal png
set output "JaguarDataBackwards.png"
set autoscale
set xtic auto
set ytic auto
set title "Jaguar Backward Data: Distance vs. Time"
set xlabel "time [s]"
set ylabel "distance [mm]"
plot "JaguarDataBackwards.dat" using 1:5
