#!/bin/zsh

# This script runs a demo_slam command a bunch of time and check divergence
# and average robot innovation
#
# Example of use:
# ./compare_robustness.sh 1 "$JAFAR_DIR/build_release/modules/rtslam/demo_suite/x86_64-linux-gnu/demo_slam --robot=1 --camera=1 --gps=0 --map=1 --simu=0 --heading=0" /mnt/data/LAAS/data/luma/setup.cfg /mnt/data/LAAS/data/luma/estimation.cfg /mnt/data/LAAS/data/luma/run06 "-0.02 0.08 -0.11" 0.05 100 eval.log
# 
# Author: croussil

threads=$1
# the command must have type of slam parameters
# (--robot --camera --gps --map --simu --heading --data-path)
# but run and display parameters are (overriden)
demoslam_command=( $=2 )
configsetup=$3
configestimation=$4
datapath=$5

# three floats separated by spaces, in meters
final_pos=( $=6 )
# in meters
tolerance=$7

samples=$8
outfile=$9
#max_cor=10000 # in us
#step_cor=250


inter_results=100

# other variables, don't modify
demooptions=( --disp-2d=0 --disp-3d=0 --render-all=0 --replay=1 --dump=0 --log=0 --rand-seed=0 --pause=0 --auto )

# go

zmodload zsh/mathfunc
cp $configsetup /tmp/setup.cfg
cp $configestimation /tmp/estimation.cfg
rm -f /tmp/thread_*.lock >& /dev/null

# for ((i = 0; i <= $max_cor; i+=$step_cor)); do
# 
# 	for ((k = 0; k < 2; k++)); do
# 
# 		cor="0."`printf "%06d" $i`;
# 		if (($k == 1)); then
# 			if (($i == 0)); then
# 				break;
# 			else
# 				cor="-$cor";
# 			fi
# 		fi
# 
# 		echo "### Running cor $cor"
# 
# 		sed -i "s/^IMU_TIMESTAMP_CORRECTION:.*/IMU_TIMESTAMP_CORRECTION: $cor/" /tmp/setup.cfg

		sumin=0.
		sumt=0.
		sum=0

		prev_sumin=0.
		prev_sumt=0.
		prev_sum=0

		while ((`ps aux | grep -E ".*/demo_slam.*--auto.*" | grep -v "grep" | grep -v "zsh -c" | wc -l` > 0)); do sleep 1; done

		if (($threads == 1)); then

			for ((j = 0; j < $samples; j++)); do
				res=`(time $demoslam_command $demooptions --data-path=$datapath \
					--config-setup=/tmp/setup.cfg --config-estimation=/tmp/estimation.cfg) 2>&1`
				score=`echo $res | grep average_robot_innovation | cut -d ' ' -f 2`
				pos=`echo $res | grep final_robot_position | cut -d ' ' -f 2-4`
				time=`echo $res | grep -E ".*user.*system.*cpu.*total$" | sed -r "s/.*( ){2}//" | sed -r "s/s.*//"`
				rseed=`echo $res | grep "Random seed" | cut -d ' ' -f 3`
				apos=( $=pos )
				dist=0.; for ((d=1; d<=3; d++)); do diff=$((${apos[$d]}-(${final_pos[$d]}))); dist=$(($dist+(($diff)*($diff)))); done; dist=$((sqrt($dist)));
				if (($dist < $tolerance)); then ok=1; sum=$(($sum+1)); sumin=$(($sumin+$score)); sumt=$(($sumt+$time)); else ok=0; fi
				echo "Run finishes : score $score\tpos $pos\tdist $dist\tok $ok\ttime $time\trseed $rseed"
				echo "$score\t$pos\t$dist\t$ok\t$time\t$rseed" >> $outfile
			done

		else

			t=0
			got=0
			for ((j = 0; j < $(($samples+$threads)); j++)); do

				# make some checks
				sleep 0.2
				realnt=`ps aux | grep -E ".*/demo_slam.*--auto.*" | grep -v "grep" | grep -v "zsh -c" | wc -l`
				if (($realnt > $threads)); then echo "Error! The script somehow started $realnt threads simultaneously instead of max $threads!"; fi

				# wait that one has finished
				finished=0
				while (($finished == 0)); do
					t=$(($t+1))
					if (($t>=$threads)); then t=0; sleep 0.2; fi
					lockfile -0 -r1 /tmp/thread_$t.lock >& /dev/null && finished=1
				done

				# process its data
				if (($j >= $threads)); then
					res=`cat /tmp/thread_$t.dat`
					score=`echo $res | grep average_robot_innovation | cut -d ' ' -f 2`
					pos=`echo $res | grep final_robot_position | cut -d ' ' -f 2-4`
					time=`echo $res | grep -E ".*user.*system.*cpu.*total$" | sed -r "s/.*( ){2}//" | sed -r "s/s.*//"`
					rseed=`echo $res | grep "Random seed" | cut -d ' ' -f 3`
					apos=( $=pos )
					dist=0.; for ((d=1; d<=3; d++)); do diff=$((${apos[$d]}-(${final_pos[$d]}))); dist=$(($dist+(($diff)*($diff)))); done; dist=$((sqrt($dist)));
					if (($dist < $tolerance)); then ok=1; sum=$(($sum+1)); sumin=$(($sumin+$score)); sumt=$(($sumt+$time)); else ok=0; fi
					echo "Thread $t finishes : score $score\tpos $pos\tdist $dist\tok $ok\ttime $time\trseed $rseed"
					echo "$score\t$pos\t$dist\t$ok\t$time\t$rseed" >> $outfile

					# intermediary results
					got=$(($got+1))
					if (( $(($got % $inter_results)) == 0 )); then
						prev_sum=$(( $sum - $prev_sum ))
						if (( $prev_sum == 0 )); then
							prev_sumin=0.0
							prev_sumt=0.0
						else
							prev_sumin=$(( ($sumin-$prev_sumin)/$prev_sum ))
							prev_sumt=$(( ($sumt-$prev_sumt)/$prev_sum ))
						fi
						sum_perc=$(( ($prev_sum*100.)/$inter_results ))

						echo "#inter_results $inter_results: $sum_perc $prev_sumin $prev_sumt"
						echo "#inter_results $inter_results: $sum_perc $prev_sumin $prev_sumt" >> $outfile

						prev_sumin=$sumin
						prev_sumt=$sumt
						prev_sum=$sum
					fi
				fi

				# start a new one
				if (($j < $samples)); then
					#echo "Thread $t starts"
					zsh -c "(time $demoslam_command $demooptions --data-path=$datapath \
						--config-setup=/tmp/setup.cfg --config-estimation=/tmp/estimation.cfg) >& /tmp/thread_$t.dat ; \
						rm -f /tmp/thread_$t.lock" &
				fi

			done

		fi

		if (( $sum == 0 )); then
			sumin=0.0
			sumt=0.0
		else
			sumin=$(($sumin / $sum))
			sumt=$(($sumt / $sum))
		fi
		sum_perc=$(( ($sum*1.)/$samples ))
		stddev=$(( sqrt(($sum_perc*(1.-$sum_perc)) / $samples)*100. ))
		sum_perc=$(( $sum_perc * 100. ))
		echo "Average percentage of successful runs $sum_perc sigma $stddev"
 		echo "Average innovation for successful runs $sumin"
 		echo "Average time for successful runs $sumt"

		echo "#results: $sum_perc $stddev $sumin $sumt" >> $outfile
		echo "#############\n#setup.cfg" >> $outfile
		cat $configsetup >> $outfile
		echo "#############\n#estimation.cfg" >> $outfile
		cat $configestimation >> $outfile

# 		rm -f /tmp/thread_*.lock >& /dev/null
# 	done
# 
# done






