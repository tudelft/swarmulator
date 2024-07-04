# Bash script to run a simulation multiple times and save all logs in a folder
# This script expects swarmulator to have a kill-switch activated somewhere,
# although manual quitting (pressing the 'q' key) is also fine.

# number of sims per category
nsims=5

for (( k = 1; k <= 8; k++)); do
	nagents=$(($k*5))
	echo $nagents
	for (( i = 1; i <= $nsims; i++ )); do
		name=${nagents}drones_inf_$i
		# Bash text
		echo "Running trial $i out of $nsims for $name"
		# Run code
		./swarmulator $nagents $name
		sleep 1 # Give it some time to close
	done
done