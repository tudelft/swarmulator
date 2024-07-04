# Bash script to run a simulation multiple times and save all logs in a folder
# This script expects swarmulator to have a kill-switch activated somewhere,
# although manual quitting (pressing the 'q' key) is also fine.

# number of sims per category
nsims=5

# simulation arguments (make sure to compile with the correct evaluation set)
nagents=21
name_base=dyn_ekf_evaluation_lim20

for (( i = 1; i <= $nsims; i++ )); do
	# Bash text
	echo "Running trial $i out of $nsims for $name_base"
	name=$name_base$i
	# Run code
	./swarmulator $nagents $name
	sleep 1 # Give it some time to close
done
