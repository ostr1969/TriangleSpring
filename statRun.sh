#!/bin/bash
echo $#
if [ ! -z "$1" ];then
	stif=$1
	st=$2
		echo $stif $n
		sed -i "s|predDirectory.*\t|predDirectory b${stif}_$st\t|;
		s|springStif.*\t|springStif $stif\t|;
		s|predGuessFile.*|predGuessFile tpring${st}/bred3_solution.sto|
		s|predDivisions.*\t|predDivisions 50\t|" src/inpdata.txt
		sleep 1
		./predrun
	return

fi
for  stif in 2  ;do
	for st in 0-0G 0.5-40D 1-40C 1.5-40C 2-40F 2.5-40D ;do
		ls -l tpring${st}/bred3_solution.sto
		sed -i "s|predDirectory.*\t|predDirectory b${stif}_$st\t|;
		s|springStif.*\t|springStif $stif\t|;
		s|predGuessFile.*|predGuessFile tpring${st}/bred3_solution.sto|
		s|predDivisions.*\t|predDivisions 50\t|" src/inpdata.txt
		sleep 1
		./predrun
	done
done	
