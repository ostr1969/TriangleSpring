#!/bin/bash
for  stif in 3 4 5 6;do
	for ang in -15 -20 -25;do
		sed -i "s|predDirectory.*\t|predDirectory tpring$stif$ang\t|;
		s|springStif.*\t|springStif $stif\t|;
		s|springAng.*\t|springAng $ang\t|" src/inpdata.txt
		sleep 1
		./predrun
	done
done	
