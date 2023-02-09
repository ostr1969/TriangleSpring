#!/bin/bash
for f in ../run/OSIM/*;do
	ln $f OSIM/$(basename -- "$f")
done
