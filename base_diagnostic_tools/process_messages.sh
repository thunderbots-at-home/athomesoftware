#!/bin/bash

OUT_FILE="./distance.csv"

awk 'BEGIN {print "left distance, right distance, time period ms";}
{print $2", "$3", "$5;}' messages.log >> $OUT_FILE
