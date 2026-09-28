#!/bin/bash

awk '
/SUCCESS/ {succ += $1}
/FAIL ERRORCONTROL/ {err += $1}
/FAIL FEASIBILITY/ {feas += $1}
END {
  print "SUCCESS:", succ
  print "FAIL ERRORCONTROL:", err
  print "FAIL FEASIBILITY:", feas
}' $1
