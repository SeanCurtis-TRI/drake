#!/usr/bin/env bash

# Usage: ./extract_steps.sh input.txt > output.txt

awk -F'\t' '
{
    # Save each full line so we can print it later
    line[NR] = $0
    step[NR] = $1          # step_type: full_step / half_step_1 / half_step_2
    t[NR]    = $2 + 0      # numeric value of t (second column)
}
END {
    for (i = 1; i <= NR; i++) {
        # We want pairs:
        #   line i:   half_step_1 t0 ...
        #   line i+1: half_step_2 t1 ...
        if (step[i]   == "half_step_1" &&
            step[i+1] == "half_step_2") {

            # Condition on the *following* line (i+2):
            #   - if it does not exist (EOF), OK
            #   - or if it exists and its t2 > t1, OK
            if (i + 2 > NR || t[i+2] > t[i+1]) {
                print line[i]
                print line[i+1]
            }
        }
    }
}
' "$1"

