#!/usr/bin/env bash

# Usage: ./clean_data.sh input.txt

input="$1"
full_out="$1_full"
accepted_out="$1_accepted"

# --- 1) Create full file without MONITOR lines ---
grep -v '^MONITOR' "$input" > "$full_out"


# --- 2) Extract last two lines before each MONITOR line ---
awk '
$0 == "MONITOR" {
    if (prev2 != "" && prev1 != "") {
        print prev2
        print prev1
    }
}
{
    prev2 = prev1
    prev1 = $0
}' "$input" > "$accepted_out"
