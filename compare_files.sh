#!/bin/bash

# some files are not formatted correctly, so I am reformating the whitespace
# to check if the file contents are truely the same (ignoring whitespace), i am using this script
# Usage: ./compare_files.sh file1 file2

if [[ $# -ne 2 ]]; then
  echo "Usage: $0 file1 file2"
  exit 1
fi

strip_comments() {
    local in_block=0
    while IFS= read -r line || [[ -n "$line" ]]; do

        local processed="$line"

        # Remove single-line comments only if not inside block comment
        if [[ $in_block -eq 0 ]]; then
            processed=$(awk '{
                # Remove /*...*/ in line first
                while (match($0, /\/\*.*\*\//)) {
                    $0 = substr($0, 1, RSTART-1) substr($0, RSTART+RLENGTH)
                }
                # Now handle //
                n = match($0, /\/\//)
                if (n > 0) $0 = substr($0, 1, n-1)
                print
            }' <<< "$processed")
        fi

        while [[ $processed == *"/*"* ]] || [[ $in_block -eq 1 ]]; do
            if [[ $in_block -eq 0 ]]; then
                # Start of block comment
                prefix="${processed%%/*\**}"
                suffix="${processed#*/\**}"
                if [[ $suffix == *"*/"* ]]; then
                    # Comment ends on same line
                    before="${processed%%/*\**}"
                    after="${processed#*\*/}"
                    processed="${before}${after}"
                else
                    processed="$prefix"
                    in_block=1
                fi
            else
                # Already inside a block comment
                if [[ $processed == *"*/"* ]]; then
                    processed="${processed#*\*/}"
                    in_block=0
                else
                    processed=""
                    break
                fi
            fi
        done

        # Remove whitespace
        processed=$(echo "$processed" | tr -d '[:space:]')

        # Ignore empty lines
        if [[ -n "$processed" ]]; then
            echo -n "$processed"
        fi
    done < "$1"
    echo
}

tmp1=$(mktemp)
tmp2=$(mktemp)

strip_comments "$1" > "$tmp1"
strip_comments "$2" > "$tmp2"

file1=$(cat "$tmp1")
file2=$(cat "$tmp2")

rm -f "$tmp1" "$tmp2"

len1=${#file1}
len2=${#file2}
maxlen=$(( len1 > len2 ? len1 : len2 ))

if [[ $file1 == "$file2" ]]; then
    echo "FILES ARE THE SAME (by your criteria)"
    exit 0
else
    for ((i=0; i<maxlen; i++)); do
        c1=${file1:$i:1}
        c2=${file2:$i:1}
        if [[ "$c1" != "$c2" ]]; then
            echo "Difference at character $((i+1)):"
            echo "  File1: '${c1:-<none>}'"
            echo "  File2: '${c2:-<none>}'"
            exit 1
        fi
    done
    # In case one file is a prefix of the other
    echo "FILES HAVE DIFFERENT LENGTH"
    exit 1
fi
