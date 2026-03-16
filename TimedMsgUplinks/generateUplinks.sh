#!/bin/bash

filename='messages.csv'
count=1

if [ ! -f "$filename" ]; then
    echo "File $filename not found."
    exit 1
fi

while IFS= read -r line
do
    echo "$line" > "message_$count.msg"
    count=$((count + 1))
done < "$filename"

echo "Total .msg files created: $((count-1))"