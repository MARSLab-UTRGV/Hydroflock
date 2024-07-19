#!/bin/bash

# Check if any arguments are provided
if [ "$#" -eq 0 ]; then
    # No arguments provided
    argos3 -c experiments/hydroflock_dev.argos
else
    # Check if the first argument is -z
    if [ "$1" == "-z" ]; then
        argos3 -c experiments/hydroflock_dev.argos -z
    else
        echo "Unknown argument: $1"
        echo "Usage: $0 [-z]"
        exit 1
    fi
fi
