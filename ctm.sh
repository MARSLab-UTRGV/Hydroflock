#!/bin/bash

LOG_DIR="./controllers/footbot_hydroflock/controller_logs"

# Function to get the most recently created directory within a given path
get_latest_dir() {
  local dir_path=$LOG_DIR
  local latest_dir=$(ls -td -- "$dir_path"/*/ 2>/dev/null | head -n 1)
  echo "$latest_dir"
}

# Function to process log files within the most recently created directory
process_logs_in_latest_dir() {

  local robot_id=$1

  # Check if the directory exists
  if [ ! -d "$LOG_DIR" ]; then
    echo "The default log directory does not exist. Check shell script for correct path."
    exit 1
  fi

  local latest_dir=$(get_latest_dir)

  # Check if a directory was found
  if [ -z "$latest_dir" ]; then
    echo "No subdirectories found in the provided directory."
    exit 1
  fi

  echo "Using most recently created log directory: $latest_dir"
  # exit 0

  # Find the log file for the specified robot
  local log_file=$(find "$latest_dir" -type f -name "${robot_id}_adr_dev.log")

  # Check if the log file exists
  if [ ! -f "$log_file" ]; then
    echo "Log file for robot $robot_id not found."
    exit 1
  fi

  echo "Processing $log_file..."

  # Use awk to extract blocks starting with "Tick" and ending with "In CalculateTangentialMovement"
  awk '
    BEGIN { RS = "Tick"; FS = "\n" }
    /In CalculateTangentialMovement/ {
      print "Tick" $0
    }
  ' "$log_file"
}

# Function to process log files based on provided arguments
process_logs_with_args() {
  local log_path="$LOG_DIR/$1"
  local robot_id=$2

  # Ensure the provided path exists and is a directory
  if [ ! -d "$log_path" ]; then
    echo "The provided path is not a directory."
    exit 1
  fi

  # Find the log file for the specified robot
  local log_file=$(find "$log_path" -type f -name "${robot_id}_adr_dev.log")

  # Check if the log file exists
  if [ ! -f "$log_file" ]; then
    echo "Log file for robot $robot_id not found."
    exit 1
  fi

  echo "Processing $log_file..."

  # Use awk to extract blocks starting with "Tick" and "In CalculateTangentialMovement"
  awk '
    BEGIN { RS = "Tick"; FS = "\n" }
    /In CalculateTangentialMovement/ {
      print "Tick" $0
    }
  ' "$log_file"
}

# Check the number of arguments
if [ "$#" -eq 0 ]; then
  echo "Usage: $0 <arg1> [<arg2>]"
  exit 1
elif [ "$#" -eq 1 ]; then
  process_logs_in_latest_dir "$1"
elif [ "$#" -eq 2 ]; then
  process_logs_with_args "$1" "$2"
else
  echo "Usage: $0 <arg1> [<arg2>]"
  exit 1
fi
