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
  local fname=$2

  # Check if the directory exists
  if [ ! -d "$LOG_DIR" ]; then
    echo "The default log directory does not exist."
    echo -e "Default log directory: \033[0;35m$LOG_DIR\033[0m"
    exit 1
  fi

  local latest_dir=$(get_latest_dir)

  # Check if a directory was found
  if [ -z "$latest_dir" ]; then
    echo "No subdirectories found in the provided directory."
    echo -e "Make sure you are logging data correctly."
    exit 1
  fi

  echo -e "Using most recently created log directory: \033[0;35m$LOG_DIR/\033[0m\033[0;36m$(basename $latest_dir)\033[0m"
  # exit 0

  # Find the log file for the specified robot
  local log_file=$(find "$latest_dir" -type f -name "${robot_id}_adr_dev.log")

  # Check if the log file exists
  if [ ! -f "$log_file" ]; then
    echo 
    echo "Log file for robot $robot_id not found."
    echo "Only provide the robot_id (e.g., fb0, fb1), not the full log file name."
    echo "Use ./filter_log.sh -h for help."
    echo "Available log files in the latest directory:\n"
    list_tree "$LOG_DIR" "" true
    echo
    exit 1
  fi

  echo "Processing $log_file..."

  # Check if the function name exists in the log file
  if ! grep -q "In $fname" "$log_file"; then
    echo "No matching entries found for function $fname in log file."
    exit 1
  fi

  # Use awk to extract blocks starting with "Tick" and "In <function_name>"
  awk '
    BEGIN { RS = "Tick"; FS = "\n" }
    $0 ~ /In '"$fname"'/ {
      print "Tick" $0
    }
  ' "$log_file"
}

# Function to process log files based on provided arguments
process_logs_with_args() {
  local subdir="$1"
  local log_path="$LOG_DIR/$subdir"
  local robot_id=$2
  local fname=$3

  # Ensure the provided path exists and is a directory
  if [ ! -d "$log_path" ]; then
    echo "The provided path is not a directory."
    echo "Available log directories:\n"
    list_tree "$LOG_DIR" ""
    echo
    exit 1
  fi

  # Find the log file for the specified robot
  local log_file=$(find "$log_path" -type f -name "${robot_id}_adr_dev.log")

  # Check if the log file exists
  if [ ! -f "$log_file" ]; then
    echo "Log file for robot $robot_id not found."
    echo "Only provide the robot_id (e.g., fb0, fb1), not the full log file name."
    echo "Use ./filter_log.sh -h for help."
    echo "Available log files in the specified directory:\n"
    list_tree "$LOG_DIR" "" true
    echo
    exit 1
  fi

  echo "Processing $log_file..."

  # Check if the function name exists in the log file
  if ! grep -q "In $fname" "$log_file"; then
    echo "No matching entries found for function $fname in log file."
    exit 1
  fi

  # Use awk to extract blocks starting with "Tick" and "In <function_name>"
  awk '
    BEGIN { RS = "Tick"; FS = "\n" }
    $0 ~ /In '"$fname"'/ {
      print "Tick" $0
    }
  ' "$log_file"
}

# Function to list directories in a tree-like format with |- characters
list_tree() {

    local base_dir="$1"
    local indent="$2"
    local show_files="${3:-false}"
    

    if [[ "$show_files" == "false" ]]; then

        local dirs=("$base_dir"/*/)
        local num_dirs=${#dirs[@]}

        echo -e "\033[0;35m$base_dir/\033[0m"

        # List directories
        for ((i = 0; i < num_dirs; i++)); do
            local dir="${dirs[$i]}"
            if [ -d "$dir" ]; then
                if [ $((i + 1)) -eq $num_dirs ]; then
                    echo -e "${indent}└── \033[0;36m$(basename "$dir")\033[0m"
                else
                    echo -e "${indent}├── \033[0;36m$(basename "$dir")\033[0m"
                fi
            fi
        done
    else
        
        local files=("$base_dir/$(basename $(get_latest_dir))"/*)
        local num_files=${#files[@]}

        echo -e "\033[0;35m$base_dir/\033[0m"
        echo -e "${indent}└── \033[0;36m$(basename $(get_latest_dir))\033[0m"
        indent="    "
        # List files
        for ((i = 0; i < num_files; i++)); do
            local file="${files[$i]}"
            if [ -f "$file" ]; then
                if [ $((i + 1)) -eq $num_files ]; then
                    echo -e "${indent}└── \033[0;33m$(basename "$file")\033[0m"
                else
                    echo -e "${indent}├── \033[0;33m$(basename "$file")\033[0m"
                fi
            fi
        done
    fi
}


if [ "$#" -eq 1 ]; then
  if [ "$1" == "-h" ]; then
    echo -e "\nUsage 1: For filtering logs in the most recently created directory\n"
    echo -e "\t$0 \033[0;33m<robot_id>\033[0m \033[0;32m<function_name>\033[0m\n"
    echo -e "Usage 2: For processing logs in a specific directory\n"
    echo -e "\t$0 \033[0;36m<subdir>\033[0m \033[0;33m<robot_id>\033[0m \033[0;32m<function_name>\033[0m\n" 
    echo -e "\nDefault log directory: \033[0;35m$LOG_DIR\033[0m\n"
    echo -e "Available log directories:\n"
    list_tree "$LOG_DIR" ""
    echo ""
    echo -e "Latest log directory: \033[0;36m$(basename $(get_latest_dir))\033[0m\n"
    echo -e "Available log files in the latest log directory:\n"
    list_tree "$LOG_DIR" "" true
    echo
    echo -e "The \033[0;33m<robot_id>\033[0m argument should be the robot identifier (e.g., fb0, fb1). Not the full log file name.\n"
    echo -e "The \033[0;32m<function_name>\033[0m argument should be the name of the function to filter in the log file.\n"
    echo -e "\tThis will vary depending on where you called LogThis().\n"
    exit 0
  fi
elif [ "$#" -eq 2 ]; then
  process_logs_in_latest_dir "$1" "$2"
elif [ "$#" -eq 3 ]; then
  process_logs_with_args "$1" "$2" "$3"
else
  echo "Incorrect Usage: $0 <arg1> <arg2> [arg3]"
  echo "Use $0 -h for help."
  exit 1
fi
