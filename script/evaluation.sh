#!/bin/bash

# to specify the testcases
TESTCASE1=ariane133_51
TESTCASE2=ariane133_68
TESTCASE3=bsg_chip
TESTCASE4=mempool_group
TESTCASE5=mempool_tile
TESTCASE6=nvdla
TESTCASE7=cluster

# to specify some parameters
DATA_LIST=("ariane133_51" "ariane133_68" "bsg_chip" "mempool_group" "mempool_tile" "nvdla" "cluster")
INPUT_PATH="../benchmarks/nangate45/Simple_inputs"
OUTPUT_PATH="../benchmarks/nangate45/guide"
INPUT_PATH=$(realpath "$INPUT_PATH")
OUTPUT_PATH=$(realpath "$OUTPUT_PATH")

# to user interface
echo "=================================================================="
echo " Please select the testcase you want to run:"
echo "   1. $TESTCASE1"
echo "   2. $TESTCASE2"
echo "   3. $TESTCASE3"
echo "   4. $TESTCASE4"
echo "   5. $TESTCASE5"
echo "   6. $TESTCASE6"
echo -e "   7. $TESTCASE7\n"
read -p "Enter the index (1-7) to select a data: " TESTCASE_NUM
echo "=================================================================="

# to obtain the testcase user would like to run
case $TESTCASE_NUM in
    1) TESTCASE=$TESTCASE1 ;;
    2) TESTCASE=$TESTCASE2 ;;
    3) TESTCASE=$TESTCASE3 ;;
    4) TESTCASE=$TESTCASE4 ;;
    5) TESTCASE=$TESTCASE5 ;;
    6) TESTCASE=$TESTCASE6 ;;
    7) TESTCASE=$TESTCASE7 ;;
    *) echo -e "Error: Invalid selection. Please check again. Exiting. \n"; exit 1 ;;
esac
echo -e "\nInfo: Runing the testcase $TESTCASE_NUM, processing ..."

if [[ ! -d "evaluation_log" ]]; then
    mkdir "./evaluation_log"
    echo -e "\nInfo: Folder 'evaluation_log' created ..."
else
    echo -e "\nInfo: Folder 'evaluation_log' already exists ..."
fi

if [[ ! -d "debug_log" ]]; then
    mkdir "./debug_log"
    echo -e "\nInfo: Folder 'debug_log' created ..."
else
    echo -e "\nInfo: Folder 'debug_log' already exists ..."
fi

rm ./evaluator
make

if (($TESTCASE_NUM >= 1 && $TESTCASE_NUM <= ${#DATA_LIST[@]})); then
    selected_data=${DATA_LIST[$(($TESTCASE_NUM - 1))]}
    echo -e "The selected benchmark = $selected_data \n"

    mkdir -p "./evaluation_log/$selected_data"
    mkdir -p "./debug_log/$selected_data"

    echo -e "\nInfo: Testcase= $selected_data"

    DEBUG_LOG="./debug_log/$selected_data/${selected_data}_debug.log"
    
    if [ $# -eq 0 ]; then
        GUIDE_FILE="$OUTPUT_PATH/$selected_data/${selected_data}.output"
        LOG_FILE="./evaluation_log/$selected_data/${selected_data}.log"
    else 
        # GUIDE_FILE="$OUTPUT_PATH/$selected_data/${selected_data}_${1}.output"
        GUIDE_FILE="$OUTPUT_PATH/$selected_data/${selected_data}.output"
        LOG_FILE="./evaluation_log/$selected_data/${selected_data}_${1}.log"
    fi

    if [ ! -e $GUIDE_FILE ]; then
        echo -e "\nError: Cannot find the .PR_out file, please check again!"
        echo -e "Info: The .PR_out file path: $GUIDE_FILE\n"
        exit 1
    else
        echo -e "\nInfo: Outputfile = $OUTPUT_PATH/$selected_data/$selected_data.output"
    fi


    ./evaluator "$INPUT_PATH/$selected_data.cap" "$INPUT_PATH/$selected_data.net" "$GUIDE_FILE" "$DEBUG_LOG" |& tee "$LOG_FILE"
else
    echo "Invalid input. Please enter a number between 1 and ${#DATA_LIST[@]}."
fi