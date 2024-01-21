#!/bin/bash
############################################
# envirinement :                           #
# python3                                  #
# GCC >= 9.3.0                             #
# Boost == 1.77.0                          #
# OpenMP >= 4.5                            #
# Bison >= 3.0.4                           #
# zlib >= 1.2.7                            #
# CMake >= 3.1                             #
############################################

# to specify the benchmarks path (can be modified)
BENCHMARK_PATH="../../benchmarks/nangate45"
NTHU_ROUTE_PATH="../"
# change the relative path to absolute path
BENCHMARK_PATH=$(realpath "$BENCHMARK_PATH")
NTHU_ROUTE_PATH=$(realpath "$NTHU_ROUTE_PATH")

# to specify the testcases
TESTCASE1=ariane133_51
TESTCASE2=ariane133_68
TESTCASE3=bsg_chip
TESTCASE4=mempool_group
TESTCASE5=mempool_tile
TESTCASE6=nvdla
TESTCASE7=cluster

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
read -p "-> Your selection (enter 1~7) = " TESTCASE_NUM
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

# to specify the .cap file path
CAP_FILE="$BENCHMARK_PATH/Simple_inputs/$TESTCASE.cap"
if [ -e "$CAP_FILE" ]; then
    echo -e "\nInfo: We have found the .cap file in $CAP_FILE ..."
else
    echo -e "\nError: The .cap file do not exist, please check again!"
    echo -e "-> The required .cap file path: $CAP_FILE\n"
    exit 1
fi

# to specify the .net file path
NET_FILE="$BENCHMARK_PATH/Simple_inputs/$TESTCASE.net"
if [ -e "$NET_FILE" ]; then
    echo -e "\nInfo: We have found the .net file in $NET_FILE ..."
else
    echo -e "\nError: The .net file do not exist, please check again!"
    echo -e "-> The required .net file path: $NET_FILE\n"
    exit 1
fi

# to create the guide folder
GUIDE_PATH="$BENCHMARK_PATH/guide"
mkdir $GUIDE_PATH
GUIDE_FOLDER="$GUIDE_PATH/$TESTCASE"
if [[ ! -d "$GUIDE_FOLDER" ]]; then
    mkdir "$GUIDE_FOLDER"
    echo -e "\nInfo: Folder '$GUIDE_FOLDER' created ..."
else
    echo -e "\nInfo: Folder '$GUIDE_FOLDER' already exists ..."
fi

# to create the .guide file path
if [ $# -eq 0 ]; then
    GUIDE_FILE="$GUIDE_FOLDER/$TESTCASE.output"
    LOG_FILE="$GUIDE_FOLDER/$TESTCASE.log"
else
    GUIDE_FILE="$GUIDE_FOLDER/${TESTCASE}_$1.output"
    LOG_FILE="$GUIDE_FOLDER/${TESTCASE}_$1.log"
fi

# to execute gthe Makefile
echo -e "\nInfo: Runing the Makefile ..."
make -C ${NTHU_ROUTE_PATH}/src clean
make -C ${NTHU_ROUTE_PATH}/src -j 128

# to run the nthu route
echo -e "\nInfo: Running the NTHU Route 2.0 ..."
rm -rf $GUIDE_FILE
echo -e "Running the command: './route -cap $CAP_FILE -net $NET_FILE -output $GUIDE_FILE |& tee $LOG_FILE'"
# ./route -cap $CAP_FILE -net $NET_FILE -output $GUIDE_FILE |& tee $LOG_FILE
./route --cap $CAP_FILE --net $NET_FILE --out $GUIDE_FILE |& tee $LOG_FILE
if [[ -e "$GUIDE_FILE" ]]; then
    echo -e "\nInfo: Finish running the testcase $TESTCASE_NUM, please check the result"
    echo -e "-> The .guide file will be in the $GUIDE_FOLDER\n"
else
    echo -e "\nError: Something wrong while running the testcase $TESTCASE_NUM, please check again!"
    echo -e "-> You can check if there exist .guide file in $GUIDE_FOLDER\n"
fi