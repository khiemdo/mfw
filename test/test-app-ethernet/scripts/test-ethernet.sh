#!/bin/bash
red='\e[0;31m'
blue='\e[0;34m'
green='\e[0;32m'
clean='\e[0m' # No Color

# IP configs
DEST_IP=192.168.2.42
DEST_PORT_DATA=8080
DEST_PORT_CONF=8081
# script configs/variables
CONF_PORT_LOG_FILE=log_port_udp_conf.txt
DATA_PORT_RX_LOG_FILE=log_port_udp_data_rx.txt
DATA_PORT_TX_LOG_FILE=log_port_udp_data_tx.txt
TARGET_RESPONSE=
TARGET_STAT_RX_BYTES=
TARGET_STAT_TX_BYTES=
TARGET_STAT_TX_ERROR=
TX_DATA_BLOCK_SIZE=$1
TX_DATA_REPETITIONS=$2

# some usefull functions

# @brief Checks if embedded target is available using PING command
# @param none
function ping_target # param none
{
	echo -n -e "Check connection using ping...\t"
	ping -c 1 $DEST_IP > /dev/null 2>&1
	if [ $? -ne 0 ]; then
		echo -e "${red}failed! Check IP and port settings.${clean}"
		exit 1;
	fi
	echo -e "${green}OK.${clean}"
}

# @brief Sends a command to the embedded target
# @param $1: command string to be sent
function send_command
{
	echo -n -e "Sending command to target: $1 \t"
	echo -n -e "$1\r\n" | nc.traditional -u -q 0 $DEST_IP $DEST_PORT_CONF > /dev/null 2>&1
	if [ $? -ne 0 ]; then
		echo -e "${red}nc failed with code $?.${clean}"
		exit 1;
	fi

    TARGET_RESPONSE=(`tail -n 1 $CONF_PORT_LOG_FILE`)
}

# @brief Sends the command CLEAR statistics to the embedded target and handles its response
# @param none
function send_command_statistic_clear
{
	send_command "ETH_BENCHMARK_COMMAND_STATISTIC_CLEAR"
	local RESP=${TARGET_RESPONSE[@]}
    echo -e "${green}${RESP}${clean}"
#    if [ "${resp}" != "OK." ]; then
#		echo -e "${red}failed sending command ETH_BENCHMARK_COMMAND_STATISTIC_CLEAR.${clean}"
#    	exit 1;
#    fi
#    echo -e "${green}OK.${clean}"
}

# @brief Sends the command GET statistics to the embedded target and handles its response
# @param none
function send_command_statistic_get
{
	send_command "ETH_BENCHMARK_COMMAND_STATISTIC_GET"
	TARGET_STAT_RX_BYTES=${TARGET_RESPONSE[1]}
	TARGET_STAT_TX_BYTES=${TARGET_RESPONSE[3]}
	TARGET_STAT_TX_ERROR=${TARGET_RESPONSE[5]}
    # TODO: add some checks here before stating OK
    echo -e "${green}OK.${clean}"
}

# @brief Checks if embedded target is available using PING command
# @param $1: block-size of one data block to be sent
# @param $2: number of repetitions the data block is sent
function send_data
{
	# start listener on data port to check the amount of received (a.k.a. echoed) data
    # listener writes to log file and terminates when data sending is finished
    # Use following line for logging into file (-v -v writes sent/rcvd bytes to stderr when port is closed):
    # nc.traditional -l -v -v -n -u -p $DEST_PORT_DATA > $DATA_PORT_RX_LOG_FILE 2>&1 &
    nc.traditional -l -n -u -p $DEST_PORT_DATA > /dev/null 2>&1 &

    #dd if=/dev/zero bs=$TX_DATA_BLOCK_SIZE count=$TX_DATA_REPETITIONS status=none | nc.traditional -n -u -q 0 $DEST_IP $DEST_PORT_DATA > /dev/null 2>&1 # 2> $DATA_PORT_TX_LOG_FILE
    dd if=/dev/zero bs=$TX_DATA_BLOCK_SIZE count=1 status=none | nc.traditional -n -u -q 0 $DEST_IP $DEST_PORT_DATA > /dev/null 2>&1 # 2> $DATA_PORT_TX_LOG_FILE
	if [ $? -ne 0 ]; then
		echo -e "${red}nc failed with code $?.${clean}"
		exit 1;
	fi
}


# ---- start of script ----

echo -e "${green}=============================${clean}"
echo -e "${green}HIGHWIND Ethernet test script${clean}"
echo -e "${green}=============================${clean}"
echo -e ""

# Check for nc before using it
command nc > /dev/null 2>&1
if [ $? -ne 1 ]; then
   echo -e "${red}This script requires netcat (nc). Install it before running.${clean}"
fi

echo -e "${green}Test config:${clean}"
echo -e "Destination IP:       \t$DEST_IP"
echo -e "Destination Port Data:\t$DEST_PORT_DATA"
echo -e "Destination Port Conf:\t$DEST_PORT_CONF"

# first check if target is available using PING
ping_target

echo -e ""
echo -e "${green}Test run:${clean}"

# create log file to store all target responses received on conf port
touch $CONF_PORT_LOG_FILE
touch $DATA_PORT_RX_LOG_FILE
touch $DATA_PORT_TX_LOG_FILE

# start linstener for conf port
nc.traditional -u -l -p $DEST_PORT_CONF > $CONF_PORT_LOG_FILE &

# clear statistics on target before starting any tests
send_command_statistic_clear

echo -n -e "Start data transmission to target...\t"

START=$(date +%s%3N)
for i in `seq 1 $TX_DATA_REPETITIONS`;
do
    send_data $TX_DATA_BLOCK_SIZE
done
END=$(date +%s%3N)
echo -e "${green}OK.${clean}"

sleep 0.5

send_command_statistic_get


DIFF=$(echo "$END - $START" | bc)


DATA_SENT_TOTAL=$(echo "$TX_DATA_BLOCK_SIZE * $TX_DATA_REPETITIONS" | bc)
DATA_SPEED_BYTE=$(echo "$DATA_SENT_TOTAL / $DIFF * 1000" | bc)
DATA_SPEED_BIT=$(echo "$DATA_SPEED_BYTE * 8" | bc)
DATA_SPEED_MBIT=$(echo "$DATA_SPEED_BIT / 1000" | bc)
TIME_TX_BLOCK=$(echo "scale=3; $DIFF / $TX_DATA_REPETITIONS" | bc)

echo -e ""
echo -e "${green}Test Result:${clean}"
echo "Tx time start: $START"
echo "Tx time end:   $END"
echo "Tx time (ms):  $DIFF"
echo "Tx time per block (ms): $TIME_TX_BLOCK"
printf "[HOST] sent, rcvd bytes: %'d \t n/a\n" $DATA_SENT_TOTAL
printf "[TRGT] rcvd, sent bytes: %'d \t %'d\n" $TARGET_STAT_RX_BYTES $TARGET_STAT_TX_BYTES
printf "Data speed (mbit): %'d" $DATA_SPEED_MBIT

# kill all port listeners we started at the beginning
kill $(ps aux | grep '[n]c.traditional' | awk '{print $2}')
