#!/bin/bash

hostdir="/home/hrst/rn1-host"
prog="/home/hrst/rn1-tools/p.sh"

#export LD_LIBRARY_PATH=/opt/softkinetic/DepthSenseSDK/lib
#export DEPTHSENSESDK_DIR=/opt/softkinetic/DepthSenseSDK
rm -f ${hostdir}/*.map

while true
do
	sleep 1
	/home/hrst/rn1-tools/spiprog r
	sleep 4
	mv ${hostdir}/log.3.txt ${hostdir}/log.4.txt
	mv ${hostdir}/log.2.txt ${hostdir}/log.3.txt
	mv ${hostdir}/log.1.txt ${hostdir}/log.2.txt
	mv ${hostdir}/log.txt ${hostdir}/log.1.txt
	touch ${hostdir}/rn1host.lock
	${hostdir}/rn1host > ${hostdir}/log.txt
	HOSTRET=$?
	rm -f ${hostdir}/rn1host.lock

	echo "rn1host returned ${HOSTRET}"

	case "$HOSTRET" in
	135)	echo "Rebooting the computer..."
		shutdown -r now
		;;

	136)	echo "Shutting down the computer..."
		shutdown -h now
		;;

	10)	echo "Running flasher..."
		sleep 1
		$prog
		sleep 5
		echo "Restarting rn1host..."
		;;

	5)	echo "Quitting..."
		break
		;;

	6)	echo "Updating rn1host from github & compiling..."
		cd ${hostdir}
		git pull
		make
		sleep 1
		echo "Restarting rn1host..."
		;;

	7)	echo "Deleting maps & restarting rn1host..."
		rm -f ${hostdir}/*.map
		sleep 1
		;;

	*)	echo "Restarting rn1host..."
		;;
	esac
done
