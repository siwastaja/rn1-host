#!/bin/bash

hostdir="/home/hrst/rn1-host"
prog="/home/hrst/rn1-tools/p.sh"

export LD_LIBRARY_PATH=/opt/softkinetic/DepthSenseSDK/lib
export DEPTHSENSESDK_DIR=/opt/softkinetic/DepthSenseSDK
rm -f ${hostdir}/*.map

while true
do
	touch ${hostdir}/rn1host.lock
	${hostdir}/rn1host
	HOSTRET=$?
	rm -f ${hostdir}/rn1host.lock

	echo "rn1host returned ${HOSTRET}"

	case "$HOSTRET" in
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

	*)	echo "Restarting rn1host..."
		;;
	esac
done
