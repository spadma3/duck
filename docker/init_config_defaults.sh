#!/bin/bash
set -eu
DATADIR=/data/config
DUCKIESRC=/home/software

if [[ ! -d ${DATADIR}/calibrations ]]; then
    mkdir -p ${DATADIR}/calibrations/kinematics
    mkdir -p ${DATADIR}/calibrations/camera_intrinsic
    mkdir -p ${DATADIR}/calibrations/camera_extrinsic

    cp ${DUCKIESRC}/docker/kinematics_default.yaml ${DATADIR}/calibrations/kinematics/default.yaml

    cp ${DUCKIESRC}/docker/camera_intrinsic_default.yaml ${DATADIR}/calibrations/camera_intrinsic/default.yaml
    sed -i "s/duckiebot/$HOSTNAME/g" ${DATADIR}/calibrations/camera_intrinsic/default.yaml

    cp ${DUCKIESRC}/docker/camera_extrinsic_default.yaml ${DATADIR}/calibrations/camera_extrinsic/default.yaml
fi
