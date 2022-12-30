#!/bin/bash

which idf.py
if [ "$?" != "0" ]; then
    echo "idf.py not found. Be sure to run '. ~/esp/esp-idf/export.sh'"
    exit -1
fi

idf.py  build
