#!/bin/bash
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$DIR/../../cudaHOG/
./RealPDT
