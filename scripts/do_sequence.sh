#!/bin/sh

PAT=~/thesis/movies/blender_test1/%04d.pgm
BASE="big"
FEATS=2000
START=28
END=64
./track_sequence -p $PAT -n $START -N $END -f $FEATS -o $BASE.ts
./reconstruct -t $BASE.ts -o $BASE.ppr
./merge_subsets -t $BASE.ts -s $BASE.ppr -o $BASE.pr
