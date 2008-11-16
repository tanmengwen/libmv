################################################################################
# No changes past here!
################################################################################
# This script is designed to be copied and pasted into a directory, then edited
# Then it runs through the various stages of reconstruction in a way that it
# can be restarted.
#
# You will definitely have to change these
BASE=hydrant
AVI=${BASE}.avi
CAMERA_INTRINSICS=${HOME}/thesis/movies/CanonSD230.k.json
START_FRAME=1
END_FRAME=200
LIBMV=~/wrk/libmv/

# You might want to change this
NUM_FEATURES=1000
RECONSTRUCT_RANSAC_ROUNDS=50
METRIC_TRIM_OUTLIERS=false
METRIC_FORCE_POSITIVE=true
RECONSTRUCTION_USE_NVIEW=true
FIRST_SUBSET=0
LAST_SUBSET=5
RESECTION_TRIM_OUTLIERS=true
GROUND_TRUTH_PATTERN=/home/keir/thesis/movies/blender_test1/truth_%04d.exr

# Hopefully won't have to change these ones
PATTERN=%08d.pgm
DEBUG_TRACK_PATTERN=debug
PGMDIR=pgms
TRACK=${BASE}.ts.json
TRACK_CONFIG=track_sequence.config.json
DISTORTION_CORRECTED_TRACK=${BASE}_corrected.ts.json
RECONSTRUCTION_TRACK=${DISTORTION_CORRECTED_TRACK}
# Don't do a resection
FINAL_PROJECTIVE_RECONSTRUCTION=${BASE}.pr.json
# OR resection missing frames
FINAL_PROJECTIVE_RECONSTRUCTION=${BASE}_resectioned.pr.json

include ${LIBMV}/scripts/track_video.makefile
