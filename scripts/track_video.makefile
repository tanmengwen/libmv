################################################################################
# No changes past here!
################################################################################

PATH+=${LIBMV}

all : ${BASE}.blender.py

clean-json : 
	rm -f ${BASE}.ts.json
	rm -f ${BASE}_corrected.ts.json
	rm -f ${BASE}.prs.json
	rm -f ${BASE}.pr.json
	rm -f ${BASE}.mr.json
	rm -f ${BASE}.blender.py
	rm -f ${BASE}.kf.json

pgm : ${AVI}.done

${AVI}.done : ${AVI}
	mplayer -vo pnm:pgm ${AVI} -ao none
	mkdir -p ${PGMDIR}
	mv *.pgm ${PGMDIR}
	touch ${AVI}.done

ts : ${BASE}.ts.json

${BASE}.ts.json : ${TRACK_CONFIG} ${AVI}.done
	track_sequence \
		--config=${TRACK_CONFIG} \
		--pattern=${PGMDIR}/${PATTERN} \
		--start=${START_FRAME} \
		--end=${END_FRAME} \
		--num_features=${NUM_FEATURES} \
		--output=${BASE}.ts.json \
		--debug=true
	mkdir -p debug
	mv cairo*.png  debug

corrected : ${BASE}_corrected.ts.json

${BASE}_corrected.ts.json : ${CAMERA_INTRINSICS} ${BASE}.ts.json
	correct_radial_distortion \
		--track=${BASE}.ts.json \
		--intrinsics=${CAMERA_INTRINSICS} \
		--output=${BASE}_corrected.ts.json

keyframes : ${BASE}.kf.json 

${BASE}.kf.json : ${RECONSTRUCTION_TRACK}
	pick_keyframes \
		--track=${RECONSTRUCTION_TRACK} \
		--output=${BASE}.kf.json

prs : ${BASE}.prs.json

${BASE}.prs.json : ${RECONSTRUCTION_TRACK} ${BASE}.kf.json
	reconstruct \
		--track=${RECONSTRUCTION_TRACK} \
		--keyframes_file=${BASE}.kf.json \
		--nview=${RECONSTRUCTION_USE_NVIEW} \
		--ransac_rounds=${RECONSTRUCT_RANSAC_ROUNDS} \
		--output=${BASE}.prs.json

pr : ${BASE}.pr.json 

${BASE}.pr.json : ${RECONSTRUCTION_TRACK} ${BASE}.prs.json
	merge_subsets \
		--track=${RECONSTRUCTION_TRACK} \
		--subsets=${BASE}.prs.json \
		--output=${BASE}.pr.json \
		--bundle_subsets=true \
		--first_subset=${FIRST_SUBSET} \
		--last_subset=${LAST_SUBSET}

resectioned : ${BASE}_resectioned.pr.json 

${BASE}_resectioned.pr.json : ${RECONSTRUCTION_TRACK} ${BASE}.pr.json
	resection \
		--track=${RECONSTRUCTION_TRACK} \
		--reconstruction=${BASE}.pr.json \
		--output=${BASE}_resectioned.pr.json \
		--trim_outliers=${RESECTION_TRIM_OUTLIERS}

mr : ${BASE}.mr.json
metric : ${BASE}.mr.json

${BASE}.mr.json : ${RECONSTRUCTION_TRACK} ${FINAL_PROJECTIVE_RECONSTRUCTION}
	metric \
		--track=${RECONSTRUCTION_TRACK} \
		--reconstruction=${FINAL_PROJECTIVE_RECONSTRUCTION} \
		--intrinsics=${CAMERA_INTRINSICS} \
		--output=${BASE}.mr.json \
		--trim_outliers=${METRIC_TRIM_OUTLIERS} \
		--force_positive=${METRIC_FORCE_POSITIVE}

blender : ${BASE}.blender.py

${BASE}.blender.py :${RECONSTRUCTION_TRACK} ${BASE}.mr.json
	export_blender \
		--track=${RECONSTRUCTION_TRACK} \
		--reconstruction=${BASE}.mr.json \
		--output=${BASE}.blender.py \

evaluation : ${BASE}.eval.json

${BASE}.eval.json : ${RECONSTRUCTION_TRACK} ${BASE}.mr.json
	eval_reconstruction \
		--track=${RECONSTRUCTION_TRACK} \
		--reconstruction=${BASE}.mr.json \
		--ground_truth=${GROUND_TRUTH_PATTERN} \
		--output=${BASE}.eval.json \
		--first=${START_FRAME} \
		--last=${END_FRAME}
