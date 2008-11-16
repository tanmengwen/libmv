/*********************************************************************
 * selectGoodFeatures.c
 *
 *********************************************************************/

/* Standard includes */
#include <assert.h>
#include <stdlib.h> /* malloc(), qsort() */
#include <stdio.h>  /* fflush()          */
#include <string.h> /* memset()          */
#include <math.h>   /* fsqrt()           */
#define fsqrt(X) sqrt(X)

/* Our includes */
#include "base.h"
#include "error.h"
#include "convolve.h"
#include "klt.h"
#include "klt_util.h"
#include "pyramid.h"

int KLT_verbose = 1;

typedef enum {SELECTING_ALL, REPLACING_SOME} selectionMode;


/*********************************************************************
 * _quicksort
 * Replacement for qsort().  Computing time is decreased by taking
 * advantage of specific knowledge of our array (that there are 
 * three ints associated with each point).
 *
 * This routine generously provided by 
 *      Manolis Lourakis <lourakis@csi.forth.gr>
 *
 * NOTE: The results of this function may be slightly different from
 * those of qsort().  This is due to the fact that different sort 
 * algorithms have different behaviours when sorting numbers with the 
 * same value: Some leave them in the same relative positions in the 
 * array, while others change their relative positions. For example, 
 * if you have the array [c d b1 a b2] with b1=b2, it may be sorted as 
 * [a b1 b2 c d] or [a b2 b1 c d].
 */

#define SWAP3(list, i, j)               \
{register int *pi, *pj, tmp;            \
     pi=list+3*(i); pj=list+3*(j);      \
                                        \
     tmp=*pi;    \
     *pi++=*pj;  \
     *pj++=tmp;  \
                 \
     tmp=*pi;    \
     *pi++=*pj;  \
     *pj++=tmp;  \
                 \
     tmp=*pi;    \
     *pi=*pj;    \
     *pj=tmp;    \
}

void _quicksort(int *pointlist, int n)
{
  unsigned int i, j, ln, rn;

  while (n > 1)
  {
    SWAP3(pointlist, 0, n/2);
    for (i = 0, j = n; ; )
    {
      do
        --j;
      while (pointlist[3*j+2] < pointlist[2]);
      do
        ++i;
      while (i < j && pointlist[3*i+2] > pointlist[2]);
      if (i >= j)
        break;
      SWAP3(pointlist, i, j);
    }
    SWAP3(pointlist, j, 0);
    ln = j;
    rn = n - ++j;
    if (ln < rn)
    {
      _quicksort(pointlist, ln);
      pointlist += 3*j;
      n = rn;
    }
    else
    {
      _quicksort(pointlist + 3*j, rn);
      n = ln;
    }
  }
}
#undef SWAP3


/*********************************************************************/

static void _fillFeaturemap(
  int x, int y, 
  uchar *featuremap, 
  int mindist, 
  int ncols, 
  int nrows)
{
  int ix, iy;

  for (iy = y - mindist ; iy <= y + mindist ; iy++)
    for (ix = x - mindist ; ix <= x + mindist ; ix++)
      if (ix >= 0 && ix < ncols && iy >= 0 && iy < nrows)
        featuremap[iy*ncols+ix] = 1;
}


/*********************************************************************
 * _enforceMinimumDistance
 *
 * Removes features that are within close proximity to better features.
 *
 * INPUTS
 * featurelist:  A list of features.  The nFeatures property
 *               is used.
 *
 * OUTPUTS
 * featurelist:  Is overwritten.  Nearby "redundant" features are removed.
 *               Writes -1's into the remaining elements.
 *
 * RETURNS
 * The number of remaining features.
 */

static int _enforceMinimumDistance(
		int *pointlist,              /* featurepoints */
		int npoints,                 /* number of featurepoints */
		int max_to_use,              /* max number of features to use */
		KLT_FeatureList featurelist, /* features */
		int ncols, int nrows,        /* size of images */
		int mindist,                 /* min. dist b/w features */
		int min_eigenvalue,          /* min. eigenvalue */
		int top, int left,
		int factorx, int factory,
		int start_index,
		KLT_BOOL overwriteAllFeatures,
		KLT_BOOL ignore_existing_features)
{
	int indx;          /* Index into features */
	int added = 0;     /* How many were successfully added */
	int x, y, val;     /* Location and trackability of pixel under consideration */
	uchar *featuremap; /* Boolean array recording proximity of features */
	int *ptr;

	/* Cannot add features with an eigenvalue less than one */
	if (min_eigenvalue < 1)  min_eigenvalue = 1;

	/* Allocate memory for feature map and clear it */
	featuremap = (uchar *) malloc(ncols * nrows * sizeof(uchar));
	memset(featuremap, 0, ncols*nrows);

	/* Necessary because code below works with (mindist-1) */
	mindist--;

	mindist /= factorx;

	/* If we are keeping all old good features, then add them to the featuremap */
	if (!overwriteAllFeatures && !ignore_existing_features) {
		for (indx = 0 ; indx < featurelist->nFeatures ; indx++) {
			KLT_Feature feat = featurelist->feature[indx];
			if (feat->val >= 0)  {
				x   = (int) (feat->x - left)/factorx;
				y   = (int) (feat->y - top )/factory;
				_fillFeaturemap(x, y, featuremap, mindist, ncols, nrows);
			}
		}
	}

	/* For each feature point, in descending order of importance, do ... */
	ptr = pointlist;
	indx = start_index;
	int last_index;
	while (1)  {
		KLT_Feature feat;

		/* If we can't add all the points, then fill in the rest
		   of the featurelist with -1's */
		if (ptr >= pointlist + 3*npoints || added >= max_to_use)  {
			last_index = indx;
			while (indx < featurelist->nFeatures)  {	
				feat = featurelist->feature[indx];
				if (overwriteAllFeatures
						|| feat->val < 0) {
					feat->x   = -1;
					feat->y   = -1;
					feat->val = KLT_NOT_FOUND;
					feat->aff_img = NULL;
					feat->aff_img_gradx = NULL;
					feat->aff_img_grady = NULL;
					feat->aff_x = -1.0;
					feat->aff_y = -1.0;
					feat->aff_Axx = 1.0;
					feat->aff_Ayx = 0.0;
					feat->aff_Axy = 0.0;
					feat->aff_Ayy = 1.0;
				}
				indx++;
			}
			printf("-- leaving because we passed the end of pointlist.");
			break;
		}

		int orig_x   = *ptr++;
		int orig_y   = *ptr++;
		val = *ptr++;
		x   = (orig_x - left)/factorx;
		y   = (orig_y - top )/factory;

		/* Ensure that feature is in-bounds */
//		assert(x >= 0);
//		assert(x < ncols);
//		assert(y >= 0);
//		assert(y < nrows);
		if (!((x >= 0)
			&& (x < ncols)
			&& (y >= 0)
			&& (y < nrows))) {
			printf("x=%d,y=%d\n",x,y);
			printf("orig_x=%d,orig_y=%d\n",orig_x,orig_y);
		}

		while (!overwriteAllFeatures && 
				indx < featurelist->nFeatures &&
				featurelist->feature[indx]->val >= 0)
			indx++;

		if (indx >= featurelist->nFeatures) {
			last_index = indx;
			printf("--- leaving by running past end of featureList\n");
			break;
		}

		feat = featurelist->feature[indx];

		/* If no neighbor has been selected, and if the minimum
		   eigenvalue is large enough, then add feature to the current list */
		if (!featuremap[y*ncols+x] && val >= min_eigenvalue)  {
			feat->x   = (KLT_locType) orig_x;
			feat->y   = (KLT_locType) orig_y;
			feat->val = (int) val;
			feat->aff_img = NULL;
			feat->aff_img_gradx = NULL;
			feat->aff_img_grady = NULL;
			feat->aff_x = -1.0;
			feat->aff_y = -1.0;
			feat->aff_Axx = 1.0;
			feat->aff_Ayx = 0.0;
			feat->aff_Axy = 0.0;
			feat->aff_Ayy = 1.0;
			indx++;
			added++;

			/* Fill in surrounding region of feature map, but
			   make sure that pixels are in-bounds */
			_fillFeaturemap(x, y, featuremap, mindist, ncols, nrows);
		}
	}

	/* Free feature map  */
	free(featuremap);

	return last_index;
}


/*********************************************************************
 * _comparePoints
 *
 * Used by qsort (in _KLTSelectGoodFeatures) to determine
 * which feature is better.
 * By switching the '>' with the '<', qsort is fooled into sorting 
 * in descending order.
 */

#ifdef KLT_USE_QSORT
static int _comparePoints(const void *a, const void *b)
{
  int v1 = *(((int *) a) + 2);
  int v2 = *(((int *) b) + 2);

  if (v1 > v2)  return(-1);
  else if (v1 < v2)  return(1);
  else return(0);
}
#endif


/*********************************************************************
 * _sortPointList
 */

static void _sortPointList(
  int *pointlist,
  int npoints)
{
#ifdef KLT_USE_QSORT
  qsort(pointlist, npoints, 3*sizeof(int), _comparePoints);
#else
  _quicksort(pointlist, npoints);
#endif
}


/*********************************************************************
 * _minEigenvalue
 *
 * Given the three distinct elements of the symmetric 2x2 matrix
 *                     [gxx gxy]
 *                     [gxy gyy],
 * Returns the minimum eigenvalue of the matrix.  
 */

static float _minEigenvalue(float gxx, float gxy, float gyy)
{
  return (float) ((gxx + gyy - sqrt((gxx - gyy)*(gxx - gyy) + 4*gxy*gxy))/2.0f);
}
	
int limit = ((unsigned int)-1)/2;

static int _max(int a, int b) {
	if (a > b)
		return a;
	return b;
}

static int _min(int a, int b) {
	if (a < b)
		return a;
	return b;
}

static void _calculateMinDets(
		KLT_TrackingContext tc,
		int miny, int maxy,
		int minx, int maxx,
		_KLT_FloatImage gradx,
		_KLT_FloatImage grady,
		int *ptr,
		int *npoints)
{
	register float gx, gy;
	register float gxx, gxy, gyy;
	register int xx, yy;
	int x, y;
	float val;
	int ncols = gradx->ncols;
	int nrows = gradx->nrows;

	(*npoints) = 0;
	assert(gradx);
	assert(grady);

	int window_hw = tc->window_width/2; 
	int window_hh = tc->window_height/2;

	miny = _max(miny, _max(window_hh, tc->bordery));
	minx = _max(minx, _max(window_hw, tc->borderx));

	maxy = _min(maxy, nrows-_max(window_hh, tc->bordery));
	maxx = _min(maxx, ncols-_max(window_hw, tc->borderx));

	printf("bounds minx=%d,maxx=%d,miny=%d,maxy=%d\n",minx,maxx,miny,maxy);

	/* If this is a bin near the edge, it could be that we shouldn't look at any
	 * pixels! */
	if ((miny >= maxy)
			|| (minx >= maxx))
		return;

	for (y = miny; y < maxy; y += tc->nSkippedPixels + 1)
		for (x = minx; x < maxx; x += tc->nSkippedPixels + 1)  {

			/* Sum the gradients in the surrounding window */
			gxx = 0;  gxy = 0;  gyy = 0;
			for (yy = y-window_hh ; yy <= y+window_hh ; yy++)
				for (xx = x-window_hw ; xx <= x+window_hw ; xx++)  {
					gx = *(gradx->data + ncols*yy+xx);
					gy = *(grady->data + ncols*yy+xx);
					gxx += gx * gx;
					gxy += gx * gy;
					gyy += gy * gy;
				}

			/* Store the trackability of the pixel as the minimum
			   of the two eigenvalues */
			*ptr++ = x;
			*ptr++ = y;
			val = _minEigenvalue(gxx, gxy, gyy);
			if (val > limit)  {
				KLTWarning("(_KLTSelectGoodFeatures) minimum eigenvalue %f is "
						"greater than the capacity of an int; setting "
						"to maximum value", val);
				val = (float) limit;
			}
			*ptr++ = (int) val;
			(*npoints)++;
		}
}


/*********************************************************************/

void _KLTSelectGoodFeatures(
		KLT_TrackingContext tc,
		KLT_PixelType *img, 
		int ncols, 
		int nrows,
		KLT_FeatureList featurelist,
		selectionMode mode)
{
	_KLT_FloatImage floatimg, gradx, grady;
	int *pointlist;
	int npoints = 0;
	KLT_BOOL overwriteAllFeatures = (mode == SELECTING_ALL) ?
		TRUE : FALSE;
	KLT_BOOL floatimages_created = FALSE;

	/* Check window size (and correct if necessary) */
	if (tc->window_width % 2 != 1) {
		tc->window_width = tc->window_width+1;
		KLTWarning("Tracking context's window width must be odd.  "
				"Changing to %d.\n", tc->window_width);
	}
	if (tc->window_height % 2 != 1) {
		tc->window_height = tc->window_height+1;
		KLTWarning("Tracking context's window height must be odd.  "
				"Changing to %d.\n", tc->window_height);
	}
	if (tc->window_width < 3) {
		tc->window_width = 3;
		KLTWarning("Tracking context's window width must be at least three.  \n"
				"Changing to %d.\n", tc->window_width);
	}
	if (tc->window_height < 3) {
		tc->window_height = 3;
		KLTWarning("Tracking context's window height must be at least three.  \n"
				"Changing to %d.\n", tc->window_height);
	}

	/* Create pointlist, which is a simplified version of a featurelist, */
	/* for speed.  Contains only integer locations and values. */
	pointlist = (int *) malloc(ncols * nrows * 3 * sizeof(int));

	/* Create temporary images, etc. */
	if (mode == REPLACING_SOME && 
			tc->sequentialMode && tc->pyramid_current != NULL)  {
		floatimg = ((_KLT_Pyramid) tc->pyramid_current)->img[0];
		gradx = ((_KLT_Pyramid) tc->pyramid_current_gradx)->img[0];
		grady = ((_KLT_Pyramid) tc->pyramid_current_grady)->img[0];
		assert(gradx != NULL);
		assert(grady != NULL);
	} else  {
		floatimages_created = TRUE;
		floatimg = _KLTCreateFloatImage(ncols, nrows);
		gradx    = _KLTCreateFloatImage(ncols, nrows);
		grady    = _KLTCreateFloatImage(ncols, nrows);
		if (tc->smoothBeforeSelecting)  {
			_KLT_FloatImage tmpimg;
			tmpimg = _KLTCreateFloatImage(ncols, nrows);
			_KLTToFloatImage(img, ncols, nrows, tmpimg);
			_KLTComputeSmoothedImage(tmpimg, _KLTComputeSmoothSigma(tc), floatimg);
			_KLTFreeFloatImage(tmpimg);
		} else _KLTToFloatImage(img, ncols, nrows, floatimg);

		/* Compute gradient of image in x and y direction */
		_KLTComputeGradients(floatimg, tc->grad_sigma, gradx, grady);
	}

	/* Write internal images */
	if (tc->writeInternalImages)  {
		_KLTWriteFloatImageToPGM(floatimg, "kltimg_sgfrlf.pgm");
		_KLTWriteFloatImageToPGM(gradx, "kltimg_sgfrlf_gx.pgm");
		_KLTWriteFloatImageToPGM(grady, "kltimg_sgfrlf_gy.pgm");
	}

	/* Compute trackability of each image pixel as the minimum
	   of the two eigenvalues of the Z matrix */
	_calculateMinDets(tc, 0, nrows, 0, ncols, gradx, grady, pointlist, &npoints);

	/* Sort the features  */
	_sortPointList(pointlist, npoints);

	/* Check tc->mindist */
	if (tc->mindist < 0)  {
		KLTWarning("(_KLTSelectGoodFeatures) Tracking context field tc->mindist "
				"is negative (%d); setting to zero", tc->mindist);
		tc->mindist = 0;
	}

	/* Enforce minimum distance between features */
	_enforceMinimumDistance(
			pointlist,
			npoints,
			npoints,
			featurelist,
			ncols, nrows,
			tc->mindist,
			tc->min_eigenvalue,
			0,0,1,1, /* bounding box */
			0, /* startindex */
			overwriteAllFeatures,
			0);

	/* Free memory */
	free(pointlist);
	if (floatimages_created)  {
		_KLTFreeFloatImage(floatimg);
		_KLTFreeFloatImage(gradx);
		_KLTFreeFloatImage(grady);
	}
}

static void _fillBucketsWithFeatureCounts(
		KLT_FeatureList featurelist,
		int nrows, int ncols,
		int *buckets,
		int buckets_x,
		int buckets_y)
{
	int i;
	memset(buckets,0,sizeof(int)*buckets_x*buckets_y);
	for (i=0; i<featurelist->nFeatures; i++) {
		KLT_Feature feat = featurelist->feature[i];
		if (feat->val >= 0) {
			int x = buckets_x*feat->x/ncols;
			int y = buckets_y*feat->y/nrows;
			assert(0 <= x);
			assert(x < buckets_x);
			assert(0 <= y);
			assert(y < buckets_y);
			buckets[buckets_x*y+x] += 1;
		}
	}
}

/*********************************************************************
 * _selectGoodFeaturesFromBuckets
 *
 * Given buckets indicating the number of features in that rectangle of the
 * image, find good features to track in the empty buckets, and add them to the
 * feature list 
 * 
 */
static void _selectGoodFeaturesFromBuckets(
		KLT_TrackingContext tc,
		KLT_FeatureList featurelist,
		int *buckets, int buckets_x, int buckets_y)
{
	_KLT_FloatImage gradx, grady;
	int *pointlist;
	int npoints = 0;
	int nrows, ncols;
	int bottom = 0;

	gradx = ((_KLT_Pyramid) tc->pyramid_current_gradx)->img[0];
	grady = ((_KLT_Pyramid) tc->pyramid_current_grady)->img[0];
	ncols = gradx->ncols;
	nrows = gradx->nrows;
	assert(gradx != NULL);
	assert(grady != NULL);

	int bucket_width = ncols/buckets_x;
	int bucket_height = nrows/buckets_y;

	pointlist = malloc(bucket_width * bucket_height * 3 * sizeof(int));

	int empty = 0;
	int i;
	for (i=0; i<buckets_y*buckets_x; i++)
		if (!buckets[i])
			empty++;
	
	if (!empty)
		return;

	int new_per_bucket = (featurelist->nFeatures
			- KLTCountRemainingFeatures(featurelist))/empty;

//	new_per_bucket *= 5; // FIXME fudge factor; without increasing this we miss too many.

	printf("new per bucket=%d\n",new_per_bucket);
	printf("nfeats=%d\n",featurelist->nFeatures);
	printf("remaining=%d\n",KLTCountRemainingFeatures(featurelist));
	printf("empty=%d\n",empty);
	int x,y;
	for (y=0; y<buckets_y; y++) {
		for (x=0; x<buckets_x; x++) {
			printf("in bucket x=%d,y=%d\n",x,y);

			/* Skip buckets that are populated */
			if (buckets[buckets_x*y+x])
				continue;

			_calculateMinDets(tc,
					y*bucket_height, (y+1)*bucket_height,
					x*bucket_width,  (x+1)*bucket_width,
					gradx, grady, pointlist, &npoints);
			printf("Got %d points\n", npoints);

			_sortPointList(pointlist, npoints);

			int max_num_to_add = _min(new_per_bucket, npoints);
			printf("Trying to add tops %d to this bucket\n", max_num_to_add);

			/* Only within the bucket! */
			bottom = _enforceMinimumDistance(
					pointlist,
					npoints,
					max_num_to_add,
					featurelist,
					bucket_width, bucket_height,
					tc->mindist,
					tc->min_eigenvalue,
					y*bucket_height,x*bucket_width,1,1,
					bottom,
//					0,
					0, /* don't overwrite */
					0  /* ignore other features */ );
			printf("bottom=%d\n",bottom);

			/* Used up all the empty spots; bail out */
			if (bottom == featurelist->nFeatures) {
				printf("cleaning up\n");
				goto cleanup;
			}
		}
	}

cleanup:
	free(pointlist);
}

void KLTReplaceLostFeaturesBuckets(
		KLT_TrackingContext tc,
		KLT_FeatureList fl)
{
	_KLT_Pyramid pyr = tc->pyramid_last;
	int ncols = pyr->img[0]->ncols;
	int nrows = pyr->img[0]->nrows;
//	int bx=8,by=8;
//	int bx=11,by=11;
//	KNOB
	int bx=16,by=16;
	int *buckets = malloc(sizeof(int)*bx*by);
	_fillBucketsWithFeatureCounts(fl, nrows, ncols, buckets, bx, by);
	_selectGoodFeaturesFromBuckets(tc, fl, buckets, bx, by);
	free(buckets);
}

/*********************************************************************
 * KLTSelectGoodFeatures
 *
 * Main routine, visible to the outside.  Finds the good features in
 * an image.  
 * 
 * INPUTS
 * tc:	Contains parameters used in computation (size of image,
 *        size of window, min distance b/w features, sigma to compute
 *        image gradients, # of features desired).
 * img:	Pointer to the data of an image (probably unsigned chars).
 * 
 * OUTPUTS
 * features:	List of features.  The member nFeatures is computed.
 */

void KLTSelectGoodFeatures(
  KLT_TrackingContext tc,
  KLT_PixelType *img, 
  int ncols, 
  int nrows,
  KLT_FeatureList fl)
{
  if (KLT_verbose >= 1)  {
    fprintf(stderr,  "(KLT) Selecting the %d best features "
            "from a %d by %d image...  ", fl->nFeatures, ncols, nrows);
    fflush(stderr);
  }

  _KLTSelectGoodFeatures(tc, img, ncols, nrows, 
                         fl, SELECTING_ALL);

  if (KLT_verbose >= 1)  {
    fprintf(stderr,  "\n\t%d features found.\n", 
            KLTCountRemainingFeatures(fl));
    if (tc->writeInternalImages)
      fprintf(stderr,  "\tWrote images to 'kltimg_sgfrlf*.pgm'.\n");
    fflush(stderr);
  }
}


/*********************************************************************
 * KLTReplaceLostFeatures
 *
 * Main routine, visible to the outside.  Replaces the lost features 
 * in an image.  
 * 
 * INPUTS
 * tc:	Contains parameters used in computation (size of image,
 *        size of window, min distance b/w features, sigma to compute
 *        image gradients, # of features desired).
 * img:	Pointer to the data of an image (probably unsigned chars).
 * 
 * OUTPUTS
 * features:	List of features.  The member nFeatures is computed.
 */

void KLTReplaceLostFeatures(
  KLT_TrackingContext tc,
  KLT_PixelType *img, 
  int ncols, 
  int nrows,
  KLT_FeatureList fl)
{
  int nLostFeatures = fl->nFeatures - KLTCountRemainingFeatures(fl);

  if (KLT_verbose >= 1)  {
    fprintf(stderr,  "(KLT) Attempting to replace %d features "
            "in a %d by %d image...  ", nLostFeatures, ncols, nrows);
    fflush(stderr);
  }

  /* If there are any lost features, replace them */
  if (nLostFeatures > 0)
    _KLTSelectGoodFeatures(tc, img, ncols, nrows, 
                           fl, REPLACING_SOME);

  if (KLT_verbose >= 1)  {
    fprintf(stderr,  "\n\t%d features replaced.\n",
            nLostFeatures - fl->nFeatures + KLTCountRemainingFeatures(fl));
    if (tc->writeInternalImages)
      fprintf(stderr,  "\tWrote images to 'kltimg_sgfrlf*.pgm'.\n");
    fflush(stderr);
  }
}


