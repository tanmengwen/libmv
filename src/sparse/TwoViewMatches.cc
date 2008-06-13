#include "TwoViewMatches.h"

///////////////////////////////////
// VXL
#include <vcl_iostream.h>
#include <vcl_vector.h>
#include <vbl/vbl_array_2d.h>
#include <mvl/FMatrixComputeRANSAC.h>
///////////////////////////



void TwoViewMatches::compute()
{
	compute_candidate_matches();
	compute_fundametal_matrixRANSAC();
}

bool TwoViewMatches::is_good_enough()
{
	return robust_matches.size() >= 20;
}

void TwoViewMatches::compute_candidate_matches()
{
	int ni = frame[0]->features.size();
	int nj = frame[1]->features.size();
	
	//compute distance matrix and row and column minima
	vbl_array_2d<double> distanceMatrix(ni,nj);
	int minimaIdxJ[ni];
	int minimaIdxI[nj];
	
	for(int j=0; j<nj; j++)
		minimaIdxI[j] = 0;
	
	for(int i=0; i<ni; i++)
	{
		minimaIdxJ[i] = 0;
		for(int j=0; j<nj; j++)
		{
			distanceMatrix(i,j) = sift_feature_distance(
						frame[0]->features[i], 
						frame[1]->features[j] );
						
			//compute the descriptor of image1 that have minimal distance with descripor i in image0			
			if(distanceMatrix(i,j) < distanceMatrix(i,minimaIdxJ[i]))
				minimaIdxJ[i] = j;
			 
			//compute the descriptor of image0 that have minimal distance with descriptor j in image1
			if(distanceMatrix(i,j) < distanceMatrix(minimaIdxI[j],j))
				minimaIdxI[j] = i;
		}
	}		
				
	//compute candidate matches
	candidate_matches.clear();
	for(int i=0; i<ni; i++)
	{
		if(i == minimaIdxI[minimaIdxJ[i]])
		{
			PointMatch newMatch(i,minimaIdxJ[i]);
			candidate_matches.push_back(newMatch);
		}
	}
}

void TwoViewMatches::compute_fundametal_matrixRANSAC()
{
	vcl_vector<vgl_homg_point_2d<double> > points1;
	vcl_vector<vgl_homg_point_2d<double> > points2;
	
	for(unsigned int m=0; m<candidate_matches.size(); m++)
	{
		SiftFeature &left = frame[0]->features[candidate_matches[m].i0];
		SiftFeature &right= frame[1]->features[candidate_matches[m].i1];
		
		points1.push_back( vgl_homg_point_2d<double>( left.x, left.y ) );
		points2.push_back( vgl_homg_point_2d<double>( right.x, right.y ) );
	}
	
	// Perform the fit using Phil Torr's Robust Sampling Concensus
	FMatrixComputeRANSAC computor(true,2);
	FMatrix f = computor.compute(points1, points2);
	f.set_rank2_using_svd();

	// log
	vcl_cout << "FMatrixComputeRANSAC with rank truncation:\nF = " << f << vcl_endl;
	double d = 0;
	for (unsigned int i = 0; i < points1.size(); ++i)
		d += f.image1_epipolar_distance_squared(points1[i], points2[i]);
	vcl_cout << "Error = " << d/points1.size() << vcl_endl;
	
	
	// keep inliers
	vcl_vector<bool> inliers = computor.get_inliers();
	robust_matches.clear();
	for(unsigned int m=0; m<candidate_matches.size(); m++)
	{
//		vcl_cout << inliers[m] << vcl_endl;
		if(inliers[m])
			robust_matches.push_back(candidate_matches[m]);
	}
}

