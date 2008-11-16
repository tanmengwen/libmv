#include "mv.h"

namespace mv {

static void five_point_camera_pencil_2(const mat &points, mat *A, mat *B)
{
	mat H = normalizing_homography(points);

	mat design(3,5);
	design = dot(H, points);

	vec v1, v2;
	nullspace(design, v1, v2);
	assert(v1.size() == 5);

	mat five_points(subrange(design, 0,3, 0,4));

	v1.resize(4);
	mat tmpA = five_points*v1;

	v2.resize(4);
	mat tmpB = five_points*v2;

	mat Hinv = inv(H);
	*A = dot(Hinv, tmpA);
	*B = dot(Hinv, tmpB);
}

static vec calc_x6_from_design_mat(double a, double b, double c, double d, double e)
{
	/* This should match the matrix in step 6 above, equation (9) in [1] */
	mat X6null(6,4);
	X6null(0,0) = e-d;   /* first row */
	X6null(0,1) =  0 ;
	X6null(0,2) =  0 ;
	X6null(0,3) = a-b;

	X6null(1,0) = e-c;   /* second row */
	X6null(1,1) =  0 ;
	X6null(1,2) =  a ;
	X6null(1,3) =  0 ;

	X6null(2,0) = d-c;   /* third row */
	X6null(2,1) =  b ;
	X6null(2,2) =  0 ;
	X6null(2,3) =  0 ;

	X6null(3,0) =  0 ;   /* fourth row */
	X6null(3,1) =  e ;
	X6null(3,2) =  0 ;
	X6null(3,3) = a-c;

	X6null(4,0) =  0 ;   /* fifth row */
	X6null(4,1) = e-b;
	X6null(4,2) = a-d;
	X6null(4,3) =  0 ;

	X6null(5,0) =  0 ;   /* sixth row */
	X6null(5,1) =  0 ;
	X6null(5,2) =  d ;
	X6null(5,3) = b-c;

	/* Determine the position of the 6th world point X6 */
	vec Xp;
	nullspace(X6null, Xp);

	return Xp;
}

/* See paragraph after equation 16 in torr97robust for the equation which I
 * used to derive the following coefficients. */
#define accum_cubic_coeffs(x,y,z, sgn) \
p = t1[x]*t1[y]; \
q = t2[x]*t1[y] + t1[x]*t2[y]; \
d += sgn *  p*t1[z]; \
c += sgn * (q*t1[z] + p*t2[z]); \
b += sgn * (t2[x]*t2[y]*t1[z] + q*t2[z]); \
a += sgn *  t2[x]*t2[y]*t2[z];

ProjectiveReconstruction *six_point_n_view(mat &points)
{
	std::vector<ProjectiveReconstruction*> reconstructions;

	mat H = normalizing_homography(points);
	mat Hinv = inv(H);
	mat normalized_points = dot(H, points);

	size_t nviews = points.size2() / 6;

	/* See equation (7.2) p179 of HZ; this is the DLT for solving cameras. */
	/* Chose wi = 1, i.e. the homogenous component of each image
	 * location is 1. */
	mat *As = new mat[nviews];
	mat *Bs = new mat[nviews];
	mat ws(nviews,5);

	for (size_t i=0; i<nviews; i++) {

		/* Extract pencil of camera matricies */
		submat fivepoints(normalized_points, range(0,3), range(6*i,6*i+5));
		five_point_camera_pencil_2(fivepoints, As+i, Bs+i);

		/* Calculate Q */
		vec x6(column(normalized_points,6*i+5));
		mat x6cross = crossmat(x6);
		mat tmp = dot(transpose(As[i]), x6cross);
		mat Qa = dot(tmp, Bs[i]);
		mat Q = Qa + transpose(Qa);

		/* Read the coefficients w^i from Q and put into the ws matrix */
		ws(i,0) = Q(0,1);
		ws(i,1) = Q(0,2);
		ws(i,2) = Q(1,2);
		ws(i,3) = Q(1,3);
		ws(i,4) = Q(2,3);
	}

	/* Find a basis for the null space of ws */
	vec t1, t2;
	nullspace(ws, t1, t2);

	/* The svd gives us the basis for the nullspace of ws in which the t
	 * vector lives, such that t = beta*t1+alpha*t2. However, there is a
	 * cubic constraint on the elements of t, such that we can substitute
	 * and solve for alpha. See equation (10) in [1] */
	double a,b,c,d,p,q;
	a=b=c=d=0;
	accum_cubic_coeffs(0,1,3,  1);
	accum_cubic_coeffs(0,1,4, -1);
	accum_cubic_coeffs(0,2,4,  1);
	accum_cubic_coeffs(0,3,4, -1);
	accum_cubic_coeffs(1,2,3, -1);
	accum_cubic_coeffs(1,3,4,  1);

	/* Assume beta = 1, hope for the best FIXME */
	/* If a=b=c=0 and d!=0, then alpha=0; FIXME find beta instead */
	double a1 = b/a, b1 = c/a, c1 = d/a;
	a = a1;
	b = b1;
	c = c1;

	double alpha, alphas[3];
	size_t nroots = gsl_poly_solve_cubic(a,b,c, alphas+0,alphas+1,alphas+2);

	/* Check each solution for alpha */
	for (size_t ia=0; ia<nroots; ia++) {
		dprintf("#################### Alpha%d ###################\n",ia);
		alpha = alphas[ia];

		double e;
		a = t1[0] + alpha*t2[0];
		b = t1[1] + alpha*t2[1];
		c = t1[2] + alpha*t2[2];
		d = t1[3] + alpha*t2[3];
		e = t1[4] + alpha*t2[4];

		/* Add a new ProjectiveReconstruction */
		int n = reconstructions.size();
		reconstructions.push_back(new ProjectiveReconstruction);
		ProjectiveReconstruction &pr = *reconstructions[n];

		/* Push the world position for the first five points */
		PointStructure *Es[5];
		for (size_t i=0; i<4; i++) {
			Es[i] = new PointStructure;
			(*(Es[i]))[i] = 1.0;
		}
		Es[4] = new PointStructure;
		fill(*(Es[4]), 1.0);

		/* Find X6 from the chi vector and add to the reconstruction */
		PointStructure *Xp = new PointStructure;
		Xp->assign(calc_x6_from_design_mat(a,b,c,d,e));

		/* Push structure into new reconstruction */
		for (size_t j=0; j<5; j++) 
			pr.push_structure(Es[j]);
		pr.push_structure(Xp);

		/* Find good values of u,v for each camera */
		for (size_t i=0; i<nviews; i++) {
			dprintf("  ##------------------ i=%d -------------------\n",i);

			/* Project X6 with A and B */
			/* DO NOT NORMALIZE, it breaks the next step */
			vec AX = dot(As[i], *Xp);
			vec BX = dot(Bs[i], *Xp);

			/* Find mu and nu with smallest algebraic error; see step 7*/
			mat M(3,3);
			for (size_t j=0; j<3; j++) {
				M(j,0) = AX[j];
				M(j,1) = BX[j];
				M(j,2) = normalized_points(j,6*i+5);
			}

			vec munu;
			nullspace(M, munu);
			double mu = munu[0];
			double nu = munu[1];

			ProjectiveCamera *P = new ProjectiveCamera;
			mat tmp = mu*As[i] + nu*Bs[i];
			P->assign(dot(Hinv, tmp));

			/* Store camera and visibility */
			pr.push_camera(P);

			/* Store actual measurements */
			for (size_t j=0; j<5; j++)  {
				pr.push_measurement(P, Es[j],
						new PointFeature(points(0, 6*i+j), points(1, 6*i+j)));
			}
			pr.push_measurement(P, Xp,
					new PointFeature(points(0, 6*i+5), points(1, 6*i+5)));

			//			for (j=0; j<5; j++) 
			//				printf("repro error %d=%g\n",j, P->reprojection_error(Es[j],tracks[j][i]));
			//			printf("repro error 5=%g\n",P->reprojection_error(Xp,tracks[5][i]));

		}
	}

	// Once there is more than 3 cameras, usually of the three solutions for
	// alpha only one of them has reasonable reprojection error.
	double min_rms = HUGE_VAL;
	size_t best;
	for (size_t i=0; i<reconstructions.size(); i++) {
		double rms = reconstructions[i]->rms_reprojection_error();
		if (min_rms > rms) {
			min_rms = rms;
			best = i;
		}
	}
	delete [] As;
	delete [] Bs;
	for (size_t i=0; i<  nroots; i++) {
		if (i != best) {
			reconstructions[i]->delete_contents();
			delete reconstructions[i];
		}
	}

	return reconstructions[best];
}

} // namespace mv
