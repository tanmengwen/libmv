#include <testmv.h>
#include "mv.h"

#include "unscented.h"
TEST(unscented_class) {
	double mean = 0.0;
	double sigma2 = 1.0;
	UnscentedTransform ut(&mean, &sigma2, 1, 0.0, 0.9, 2);
	for (size_t i=0; i<ut.samples(); i++) {
		double *sample = ut.sample(i);
		*sample = (*sample)+2;
	}
	ut.compute_transformed_moments();
	VeryClose(2.0, *ut.mean());
	VeryClose(1.0, *ut.covariance());
}

TEST(unscented_class_2d) {
	double mean[] = {0,0};
	double sigma2[] = {1.0,0.0, 0.0, 1.0};
	UnscentedTransform ut(mean, sigma2, 2, 0.0, 0.9, 2);
	for (size_t i=0; i<ut.samples(); i++) {
		double *sample = ut.sample(i);
		sample[0] += 2;
		sample[1] += 1;
	}
	ut.compute_transformed_moments();
	VeryClose(2.0, ut.mean()[0]);
	VeryClose(1.0, ut.mean()[1]);
	VeryClose(1.0, ut.covariance()[0]);
	VeryClose(0.0, ut.covariance()[1]);
	VeryClose(0.0, ut.covariance()[2]);
	VeryClose(1.0, ut.covariance()[3]);
}

TEST(unscented_class_2d) {
	double mean[] = {0,0};
	double sigma2[] = {1.0,0.0, 0.0, 1.0};
	UnscentedTransform ut(mean, sigma2, 2, 0.0, 0.9, 2);
	for (size_t i=0; i<ut.samples(); i++) {
		double *sample = ut.sample(i);
		double t = sample[0];
		sample[0] = 2*sample[1] + 5;
		sample[1] = -t + 3;
	}
	ut.compute_transformed_moments();
	VeryClose(5.0, ut.mean()[0]);
	VeryClose(3.0, ut.mean()[1]);
	VeryClose(4.0, ut.covariance()[0]);
	VeryClose(0.0, ut.covariance()[1]);
	VeryClose(0.0, ut.covariance()[2]);
	VeryClose(1.0, ut.covariance()[3]);
}

TEST(unscented_class_2d_rotated_scaled_in_y) {
	double mean[] = {0,0};
	double sigma2[] = {1.0,0.0, 0.0, 1.0};
	UnscentedTransform ut(mean, sigma2, 2, 0.0, 0.9, 2);
	for (size_t i=0; i<ut.samples(); i++) {
		double *sample = ut.sample(i);
		double x = sample[0];
		double y = sample[1];
		sample[0] =  0.92387953*x + 0.76536686*y;
		sample[1] = -0.38268343*x + 1.84775907*y;
	}
	ut.compute_transformed_moments();
	Close(0.0, ut.mean()[0]);
	Close(0.0, ut.mean()[1]);
	Close(1.43933983, ut.covariance()[0]);
	Close(1.06066017, ut.covariance()[1]);
	Close(1.06066017, ut.covariance()[2]);
	Close(3.56066017, ut.covariance()[3]);
}

TEST(unscented_class_2d_rotated_scaled_in_x) {
	double mean[] = {0,0};
	double sigma2[] = {1.0,0.0, 0.0, 1.0};
	UnscentedTransform ut(mean, sigma2, 2, 0.0, 0.9, 2);
	for (size_t i=0; i<ut.samples(); i++) {
		double *sample = ut.sample(i);
		double x = sample[0];
		double y = sample[1];
		sample[0] =  2.7716386*x +  0.38268343*y;
		sample[1] = -1.1480503*x +  0.92387953*y;
	}
	ut.compute_transformed_moments();
	Close(0.0, ut.mean()[0]);
	Close(0.0, ut.mean()[1]);
	Close( 7.82842712, ut.covariance()[0]);
	Close(-2.82842712, ut.covariance()[1]);
	Close(-2.82842712, ut.covariance()[2]);
	Close( 2.17157288, ut.covariance()[3]);
}

TEST(unscented_class_2d_rotated_scaled_in_x_REVERSED) {
	double mean[] = {0,0};
	double sigma2[] = {7.82842712, -2.82842712, -2.82842712, 2.17157288};
	UnscentedTransform ut(mean, sigma2, 2, 0.0, 0.9, 2);
	for (size_t i=0; i<ut.samples(); i++) {
		double *sample = ut.sample(i);
		double x = sample[0];
		double y = sample[1];
		sample[0] =  0.30795984*x + -0.12756114*y;
		sample[1] =  0.38268343*x +  0.92387953*y;
	}
	ut.compute_transformed_moments();
	Close(0.0, ut.mean()[0]);
	Close(0.0, ut.mean()[1]);
	Close(1.0, ut.covariance()[0]);
	Close(0.0, ut.covariance()[1]);
	Close(0.0, ut.covariance()[2]);
	Close(1.0, ut.covariance()[3]);
}
