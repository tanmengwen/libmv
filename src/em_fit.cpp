#include "em_fit.h"
namespace mv {
#include <cfloat>


double logsum(vec &log_x)
{
	size_t n = log_x.size();
	double alpha = amax(log_x) - log(DBL_MAX)/2.0 + 2*log(n);
	return alpha + log(sum(aexp(log_x - alpha)));
}

#define CONVERGENCE_BOUND 1e-6
#define EM_ITERATIONS 10

double em_fit(const vec &samples, std::vector<Distribution*> &ds, vec &dropped)
{
	size_t ndists = ds.size();
	size_t nsamps = samples.size();

	assert(dropped.size() == samples.size());

	vec Q[ndists];
	for (size_t i=0; i<ndists; i++) {
		Q[i].resize(nsamps);
	}

	vec alpha(ndists);
	fill(alpha, 1./ndists);

	double totallogprob;
	double last_totallogprob = -HUGE_VAL;

//	printf("--------------\n");
//	dpmat(samples);


	for (int it=0; it<EM_ITERATIONS; it++) {

		// E-Step
		//
		// Assuming the distributions are correct, calculate relative
		// weights (``responsibilites'') of each sample
		for (size_t i=0; i<nsamps; i++) {
			double total_prob = 0.;
			if (dropped[i] == 1.0)
				continue;

			// 1. Calculate the probability of the sample under
			//    each distribution
			for (size_t j=0; j<ndists; j++) {
				double pij = ds[j]->probability(samples[i]);
				if (isnan(pij))
					pij = 0.;
				Q[j][i] = alpha(j)*pij;
				total_prob += Q[j][i];
			}

			// While this shouldn't happen if the data and
			// distributions are compatible, nevertheless this
			// check is necessary, otherwise the whole fit can blow
			// up if any data is passed in which is outside all the
			// distributions.
			if (total_prob == 0.) {
				dropped[i] = 1.0;
				continue;
			}

			// 2. Normalize by the total probability under all
			//    distributions
			for (size_t j=0; j<ndists; j++) {
				Q[j][i] /= total_prob;
			}
		}

//		dpmat(Q[0]);
//		dpmat(Q[1]);

		// M-Step
		//
		// Assuming the``responsibilities'' of the previous step are
		// correct, maximize the probability of each distribution by
		// estimating the ML parameters for the weighted samples.
		for (size_t j=0; j<ndists; j++) {
			ds[j]->maximize_likelihood(samples, Q[j]);
		}

		// Calculate the mixing coefficients alpha
		double total_mix = 0.;
		for (size_t j=0; j<ndists; j++) {
			total_mix += sum(Q[j]);
		}
		for (size_t j=0; j<ndists; j++) {
			alpha[j] = sum(Q[j])/total_mix;
		}

		// Calculate total log probability
		vec log_ps(ndists);
		totallogprob = 0.0;
		for (size_t i=0; i<nsamps; i++) {
			double sample = samples[i];
			if (dropped[i] == 1.0)
				continue;
			for (size_t j=0; j<ndists; j++) {
				 log_ps[j] = ds[j]->log_probability(sample)
					 + log(alpha[j]);
			}
			totallogprob += logsum(log_ps);
		}

		double actual_samps = samples.size() - sum(dropped);
//		printf("////////// EM ITERATION\n");
//		printf("it=%3d, tlp=%g\n", it, totallogprob);
//		printf("average lp=%g\n", totallogprob/actual_samps);
//		printf("delta average lp %g\n", -(last_totallogprob - totallogprob)/actual_samps);
//		dpmat(alpha);
//		printf("dropped=%g\n",sum(dropped));
		double avg_delta_lp = -(last_totallogprob - totallogprob)/actual_samps;

//		assert (totallogprob >= last_totallogprob);
		if (totallogprob < last_totallogprob) 
			printf("===================== OOPS!!!! lost ground: %g\n", last_totallogprob - totallogprob);

		if (avg_delta_lp < CONVERGENCE_BOUND) {
//			printf("converged in %d iterations\n",it);
			break;
		}

		last_totallogprob = totallogprob;
	}

//	printf("dropped=======================%g\n",sum(dropped));
	
	// Save mixing coefficients
	for (size_t j=0; j<ndists; j++)
		 ds[j]->alpha = alpha[j];

	return totallogprob;
}

double em_fit(const vec &samples, Distribution *d1, Distribution* d2, vec &dropped)
{
	std::vector<Distribution*> distribs;
	distribs.push_back(d1);
	distribs.push_back(d2);

	return em_fit(samples, distribs, dropped);
}

} // namespace mv
