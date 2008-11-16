#ifndef _EM_FIT_H
#define _EM_FIT_H

#include "mv.h"

namespace mv {

class Distribution
{
public:
	virtual void maximize_likelihood(const vec &samples, const vec &weights) = 0;
	virtual double probability(double val) = 0;
	virtual double log_probability(double val) = 0;
	virtual ~Distribution() {};
	double alpha; // Mixing coefficient
};

class RayleighDistribution : public Distribution
{
public:
	RayleighDistribution() {};
	RayleighDistribution(double sigma2) {
		this->sigma2 = sigma2;
	}
	void maximize_likelihood(const vec &samples, const vec &weights) {
		sigma2 = sum(samples*samples*weights);
		sigma2 /= 2*sum(weights);
	}
	double probability(double x) {
		return x/sigma2 * exp(-x*x/2./sigma2);
	}
	double log_probability(double x) {
		return log(x) - log(sigma2) - x*x/2./sigma2;
	}

	virtual ~RayleighDistribution() {};
	double sigma2;
private:
};

class RightUniformDistribution : public Distribution
{
public:
	RightUniformDistribution(double a) {
		this->a = a;
	}
	void maximize_likelihood(const vec &samples, const vec &weights) {
		a = amax(samples*weights);
	}
	double probability(double x) {
		if (x <= a)
			return 1./a/a*x;
		return 0.;
	}

	double log_probability(double x) {
		if (x <= a)
			return log(x) - 2*log(a);
		return -HUGE_VAL;
	}

	double a;
private:
};

// Untested!
class GaussianDistribution : public Distribution
{
public:
	GaussianDistribution(double mu, double sigma2) {
		this->mu = mu;
		this->sigma2 = sigma2;
	}
	void maximize_likelihood(const vec &samples, vec &weights) {
		double total_weight = sum(weights);
		mu = sum(samples*weights) / total_weight;
		sigma2 = sum(samples*samples*weights) - mu*mu;
	}
	double probability(double x) {
		return 1./sqrt(2*M_PI*sigma2) * exp(-(x-mu)*(x-mu)/2./sigma2);
	}

	double log_probability(double x) {
		return -.5*log(2*M_PI) - log(sigma2) - (x-mu)*(x-mu)/2./sigma2;
	}

	double mu;
	double sigma2;
private:
};

//ostream& operator << (ostream& os, const student& s)
//{
//return os<<s.get_name()<<'\t'<<s.get_department()<<endl;
//}


/**
 * Calculates log(sum(exp(log_x))), but in a way which tries to avoid underflow
 * / overflow. It works by shifting before exp and reshift back after, thanks
 * to the identity: log(sum(exp(x))) = alpha + log(sum(exp(x-alpha)));
 * @param log_x Log of x; this should have more precision than b itself.
 * @return log(sum(x)) with higher precision than the naive method
 */
double logsum(vec &log_x);

/**
 * Fits a mixture distribution via EM
 * @return sum of log probablity of final mixture 
 */
double em_fit(const vec &samples, std::vector<Distribution*> &ds, vec &dropped);
double em_fit(const vec &samples, Distribution *d1, Distribution* d2, vec &dropped);

} // namespace mv

#endif // _EM_FIT_H

