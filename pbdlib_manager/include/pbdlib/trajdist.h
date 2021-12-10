// December,2015, Martijn Zeestraten


#ifndef TRAJDIST_H_
#define TRAJDIST_H_

#include "armadillo"
#include "vector"

using namespace arma;
using namespace std;

namespace pbdlib
{
	struct ConditionPoint
	{
			uint index;
			colvec Mu;
			mat Sigma;
			ConditionPoint(){}
			ConditionPoint(uint index, colvec Mu)
			{
					this->index = index;
					this->Mu = Mu;
					this->Sigma = zeros(Mu.n_elem,Mu.n_elem) ;
			}
			ConditionPoint(uint index, colvec Mu, mat Sigma)
			{
					this->index = index;
					this->Mu = Mu;
					this->Sigma = Sigma;
			}
	};

	
	struct TrajDist
	{
			/* Class to represent trajectory distributions as a Gaussian.
			 *
			 * The class is still under development. Currently it only supports 
			 * the product of distributions, in order to combine multiple Gaussian.
			 * It should also implement Gaussian conditioning in order to allow Gaussian Conditioning
			 * to modify the trajectory*/
			colvec Mu;
			mat Sigma;
			uint nbVars;
			uint nbData;

			TrajDist(){}
			TrajDist(const colvec& Mu, const mat& Sigma,const uint& nbVars);
			TrajDist condition(vector<ConditionPoint> points);                 // Gaussian Conditioning
			TrajDist lintrans(const mat& A, const colvec& b);

			// Define operations for TrajectoryDistributions:
			TrajDist& operator=(const TrajDist& rhs);   // Copying
			TrajDist& operator*=(const TrajDist& rhs);  // Product of Gaussian
			TrajDist operator*(const TrajDist& rhs);   // Product of Gaussian


	};


}



#endif
