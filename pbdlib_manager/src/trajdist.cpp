#include "pbdlib/trajdist.h"


namespace pbdlib
{

TrajDist::TrajDist(const colvec& _Mu, const mat& _Sigma, const uint& _nbVars)
{
		Mu = _Mu;
		Sigma = _Sigma;
		nbVars = _nbVars;
}

TrajDist TrajDist::condition(vector<ConditionPoint> points)
{
		// Todo Implement
		cout << "Function not yet Implemented" << endl;

		return *this;
}

TrajDist TrajDist::lintrans(const mat& A, const colvec& b)
{
		// Perform linear transformation on trajectory distribution
		this->nbData = this->Mu.n_elem/this->nbVars;
		
		// Repeat A matrix and b vector overr all data points of distribution
	 	mat Ahat = kron(eye(this->nbData,this->nbData),A);
		colvec bhat = kron(ones(this->nbData,1),b);

		// Perform linear transformation
		mat Sigma_ = Ahat*this->Sigma*Ahat.t();
		colvec Mu_ = Ahat*this->Mu + bhat;

		// Return new distribution
		return TrajDist(Mu_, Sigma_, this->nbVars);
}

TrajDist& TrajDist::operator=(const TrajDist& rhs)
{
		// Check for self assignment:
		if (this != &rhs)
		{
				// Copy conents:
				this->Mu = rhs.Mu;
				this->Sigma = rhs.Sigma;
				this->nbVars = rhs.nbVars;
		}

		return *this;
}

TrajDist& TrajDist::operator*=(const TrajDist& rhs)
{
	// Compute precision matrices:
	mat reg = eye(rhs.Sigma.n_rows,rhs.Sigma.n_cols)*1e-10;
	mat iSrhs =inv(rhs.Sigma+reg);
	mat iSlhs =inv(this->Sigma+reg);

	// Compute new Covariance:
	this->Sigma = inv(iSlhs + iSlhs);

	// Compute new mean
	this->Mu = this->Sigma*(iSlhs*this->Mu + iSrhs*rhs.Mu);

	return *this;
}
			
TrajDist TrajDist::operator*(const TrajDist& rhs)
{
		// Create new object based on the current one
		TrajDist result = *this;
		result *=rhs;
		return result;
}

} // Namespace
