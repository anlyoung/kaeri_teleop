/**
Copyright (C) 2014, Davide De Tommaso

This file is part of PbDLib.

    PbDLib is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    PbDLib is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with PbDLib.  If not, see <http://www.gnu.org/licenses/>.
*/

/*! \file mvn.h
\brief GaussianDistribution class
The class GaussianDistribution model a multivariate Gaussian distribution.

\author Davide De Tommaso, Milad Malekzadeh
\bug No known bugs.
*/


#ifndef MVN_H
#define MVN_H

#define THRESHOLD_MIN std::numeric_limits<float>::epsilon()
#define THRESHOLD_MAX 1.0e60

#define PI 3.14159265359

#include "armadillo"
#include "pbdlib/datapoints.h"

using namespace arma;

namespace pbdlib
{

class GaussianDistribution
{
private:
	uint nVARS;
	mat SIGMA; // Covariance Matrix
	mat LAMBDA; // Precision Matrix
	double DET; // determinant
	colvec MU;
	bool DETupdated; // indicates if the determinant has already been computed

public:
	GaussianDistribution(uint _nVARS);
	GaussianDistribution(colvec& _MU, mat& _SIGMA);

	uint    getNumVARS();
	colvec& getMU();
	mat&    getSIGMA();  // Covariance Matrix
	mat&    getLAMBDA(); // Precision Matrix
	double 	getDET();
	// Get divergence measure
	double getKLdiv( GaussianDistribution&);
	// Get PDF Values
	colvec  getPDFValue(const mat& SAMPLES); // simple implementation
	colvec  getPDFValue(const mat& SAMPLES, urowvec ind); // simple implementation with specific indexes to use
	void 	  getPDFValue(colvec& prob, mat SAMPLES);// Implementation for real time execution: doesnt require memory allocation:
	void 	  getPDFValue(colvec& prob, mat SAMPLES, urowvec ind);// Implementation for real time execution: doesnt require memory allocation:

	mat     stochasticSampling( uint nbS);
	mat     sqrtm(const mat SIGMA);

	void    setParamsFromData(const mat SAMPLES);
	void    setParamsFromData(const mat SAMPLES, const rowvec REWARD);
	void    setParamsFromData(const mat SAMPLES, const rowvec REWARD, const uint nbImportanceSampling);

	void    updateDET();
	void    setMU(const colvec& _MU);
	void    setSIGMA(const mat& _SIGMA);
	void 	  setLAMBDA(const mat& _LAMBDA);
	void    setNumVARS(uint numvars);

	// Define operations for TrajectoryDistributions:
	GaussianDistribution& operator=(const GaussianDistribution& rhs);   // Copying
	GaussianDistribution& operator*=(const GaussianDistribution& rhs);  // Product of Gaussian
	GaussianDistribution operator*(const GaussianDistribution& rhs);   // Product of Gaussian

};

} //end of namespace pbdlib

#endif
