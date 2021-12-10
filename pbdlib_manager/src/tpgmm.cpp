/**
Copyright (C) 2014, Tohid Alizadeh, Milad Malekzadeh, Leonel Rozo, João Silvério, Sylvain Calinon

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

#include "pbdlib/tpgmm.h"

namespace pbdlib
{

TPGMM::TPGMM(uint nVARS, uint nSTATES, uint _nTPs)
{
	// Assign object variables:
	this->nVARS = nVARS;
	this->nSTATES = nSTATES;
	this->nTaskParameters = _nTPs;
	this->PRIORS.resize(this->nSTATES);
	this->GMMS.reserve(this->nTaskParameters);

	
	// Prepare for calculating Gaussian Product:
	setAuxiliaryVarsProdGauss();
}


TPGMM::TPGMM(std::vector<TPDemonstration>& demos,uint  _nSTATES)
{
	// Extract information from the first demonstration:
	this->nVARS   = demos[0].getNumVARS();
	this->nSTATES = _nSTATES;
	this->PRIORS  = rowvec(nSTATES);
	this->nTaskParameters = demos[0].getNumTASKPARAMETERS();
	this->GMMS.reserve(nTaskParameters);
	
	// Copy all data from the demonstrations. The data matrix will be shaped as:
	// [nVars*nParameters x nPOINTS]
	mat tmpData;
	for (uint i = 0;i<demos.size();i++)
	{
		if (i==0)
		{
			tmpData = demos[0].getDataInTaskParameters();
			this->Data = tmpData;
		}
		else
		{
			tmpData = demos[i].getDataInTaskParameters();
            this->Data = join_horiz(this->Data,tmpData );
		}

		/*
		// For outputting the collected data: 
		cout << tmpData.t() << endl;
		for (uint n=0;n<nTaskParameters;n++)
		{
			cout << "A["<< n << "] demo[" << i<< "] :" << endl;
			cout << demos[i].getTaskParameters(0).getTaskParameters(n).A << endl;
			cout << "b["<< n << "] demo[" << i<< "] :" << endl;
			cout << demos[i].getTaskParameters(0).getTaskParameters(n).b.t() << endl;
		}
		cout<< "demo[" << i << "].data, Press Enter" << endl;
		std::getchar();
		*/
	}


	// Prepare for performing Gaussian Product:
	setAuxiliaryVarsProdGauss();
}

void TPGMM::loadTPGMMfromMATLAB(std::string PriorsFileName, std::string VarsFileName, std::string MuFilePrefix, std::string SigmaFilePrefix)
{
	std::stringstream index; //auxiliary variable to store the index of the taskParameter
	std::stringstream MuFileName;
	std::stringstream SigmaFileName;

	cout << PriorsFileName << endl;
	cout << VarsFileName   << endl;

	for(uint i=0; i<this->nTaskParameters; i++){

		MuFileName.str("");		// Clears the string
		SigmaFileName.str("");
		index.str("");
		index<<(i+1);

		MuFileName    << MuFilePrefix    << "_P" << index.str() << ".txt";
		SigmaFileName << SigmaFilePrefix << "_P" << index.str() << ".txt";

		cout << MuFileName.str()     << endl;
		cout << SigmaFileName.str() << endl;

		GMM_Model GMM_temp(PriorsFileName, MuFileName.str(), SigmaFileName.str(), VarsFileName);
		this->PRIORS = GMM_temp.getPRIORS();
		this->GMMS.push_back(GMM_temp);
		
		// Setting variables' name using the last temporary GMM (For avoiding the re-implementation o$


		if(this->vars_names.size() == 0)
			this->setVARSNames(GMM_temp.getVARSNames());
	}

	cout << "TP-GMM loaded from text files!" << endl;

	// Prepare for performing Gaussian Product:
	setAuxiliaryVarsProdGauss();
}
void TPGMM::loadFromFiles(std::string path)
{

}

TPGMM::TPGMM(std::vector<GMM_Model> gmms){

//	this->nVARS = GMMs[0]

	assert(gmms.size() > 0);
	this->GMMS = gmms;
	this->nTaskParameters = this->GMMS.size();
	this->nSTATES = this->GMMS[0].getNumSTATES();
	this->nVARS = this->GMMS[0].getNumVARS();
	this->PRIORS = this->GMMS[0].getPRIORS();

	setAuxiliaryVarsProdGauss();
}
TPGMM::TPGMM(std::string path, uint _nTPs)
{
	this->nTaskParameters = _nTPs;
	this->GMMS.reserve(this->nTaskParameters);

	for(int l=0;l<_nTPs;l++)
	{
		mat priors, mu, sigma;
		char filename[256];

		sprintf(filename,"GMM_priors%i.txt",l);
		priors.load(path + filename, raw_ascii);

		sprintf(filename,"GMM_mu%i.txt",l);
		mu.load(path + filename, raw_ascii);

		sprintf(filename,"GMM_sigma%i.txt",l);
		sigma.load(path + filename, raw_ascii);
		cout << "Priors \n" << priors << endl;

		nVARS = mu.n_rows;
		nSTATES = priors.n_elem;

		mat _SIGMA = zeros(nVARS, nVARS);
		colvec _MU = zeros(nVARS,1);
		rowvec _PRIORS(1, nSTATES);
		std::vector<GaussianDistribution> components;

		for(uint i=0; i<nSTATES; i++){
			_PRIORS(0,i) = priors(0,i);
			for(uint j=0; j<nVARS; j++){
				_MU(j,0) = mu(j,i);
				for(uint k=0; k<nVARS; k++)
					_SIGMA(j,k) = sigma(j,k + i*(nVARS));
			}
			components.push_back(GaussianDistribution(_MU, _SIGMA));
		}
//		GMMS[l] = GMM_Model(nSTATES,nVARS);
//		GMMS[l].setPRIORS(_PRIORS);
//		GMMS[l].setCOMPONENTS(components);
//
		GMM_Model GMM_temp(nSTATES,nVARS);
		GMM_temp.setPRIORS(_PRIORS);
		GMM_temp.setCOMPONENTS(components);
		this->GMMS.push_back(GMM_temp);

	}
	// Assign object variables:

	this->PRIORS.resize(nSTATES);


	PRIORS.load(path+"TPGMM_priors.txt", raw_ascii );

	// Prepare for calculating Gaussian Product:
	setAuxiliaryVarsProdGauss();
}
void TPGMM::saveInFiles(std::string path)
{
	for(int l=0;l<nTaskParameters;l++)
	{
		mat priors(1, nSTATES);
		mat mu(nVARS, nSTATES);
		mat sigma(nVARS, nVARS*nSTATES);

		for(uint i=0; i<nSTATES; i++){
			priors(0,i) = GMMS[l].getPRIORS()(i);
			for(uint j=0; j<nVARS; j++){
				mu(j,i) = GMMS[l].getMU(i)(j);
				for(uint k=0; k<nVARS; k++)
					sigma(j,k + i*(nVARS)) = GMMS[l].getSIGMA(i)(j,k);
			}
		}
//		char filename[256];
//		sprintf(filename,"GMM_priors%i.txt",l);
//		priors.save(path + filename, raw_ascii);
//
//		sprintf(filename,"GMM_mu%i.txt",l);
//		mu.save(path + filename, raw_ascii);
//
//		sprintf(filename,"GMM_sigma%i.txt",l);
//		sigma.save(path + filename, raw_ascii);

		char filename[256];
		sprintf(filename,"GMM_priors_P%i.txt",l+1);
		priors.save(path + filename, raw_ascii);

		sprintf(filename,"GMM_mu_P%i.txt",l+1);
		mu.save(path + filename, raw_ascii);

		sprintf(filename,"GMM_sigma_P%i.txt",l+1);
		sigma.save(path + filename, raw_ascii);

	}

	PRIORS.save(path + "TPGMM_priors.txt", raw_ascii);
}

void TPGMM::setAuxiliaryVarsProdGauss()
{
	// Function prepares the auxiliary variables for the product of Gaussian:
	prodGMM= new GMM_Model(nSTATES, nVARS);
	accMu.set_size(nVARS);
	tmpMu.set_size(nVARS);
	accLambda.set_size(nVARS,nVARS);
	tmpLambda.set_size(nVARS,nVARS);

	// Vectors to hold inv(A) and inv(A^T):
	invA.resize(nTaskParameters);
	invAT.resize(nTaskParameters);
	for (uint m=0;m<nTaskParameters;m++)
	{
		invA[m] = zeros(nVARS,nVARS);
		invAT[m] = zeros(nVARS,nVARS);
	}
}

GMM_Model* TPGMM::getTransformedGMM(TaskParameters _TPs, EMatrixType AType)
{
	return getTransformedGMM(_TPs.getTaskParameters(),AType);
}

GMM_Model* TPGMM::getTransformedGMM(std::vector<TaskParameter> _TPs, EMatrixType AType)
{
	double diagRegularizationFactor = 1E-4;	//	regularization factor to invert A*Sigma*A' which is often close to singular

	// Check size of Task taskparameters
	if (_TPs.size()!=nTaskParameters)
		throw std::invalid_argument("TPGMM::getTransformedProdGMM(std::vector<TaskParameter> _TPs, EMatrixType AType) Number of supplied task parameters is not consistent expected number of taskparameters");
	

	//Projections in the different candidate taskParameters (see Eq. (3) Calinon et al, Humanoids'2012 paper)
	if (AType==OTHER || nSTATES<2)
	{
		// Use implementation based on covariance matrices when type of matrix A is not specified or
		// the number of states in the GMM is less than 2
		for (uint i=0; i<nSTATES; i++){
			accMu = zeros(nVARS, 1);
			accLambda= zeros(nVARS, nVARS);
			for (uint m = 0; m < nTaskParameters ; m++) 
			{
				// Projecting Zmu of state "i" at taskParameter "m"
				tmpMu = _TPs[m].A * GMMS[m].getMU(i) + _TPs[m].b;		

				// Projecting Zsigma of state "i" at taskParameter "m"
				tmpLambda= inv(_TPs[m].A * GMMS[m].getSIGMA(i) * trans(_TPs[m].A) + eye(nVARS,nVARS)*diagRegularizationFactor);

				// Accumulate:
				accLambda+= tmpLambda;      	
				accMu+=  tmpLambda* tmpMu;
			}
			// Saving the Gaussian distribution for state "i"
			prodGMM->setMU(i,accLambda.i()*accMu);
			prodGMM->setLAMBDA(i,accLambda);
		}
			prodGMM->setPRIORS(this->PRIORS);
	}
	else
	{
		// Approach based on Precision matrices:
		// Requires less inverses
		
		// Pre-compute A^-1 and (A^T)^-1.
		// First select method based on type of matrix A:
		switch (AType)
		{
			case ORTHONORMAL:
				for (uint m =0; m<nTaskParameters; m++)
				{
					invA[m] = _TPs[m].A.t();
					invAT[m] = _TPs[m].A;
				}
				break;
			case SYMMETRIC:
				for (uint m =0; m<nTaskParameters; m++)
				{
					invA[m] = _TPs[m].A.i();
					invAT[m] = invA[m];
				}
				break;
			case INVERTIBLE:
				for (uint m =0; m<nTaskParameters; m++)
				{
					invA[m] = _TPs[m].A.i();
					invAT[m] = inv(_TPs[m].A.t());
				}
				break;
			default:
				break;
		}

		// Compute product of Gaussian
		for (uint i=0; i<nSTATES; i++){
			accMu = zeros(nVARS, 1);
			accLambda= zeros(nVARS, nVARS);

			for (uint m = 0; m < nTaskParameters ; m++)
			{
				// Projecting Zmu of state "i" at taskParameter "m"
				tmpMu= _TPs[m].A * GMMS[m].getMU(i) + _TPs[m].b;		

				// Projecting Zsigma of state "i" at taskParameter "m"
				tmpLambda= invAT[m] * GMMS[m].getLAMBDA(i) * invA[m];	

				// Accumulate
				accLambda+= tmpLambda;      	 
				accMu+=  tmpLambda*tmpMu;  
			}
				
			// Saving the Gaussian distribution for state "i"
			prodGMM->setMU(i,accLambda.i()*accMu);
			prodGMM->setLAMBDA(i,accLambda);
		}
		prodGMM->setPRIORS(this->PRIORS);
	}

	/*
	for (uint i=0;i<nSTATES;i++)
	{
		cout << prodGMM->getMU(i) << endl;
		cout << prodGMM->getSIGMA(i) << endl;
	}
	*/

	return prodGMM;
}

void TPGMM::tpOnlineEMDP(TaskParameters _TPs, int N,colvec P,double lambda, double minSigma){
	// Check size of Task taskparameters
	if (_TPs.getTaskParameters().size()!=nTaskParameters)
		throw std::invalid_argument("TPGMM::getTransformedProdGMM(std::vector<TaskParameter> _TPs, EMatrixType AType) Number of supplied task parameters is not consistent expected number of taskparameters");

	for (uint m = 0; m < nTaskParameters ; m++)	{
		// Projecting Zmu of state "i" at taskParameter "m"
		//solve(tmpParam.A,this->Data.getData(i)-tmpParam.b);
		//colvec tmpP = _TPs.getTaskParameters()[m].A * P + _TPs.getTaskParameters()[m].b;
		colvec tmpP = solve(_TPs.getTaskParameters()[m].A,  P - _TPs.getTaskParameters()[m].b);
		if (N == 1){
			GMM_Model tmpGMM(nSTATES,nVARS);
			GMMS.push_back(tmpGMM);
			mat minSIGMA = eye(nVARS,nVARS) * minSigma;
			//			GaussianDistribution componentTmp(tmpP,minSIGMA);
			//			std::vector<GaussianDistribution> comps;
			//			comps.push_back(componentTmp);
			//			GMMS[m].setCOMPONENTS(comps);
			GMMS[m].setMU(0,tmpP);
			GMMS[m].setSIGMA(0,minSIGMA);
		}
//		cout << "mu: " << GMMS[m].getMU(0);
//		cout << "s: " << GMMS[m].getSIGMA(0);
		// Projecting Zsigma of state "i" at taskParameter "m"
		GMMS[m].onlineEMDP(N,tmpP,lambda,minSigma);
	}
	//cout << "this->nSTATES = " << this->nSTATES;
	//cout << "GMMS[0].getNumSTATES() = " << GMMS[0].getNumSTATES();
	if (this->nSTATES!=GMMS[0].getNumSTATES()){
		this->nSTATES=GMMS[0].getNumSTATES();
		prodGMM= new GMM_Model(nSTATES, nVARS);
		this->PRIORS = GMMS[0].getPRIORS();
	}



}

/*----------------------------------------------------------------------------
 *				EM-ESTIMATION FUNCTIONS 
 *--------------------------------------------------------------------------- */


/*----------------------------------------------------------------------------*/
uint TPGMM::estimateTensorGMM()
{
//	cout << "initTensorGMM ";
	initTensorGMM_timeBased();
//	cout << "Done" << endl;
	
//	cout << "EMTensorGMM ";
	EM_tensorGMM();
//	cout << "Done" << endl;
	return true;
}

/*----------------------------------------------------------------------------*/
void TPGMM::initTensorGMM_timeBased()
{
	// Auxiliary variable:
	mat _tmpData;
	mat _Mu;
	cube _Sigma;
	rowvec _TimingSep, _Priors;
	uvec _idTmp;
	double diagRegMat = 1e-4;

	// Check if Data is of correct size
	if (Data.n_rows != nTaskParameters*nVARS)
	{
		throw std::invalid_argument("[ERROR]::TPGMM::initTensorGMM_timeBased() Data dimension is not consistent with specified number of Parameters and number of Variables.");
	}

	// Linear Space Gaussian in time:
	_TimingSep = linspace(min(Data.row(0)), max(Data.row(0)),nSTATES+1).t();
	
	// Initialize 
	_Mu = zeros(nVARS*nTaskParameters,nSTATES);
	_Sigma = zeros(nVARS*nTaskParameters,nVARS*nTaskParameters,nSTATES);
	_Priors = zeros(1,nSTATES);
	
	for (uint i=0; i<nSTATES; i++)
	{
		// Find All indices of data that 
		_idTmp = find(Data.row(0)>=_TimingSep(i));
		_tmpData = Data.cols(_idTmp);
		_idTmp = find(_tmpData.row(0) <=_TimingSep(i+1));
		// Calculate Properties of GMM;
		_Mu.col(i) = mean(_tmpData.cols(_idTmp),1);
		_Sigma.slice(i) = cov(_tmpData.cols(_idTmp).t()) + eye(nVARS*nTaskParameters,nVARS*nTaskParameters)*diagRegMat;
		_Priors(i) = _idTmp.n_elem;
	}
	this->PRIORS=_Priors/sum(_Priors);

	// Copy result into a TPGMM structure.
	// Here we use a vector of GMM in which each GMM represents a representation in one Task Parameter
	mat _StateSigma, _TPSigma;
	colvec _TPMu;
	
	GMM_Model tmpGMM(nSTATES,nVARS);
	for (uint m=0; m<nTaskParameters; m++){
		GMMS.push_back(tmpGMM);
		for (uint i=0; i<nSTATES; i++){
			// Separate the GMMs for each Task Parameter:
			_TPMu= _Mu.submat(m*nVARS,i,(m+1)*nVARS-1,i);
			_StateSigma= _Sigma.slice(i);
			_TPSigma= _StateSigma.submat(m*nVARS,m*nVARS, (m+1)*nVARS-1,(m+1)*nVARS-1);

			// Saving the Gaussian distribution for state "i"
			GMMS[m].setSIGMA(i,_TPSigma);
			GMMS[m].setMU(i,_TPMu);
		}
		GMMS[m].setPRIORS(this->PRIORS);
	}

}

/*----------------------------------------------------------------------------*/
void TPGMM::initTensorGMM_kbins(std::vector<TPDemonstration>& demos){
	std::vector<mat> bins;
	for (int j = 0; j < this->nSTATES; ++j) {
		mat tmpMat;
		bins.push_back(tmpMat);
	}

	// Split each demonstration in K equal bins
	for (int i = 0; i < demos.size(); ++i) {
		mat data = demos[i].getDataInTaskParameters();
		vec binSep = round(linspace(0, demos[i].getNumPOINTS(), this->nSTATES+1));
		for (int j = 0; j < this->nSTATES; ++j) {
			bins[j] = join_horiz(bins[j], data.cols(binSep(j), binSep(j+1)-1));
		}
	}

	// Auxiliary variables and initialization
	mat _Mu = zeros(nVARS*nTaskParameters, nSTATES);
	cube _Sigma = zeros(nVARS*nTaskParameters, nVARS*nTaskParameters, nSTATES);
	rowvec _Priors = zeros(1, nSTATES);

	// Calculate statistics on bin data
	for (int i = 0; i < this->nSTATES; ++i) {
		_Mu.col(i) = mean(bins[i], 1);
		_Sigma.slice(i) = cov(bins[i].t());
		_Priors(i) = bins[i].n_cols;
	}
	this->PRIORS=_Priors/sum(_Priors);

	// Copy result into a TPGMM structure.
	// Here we use a vector of GMM in which each GMM represents a representation in one Task Parameter
	mat _StateSigma, _TPSigma;
	colvec _TPMu;

	GMM_Model tmpGMM(nSTATES, nVARS);
	for (uint m=0; m<nTaskParameters; m++){
		GMMS.push_back(tmpGMM);
		for (uint i=0; i<nSTATES; i++){
			// Separate the GMMs for each Task Parameter:
			_TPMu = _Mu.submat(m*nVARS, i, (m+1)*nVARS-1, i);
			_StateSigma = _Sigma.slice(i);
			_TPSigma = _StateSigma.submat(m*nVARS, m*nVARS, (m+1)*nVARS-1, (m+1)*nVARS-1);

			// Saving the Gaussian distribution for state "i"
			GMMS[m].setSIGMA(i, _TPSigma);
			GMMS[m].setMU(i, _TPMu);
		}
		GMMS[m].setPRIORS(this->PRIORS);
	}
}

static uvec randperm( int n )
{
	//return sort_index(randn<urowvec>(nbData));
	return sort_index(randu(n)); // was randn, should work equally well?
}

void TPGMM::initTensorGMM_kmeans(){

	//Ajay Tanwani, 2016

	double diagRegularizationFactor = 1E-4;
	double cumdist_threshold = 1e-10;
	uint maxIter = 100;

	mat _Mu;
	cube _Sigma;
	rowvec _Priors;

	// Check if Data is of correct size
	if (Data.n_rows != nTaskParameters*nVARS){
		throw std::invalid_argument("[ERROR]::TPGMM::initTensorGMM_kmeans() Data dimension is not consistent with specified number of Parameters and number of Variables.");
	}

	//Criterion to stop the EM iterative update
	uint nbVar = Data.n_rows;
	uint nbData = Data.n_cols;

	// Initialize
	_Mu = zeros(nVARS*nTaskParameters,nSTATES);
	_Sigma = zeros(nVARS*nTaskParameters,nVARS*nTaskParameters,nSTATES);
	_Priors = zeros(1,nSTATES);

	double cumdist_old = -std::numeric_limits<double>::max();
	uint nbStep = 0;

//	cout << "rand perm" << endl;
//	uvec idTmp = randperm(nbData);
	uvec idTmp = linspace<uvec>(1, nbData, nbData);

//	cout << "linspace" << endl;
	uvec allrows = linspace<uvec>(0, nbVar-1, nbVar);

	// nSTATES means
	_Mu = Data.submat(allrows, idTmp.subvec(0,nSTATES-1));

	uvec idList = zeros<uvec>(nbData,1);

	//k-means iterations
	while(true){
		mat distTmp = zeros(nbData,nSTATES);

		//E-step %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
		for (uint i=0; i<nSTATES; i++){
			//Compute distances
			distTmp.col(i) = trans( sum( pow(Data - repmat(_Mu.col(i), 1, nbData), 2.0) ) );
		}

		vec vTmp = zeros<vec>(nbData,1);
		uvec idList = zeros<uvec>(nbData,1);

		for( uint i = 0; i < nbData; i++ ){
			// there is a chance that here two elements will be as close and will skip one?s
			vTmp[i] = ((rowvec)distTmp.row(i)).min(idList[i]);
		}

		double cumdist = sum(vTmp);

		//%%%%%%%% M-step %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
		for (uint i=0; i<nSTATES; i++){
			mat TmpData;
			for (uint j=0; j < nbData; j++){
				if (idList(j) == i){
					TmpData = join_horiz(Data.col(j), TmpData);
				}
			}
//				uvec idTmp = find(idList == i);
//				_Mu.col(i) = mean(Data.submat(allrows, idTmp), 1);
			_Mu.col(i) = mean(TmpData, 1);
		}

 		//Stopping criterion %%%%%%%%%%%%%%%%%%%%
		if (fabs(cumdist-cumdist_old) < cumdist_threshold){
			cout << "%%%%%%%%%%% KMEANS %%%%%%%%%%%%%%%%%%" << endl;
			cout << nbStep << "Steps" << endl;
			cout << endl << "Mu_kmeans :" << endl << _Mu << endl;

			for(int i=0; i<nSTATES; i++){
				uvec idTmp = find(idList == i);

				PRIORS(i) = idTmp.n_elem;

				_Sigma.slice(i) = cov( trans(join_rows(Data.submat(allrows, idTmp),Data.submat(allrows, idTmp))) ,0 )
							+ eye(nbVar, nbVar)*diagRegularizationFactor;
			}

			PRIORS = PRIORS / nbData;
			this->PRIORS = PRIORS;
			cout << "PRIORS:" << endl << PRIORS << endl;

			cout << "%%%%%%% KMEANS FINISHED %%%%%%%%%%%%%" << endl;
			break;
		}
		cumdist_old = cumdist;
		nbStep = nbStep + 1;

		if (nbStep > maxIter){
			cout << "Max Iterations Reached: KMeans finished before convergence " << endl;
			break;
		}
	}

//	%Reshape GMM parameters into a tensor
	mat _StateSigma, _TPSigma;
	colvec _TPMu;

	GMM_Model tmpGMM(nSTATES,nVARS);
	for (uint m=0; m<nTaskParameters; m++){
		GMMS.push_back(tmpGMM);
		for (uint i=0; i<nSTATES; i++){
			// Separate the GMMs for each Task Parameter:
			_TPMu = _Mu.submat(m*nVARS,i,(m+1)*nVARS-1,i);
			_StateSigma= _Sigma.slice(i);
			_TPSigma= _StateSigma.submat(m*nVARS,m*nVARS, (m+1)*nVARS-1,(m+1)*nVARS-1);

			// Saving the Gaussian distribution for state "i"
			GMMS[m].setSIGMA(i,_TPSigma);
			GMMS[m].setMU(i,_TPMu);
		}
		GMMS[m].setPRIORS(this->PRIORS);
	}

}


/*----------------------------------------------------------------------------*/
void TPGMM::EM_tensorGMM()
{
	//Parameters of the EM algorithm
	uint nbMinSteps = 5; //Minimum number of iterations allowed
	uint nbMaxSteps = 100; //Maximum number of iterations allowed
	double maxDiffLL = 1E-4; //Likelihood increase threshold to stop the algorithm
	double diagRegularizationFactor = 1E-4;

	// Check if Data is of correct size
	if (Data.n_rows != nTaskParameters*nVARS)
	{
		throw std::invalid_argument("[ERROR]::TPGMM::EM_tensorGMM() Data dimension is not consistent with specified number of Parameters and number of Variables.");
	}
	
	// Start EM algorithm
	uint nDATA = Data.n_cols;     //nDATA is the total number of data points including all the demonstrations
	mat DataTmp,DataTmp2;
	vec MuTmp;
	mat SigmaTmp;
	std::vector<mat> GAMMA0;
	mat GAMMA = zeros(nVARS, nDATA);
//	mat GAMMA2 = zeros(nVARS, nDATA);
	GAMMA2 = zeros(nVARS, nDATA);
	vec LL = zeros(nbMaxSteps);
	GAMMA0.resize(nTaskParameters);
	for (uint m=0;m<nTaskParameters; m++){
		GAMMA0[m] = zeros(nSTATES, nDATA);
	}

	cout<<"Start to learn TPGMM using tensor EM"<<endl;
	for (uint nbIter=0; nbIter<nbMaxSteps; nbIter++){
		// Expectation-Step, see Eq. (6.1) in doc/TechnicalReport.pdf
		mat L = ones(nSTATES,nDATA);

		for (uint m=0; m<nTaskParameters; m++){
			DataTmp = Data.rows(m*nVARS, (m+1)*nVARS-1);
			for(uint i=0; i<nSTATES; i++){
				GAMMA0[m].row(i) = GMMS[m].getPRIORS(i) * trans(GMMS[m].getCOMPONENTS(i).getPDFValue(DataTmp));
				L.row(i) = L.row(i) % GAMMA0[m].row(i); //elementwise product
			}
		}
		GAMMA = L/repmat(sum(L,0)+REALMIN,L.n_rows,1);
		GAMMA2 = GAMMA/repmat(sum(GAMMA,1),1,GAMMA.n_cols);

		// Maximization-Step, see Eq. (6.2),(6.3) and (6.4) in doc/TechnicalReport.pdf
		MuTmp = zeros(nVARS);
		SigmaTmp = zeros(nVARS, nVARS);
		for (uint m=0; m<nTaskParameters; m++){
			DataTmp = Data.rows(m*nVARS, (m+1)*nVARS-1);
			for(uint i=0; i<nSTATES; i++){
				//update PRIORS
				PRIORS(i) = sum(sum(GAMMA.row(i)))/GAMMA.n_cols;
				//update Mu
				MuTmp = DataTmp * trans(GAMMA2.row(i));
				GMMS[m].setMU(i, MuTmp);
				//update Sigma
				DataTmp2 = DataTmp - repmat(MuTmp, 1, DataTmp.n_cols);
				SigmaTmp = DataTmp2 * diagmat(GAMMA2.row(i)) * trans(DataTmp2) + eye(nVARS,nVARS)*diagRegularizationFactor;
				GMMS[m].setSIGMA(i, SigmaTmp);
			}
			GMMS[m].setPRIORS(PRIORS);
		}

		//Compute average log-likelihood
		mat LL_Tmp;
		LL(nbIter) = 0;
		for(int n=0; n<nDATA; n++){
			LL_Tmp = log(sum(L.col(n),0));
			LL(nbIter) += as_scalar(sum(LL_Tmp,1));
		}
		LL(nbIter)/= nDATA;

		//Stop the algorithm if EM converged (small change of LL)
		if (nbIter>nbMinSteps){
			if (LL(nbIter)-LL(nbIter-1)<maxDiffLL || nbIter==nbMaxSteps-1){
				cout<<"EM converged after "<<nbIter<<" iterations"<<endl;
				cout<<"The results of the model after the learning by EM:"<<endl;
				cout<<"PRIORS"<<endl;
				cout<<PRIORS<<endl;
				for (int i=0; i<nSTATES; i++){
					for(int m=0; m<nTaskParameters; m++){
						cout<<"GMMS["<<m<<"].getMU("<<i<<"):     "<<endl;
						cout<<GMMS[m].getMU(i)<<endl;
						cout<<"GMMS["<<m<<"].getSIGMA("<<i<<"):  "<<endl;
						cout<<GMMS[m].getSIGMA(i)<<endl;
					}
				}
				break;
			}
		}
		if (nbIter==nbMaxSteps)
			cout<<"EM converged after "<<nbMaxSteps<<" iterations"<<endl;
	}
}


/*----------------------------------------------------------------------------
 *               GET/SET FUNCTIONS	
 *--------------------------------------------------------------------------- */
uint TPGMM::getNumVARS()
{
	return this->nVARS;
}

std::vector<GMM_Model>& TPGMM::getGMMS(){
	return this->GMMS;
}

 void TPGMM::setGMMS(std::vector<GMM_Model>& vGMMS){
	this->GMMS = vGMMS;
}

GMM_Model& TPGMM::getGMMS(uint id)
{
	// Check if GMM is available:
	if (GMMS.size()<=id)
		throw std::invalid_argument("[Error]TPGMM::getGMMS(int id), id cannot be larger than number of frames in TPGMM.");

	return this->GMMS[id];
}

uint TPGMM::getNumSTATES()
{
	return this->nSTATES;
}

uint TPGMM::getNumFRAMES()
{
	return this->nTaskParameters;
}

rowvec&  TPGMM::getPRIORS()
{
	return this->PRIORS;
}

void TPGMM::setPRIORS(rowvec pr) {
	this->PRIORS = pr;
}

double  TPGMM::getPRIORS(uint id)
{
	if (nSTATES<=id)
		throw std::invalid_argument("[Error]TPGMM::getPRIORS(int id), id cannot be larger than number of stats in TPGMM.");
	return this->PRIORS(id);
}

std::vector<std::string>& TPGMM::getVARSNames()
{
	return this->vars_names;
}

void TPGMM::setVARSNames(const std::vector<std::string>& vars)
{

	if(vars.size() == nVARS)
		this->vars_names = vars;
	else
	{

		std::ostringstream msg;
		msg << "\n [ERROR]::TPGMM::usetVARSNames if(vars.size() == nVARS) number of varnames (";
		msg << vars.size() << ") names does not correspond with specified number of variables (" ;
		msg << nVARS << ").";
	
		throw std::invalid_argument(msg.str());
	}
}

} //end of namespace pbdlib
