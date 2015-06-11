#include "object_tracking_2D/tracker_pf.h"

ParticleFilterTracker::ParticleFilterTracker()
	: ar_param_(1.0f)
	, num_particles_(100)
	, noise_l_(0.0005f)
	, noise_h_(0.02f)
	, num_annealing_layers_(1)
	, alpha_rate_(0.5f)
	, beta_rate_(0.5f)
	, noise_an_(0.05f)
	, th_ransac_(0.f)
	, th_ransac_iter_(1000)
	, lamda_e_(0.5f)
	, lamda_v_(25.f)
	, pf_(NULL)
	, th_neff_ratio_(0.2f)
{
	mean_ = cvCreateMat(4, 4, CV_32F);

	initAnnealing();
}

ParticleFilterTracker::~ParticleFilterTracker()
{
	cvReleaseMat(&mean_);
	delete pf_;
}

bool ParticleFilterTracker::initParticleFilter()
{
	if(pf_) delete pf_;

	pf_ = new CParticleFilter (num_particles_, ar_param_, limityrot_);

	return (true);
}

void ParticleFilterTracker::initAnnealing()
{
	alpha_.resize(num_annealing_layers_);
	beta_.resize(num_annealing_layers_);

	for(int i = 0; i < num_annealing_layers_; i++)
	{
		alpha_[i] = noise_an_ * pow(alpha_rate_, static_cast<float>(num_annealing_layers_ - 1 - i));
		beta_[i] = pow(beta_rate_, static_cast<float>(i));
	}
}

void ParticleFilterTracker::displayResults()
{
	// draw particles
	for(int i = 0; i < pf_->GetNumOfParticle(); i++)
	{
		if(dulledge_) // for valid display, occlusion reasioning is required
		{
			obj_model_->setModelviewMatrix(pf_->GetPropState(i));
			obj_model_->findVisibleSamplePoints();
		}
		obj_model_->displayPoseLine(img_result_, pf_->GetPropState(i), CV_RGB(0, 255, 0), 1, false);
	}

	obj_model_->displaySamplePointsAndErrors(img_edge_); // display data of the last particle

	// draw mean particle
	if(dulledge_) // for valid display, occlusion reasioning is required
	{
		obj_model_->setModelviewMatrix(pf_->GetMeanState());
		obj_model_->findVisibleSamplePoints();
	}
	obj_model_->displayPoseLine(img_result_, pf_->GetMeanState(), CV_RGB(255, 255, 0), 2, false);

	cvShowImage("Result", img_result_);
	cvShowImage("Edge", img_edge_);
}


