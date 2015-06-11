#pragma once

#include "tracker_base.h"
#include "ParticleFilter.h"

class ParticleFilterTracker : public TrackerBase
{
public:
	ParticleFilterTracker();

	virtual ~ParticleFilterTracker();

	bool initParticleFilter();

	inline void setNumAnnealingLayers(int l)   { num_annealing_layers_= l; initAnnealing(); }
	inline void setNumParticle(int n)          { num_particles_ = n; }
	inline float getThresholdRatioNeff()       { return th_neff_ratio_; }
	inline void setThresholdRatioNeff(float t) { th_neff_ratio_ = t; }

protected:
	void initAnnealing();

	virtual void displayResults();

	float ar_param_;
	int num_particles_;
	float noise_l_;
	float noise_h_;
	CvMat* mean_;
	CParticleFilter* pf_;
	// annealing process
	int num_annealing_layers_;
	float alpha_rate_;
	float beta_rate_;
	float noise_an_;
	std::vector<float> alpha_;
	std::vector<float> beta_;
	float th_ransac_;
	int th_ransac_iter_;
	float lamda_e_;
	float lamda_v_;
	float th_neff_ratio_;
};
