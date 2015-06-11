#pragma once

#include "tracker_pf.h"

class TexturelessParticleFilterTracker : public ParticleFilterTracker
{
public:
	TexturelessParticleFilterTracker();

	virtual ~TexturelessParticleFilterTracker();

	virtual bool initTracker(std::string &obj_name, std::string &cam_name, std::string &intrinsic, std::string &distortion, int width, int height, CvMat* pose_init);

	inline void setThresholdCM(float th) { th_cm_ = th; }
	inline float getThresholdCM()        { return th_cm_; }

protected:
	CPoseEstimationSURF* pe_surf_;
	float th_cm_;

	virtual bool initObjectModel(std::string name, int width, int height, CvMat* intrinsic, float sample_step, int maxd, bool dulledge, CEdgeTracker* edge_tracker);

	bool initPoseEstimationSURF(int width, int height, std::string data_name, std::string &obj_name);

	virtual void displayResults();

	virtual bool initialize();

	virtual void handleKey(char key);

	virtual void tracking();
};

