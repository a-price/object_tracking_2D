#pragma once

#include "tracker_pf.h"

class TextureParticleFilterTracker : public ParticleFilterTracker
{
public:
	TextureParticleFilterTracker();

	virtual ~TextureParticleFilterTracker();

	virtual bool initTracker(std::string &obj_name, std::string &cam_name, std::string &intrinsic, std::string &distortion, int width, int height, CvMat* pose_init);

protected:
	CPoseEstimationSURF* pe_surf_;
	bool init_keyframes_;

	virtual bool initObjectModel(std::string name, int width, int height, CvMat* intrinsic, float sample_step, int maxd, bool dulledge, CEdgeTracker* edge_tracker);

	bool initPoseEstimationSURF(int width, int height, std::string data_name, std::string &obj_name);


	bool initKeyframes();

	virtual void displayResults();

	virtual bool initialize();

	virtual void handleKey(char key);

	virtual void tracking();
};



