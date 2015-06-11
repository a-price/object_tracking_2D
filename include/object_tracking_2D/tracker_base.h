#pragma once

#include <boost/signals2/mutex.hpp>
#include <boost/thread.hpp>
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/filesystem.hpp>

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <omp.h>        // openmp
#include <iostream>
//#include <direct.h> // mkdir()

#include "Camera.h"
#include "PoseEstimationSURF.h"
#include "ObjectModel.h"
#include "EdgeTracker.h"
#include "Timer.h"

//#include <imageReceiver.h>
//#include "sns.h"
//#include <sns.h>

using namespace cv;
using boost::asio::ip::tcp;

class TrackerBase
{
public:
	TrackerBase();

	virtual ~TrackerBase();

	virtual bool initTracker(std::string &obj_name, std::string &cam_name, std::string &intrinsic, std::string &distortion, int width, int height, CvMat* pose_init);

	void printPose(CvMat* p);

	cv::Mat filterOutColor(Mat inputImage, int iLowH, int iLowS, int iLowV, int iHighH, int iHighS, int iHighV);

	void run();

	void saveResultText();

	void saveResultImage();

	struct NET_THREAD_PARAM
	{
		CvMat* pose;
		bool* net;
		bool* run;
		bool* init;
		boost::signals2::mutex* mutex;
	};

	inline int    getWidth()                          { return width_; }
	inline int    getHeight()                         { return height_; }
	inline int    getCannyLow()                       { return th_canny_l_; }
	inline void   setCannyLow(int th)                 { th_canny_l_ = th; }
	inline int    getCannyHigh()                      { return th_canny_h_; }
	inline void   setCannyHigh(int th)                { th_canny_h_ = th; }
	inline float  getSampleStep()                     { return sample_step_; }
	inline void   setSampleStep(float ss)             { sample_step_ = ss; }
	inline int    getMaxSearchDistance()              { return maxd_; }
	inline void   setMaxSearchDistance(int d)         { maxd_ = d; }
	inline bool   getConsideringDullEdges()           { return dulledge_; }
	inline void   setConsideringDullEdges(bool tf)    { dulledge_ = tf; }
	inline bool   getDisplay()                        { return display_; }
	inline void   setDisplay(bool tf)                 { display_ = tf; }
	inline bool   getNetworkMode()                    { return net_; }
	inline void   setNetworkMode(bool tf)             { net_ = tf; }
	inline bool   getSaveResultText()                 { return save_rslt_txt_; }
	inline void   setSaveResultText(bool tf)          { save_rslt_txt_ = tf; }
	inline bool   getSaveResultImage()                { return save_rslt_img_; }
	inline void   setSaveResultImage(bool tf)         { save_rslt_img_ = tf; }
	inline CvMat*   getPose()                         { return pose_; }
	inline void   setPose(CvMat* pose)                { pose_ = pose; }
	inline IplImage*   getResultImage()               { return img_result_; }
	inline IplImage*   getEdgeImage()                 { return img_edge_; }
	inline void   setMinKeypointMatches(int d)        { min_keypoint_matches = d; }
	inline void   setTracking(bool use_tracking)      { use_tracking_ = use_tracking; }
	inline std::string& getSaveResultPath()           { return str_result_path_; }
	inline void   saveKeyframe()                      { saveKeyframe_ = true; }

	bool init_;
	bool setSaveResultPath(std::string& path);

	virtual void tracking() = 0;

	void setColorFilter(int* thresholds)
	{
		hsvFilt = thresholds;
	}

	bool setImage(cv::Mat image);

	virtual void renderResults();

	virtual void displayResults();

	virtual bool initialize();

protected:
	void displayOpenCVInfo()
	{
		/*char* libraries;
		char* modules;
		//cvGetModuleInfo(0, (const char**)&libraries, (const char**)&modules);

		std::cout << "Libraries: " << libraries << std::endl;
		std::cout << "Modules: " << modules << std::endl;*/
	}

	bool initCamera(std::string &cam_name, std::string &intrinsic, std::string &distortion, int width, int height);

	bool initEdgeTracker(int width, int height, CvMat* intrinsic, int maxd, bool limityrot);

	virtual bool initObjectModel(std::string name, int width, int height, CvMat* intrinsic, float sample_step, int maxd, bool dulledge, CEdgeTracker* edge_tracker);

	bool initImages(int width, int height);

	bool getImage();

	virtual void handleKey(char key) = 0;

	std::string handleRequest(std::string& req);

	void networkFunc();

	CCamera* cam_;
	CEdgeTracker* edge_tracker_;
	CObjectModel* obj_model_;

	int width_;
	int height_;
	bool run_;
	bool display_init_result_;
	double th_valid_sample_points_ratio_;
	std::string obj_name_;
	int frame_num_;
	int frame_num_after_init_;
	bool net_;
	bool save_rslt_txt_;  // save estimated pose and time result in txt file
	bool save_rslt_img_;  // save result image as jpg file
	std::ofstream ofs_pose_;
	std::ofstream ofs_time_;
	std::string str_result_path_;

	int th_canny_l_;
	int th_canny_h_;
	float sample_step_;
	int maxd_;
	bool dulledge_;
	float ransac_th_;
	bool limityrot_;
	bool display_;
	bool display_result_;
	bool display_edge_result_;
	bool display_grayscale_image_;

	int smooth_size_;

	IplImage* img_input_;
	IplImage* img_gray_;
	IplImage* img_gray_tracking;
	IplImage* img_mask_;
	IplImage* img_result_;
	IplImage* img_edge_;

	CvMat* pose_;
	CvMat* pose_init_;

	boost::signals2::mutex mutex_; // For syncronization between main function and network thread
	Timer timer_;
	double time_tracking_;
	double time_init_;
	double time_run_;

	int min_keypoint_matches;
	bool use_tracking_;
	bool saveKeyframe_;
	//ImageReceiver rec;
	cv::Mat image;

	int* hsvFilt;
};
