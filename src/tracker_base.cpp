#include "object_tracking_2D/tracker_base.h"

TrackerBase::TrackerBase()
    : cam_(NULL)
	, edge_tracker_(NULL)
	, obj_model_(NULL)
	, th_canny_l_(100) //100
	, th_canny_h_(120) //120
	, sample_step_(0.005f)
	, maxd_(32)
	, dulledge_(false)
	, ransac_th_(0.0f)
	, limityrot_(false)
	, run_(true)
	, init_(true)
	, th_valid_sample_points_ratio_(0.01)
	, img_input_(NULL)
	, img_gray_(NULL)
	, img_gray_tracking(NULL)
	, img_result_(NULL)
	, img_edge_(NULL)
	, display_(true)    , display_result_(true)
	, display_init_result_(false)
	, display_edge_result_(true)
	, display_grayscale_image_(false)
	, smooth_size_(1)
	, obj_name_("")
	, frame_num_(0)
	, frame_num_after_init_(0)
	, hsvFilt(NULL)
	, net_(false)
	, save_rslt_txt_(false)
	, save_rslt_img_(false)
	, str_result_path_("result")
	, time_tracking_(0.f)
	, time_init_(0.f)
	, time_run_(0.f)
{
	pose_ = cvCreateMat(4, 4, CV_32F);
	pose_init_ = cvCreateMat(4, 4, CV_32F);
	cvSetIdentity(pose_);

	displayOpenCVInfo();
}

TrackerBase::~TrackerBase()
{
	if(cam_)            delete cam_;
	if(edge_tracker_)   delete edge_tracker_;
	if(obj_model_)      delete obj_model_;

	// 'img_input_' is aleady released
	if(img_gray_)       cvReleaseImage(&img_gray_);
	if(img_gray_tracking)       cvReleaseImage(&img_gray_tracking);
	if(img_result_)     cvReleaseImage(&img_result_);
	if(img_edge_)       cvReleaseImage(&img_edge_);

	cvReleaseMat(&pose_);
	cvReleaseMat(&pose_init_);

	if(ofs_pose_.is_open())      ofs_pose_.close();
	if(ofs_time_.is_open())      ofs_time_.close();
}

bool TrackerBase::initTracker(std::string &obj_name, std::string &cam_name, std::string &intrinsic, std::string &distortion, int width, int height, CvMat* pose_init)
{
	obj_name_ = obj_name;
	width_ = width;
	height_ = height;
	initCamera(cam_name, intrinsic, distortion, width, height);
	initEdgeTracker(width, height, cam_->getIntrinsicParams(), maxd_, limityrot_);
	initObjectModel(obj_name, width, height, cam_->getIntrinsicParams(), sample_step_, maxd_, dulledge_, edge_tracker_);
	initImages(width, height);

	if(net_)
	{
		// create a network thread to handle network commands
		boost::thread(&TrackerBase::networkFunc, this);
	}

	if(save_rslt_txt_)
	{
		if(!ofs_pose_.is_open())
			ofs_pose_.open((str_result_path_ + std::string("/pose.txt")).c_str());
		if(!ofs_time_.is_open())
			ofs_time_.open((str_result_path_ + std::string("/time.txt")).c_str());
	}

	cvCopy(pose_init, pose_init_);
	cvCopy(pose_init_, pose_);

	return (true);
}

void TrackerBase::printPose(CvMat* p)
{
	for(int cur_row = 0; cur_row<4; cur_row++)
	{
		printf("[%1.3f %1.3f %1.3f %1.3f]\n", CV_MAT_ELEM(*p, float, cur_row, 0), CV_MAT_ELEM(*p, float, cur_row, 1) ,CV_MAT_ELEM(*p, float, cur_row, 2), CV_MAT_ELEM(*p, float, cur_row, 3));
	}
	return;
}

cv::Mat TrackerBase::filterOutColor(Mat inputImage, int iLowH, int iLowS, int iLowV, int iHighH, int iHighS, int iHighV)
{
	Mat imgHSV;
	Mat imgThresholded;
	cvtColor(inputImage, imgHSV, COLOR_BGR2HSV);
	inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image

	return imgThresholded;
}

void TrackerBase::run()
{
	if(display_)
	{
		cvNamedWindow("Result");
		cvNamedWindow("Edge");
		cvNamedWindow("Initialization");
	}

	while(run_)
	{
		// Wait if network flag is on
		if(net_)
		{
			handleKey(cvWaitKey(1));
			continue;
		}

		// capture or load an image
		if(!getImage())
			break;

		//timer_.start();

		if(init_) initialize();

		//time_init_ = timer_.printTimeMilliSec("initializing");

		// filter color from image
		if(hsvFilt!=NULL)
		{
			IplImage* tmp = new IplImage(filterOutColor(img_input_, hsvFilt[0], hsvFilt[1], hsvFilt[2], hsvFilt[3], hsvFilt[4], hsvFilt[5]));
			img_gray_tracking = cvCloneImage(tmp);
			//cvCvtColor(img_gray_tracking, img_result_, CV_GRAY2RGB);
		}
		else
		{
			img_gray_tracking = cvCloneImage(img_gray_);
		}

		// do processing
		//timer_.start();

		if(!init_)
		{
			if(!use_tracking_)
			{
				init_ = true; //NO TRACKING if true
			}
			tracking();
		}

		//time_tracking_ = timer_.printTimeMilliSec("tracking");

		if(display_)
			displayResults();

		if(save_rslt_txt_)
			saveResultText();

		if(save_rslt_img_)
			saveResultImage();

		// release image if it's loaded from file
		if(!cam_->IsCamera()) cvReleaseImage(&img_input_);

		char key = (char)cvWaitKey(5); // get keyboard input
		handleKey(key);

		frame_num_++;
		frame_num_after_init_++;
	}

	if(display_)
	{
		cvDestroyWindow("Result");
		cvDestroyWindow("Edge");
		cvDestroyWindow("Initialization");
	}
}

void TrackerBase::saveResultText()
{
	if(ofs_pose_.is_open())
	{
		for(int r = 0; r < 4; r++)
			for(int c = 0; c < 4; c++)
				ofs_pose_ << std::fixed << CV_MAT_ELEM(*pose_, float, r, c) << '\t';
		ofs_pose_ << std::endl;
	}
	else
		std::cerr << "ofs_pose_ is not opened." << std::endl;

	ofs_time_ << setprecision(8) << time_init_ + time_tracking_ << std::endl;
}

void TrackerBase::saveResultImage()
{
	std::stringstream ss;
	ss << str_result_path_ << "/" << "track" << std::setw(4) << std::setfill('0') << frame_num_ << ".jpg";
	cvSaveImage(ss.str().c_str(), img_result_);
}

bool TrackerBase::setSaveResultPath(std::string& path)
{
	if(mkdir(path.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH) == -1 && errno != EEXIST)
	{
		std::cerr << "Cannot create " << path << " directory for saving results." << std::endl;
		return false;
	}
	str_result_path_ = path;
	return true;
}

bool TrackerBase::setImage(cv::Mat image)
{
	IplImage copy = image;
	img_input_ = static_cast<IplImage *>(&copy);

	cvCvtColor(img_input_, img_gray_, CV_RGB2GRAY);
	cvCvtColor(img_gray_, img_result_, CV_GRAY2RGB);
	return true;
}

void TrackerBase::renderResults()
{
	CvScalar color = cvScalar(0,0,255);
	obj_model_->displayPoseLine(img_result_, pose_, color, 1, false);
	obj_model_->displaySamplePointsAndErrors(img_edge_);
}

void TrackerBase::displayResults()
{
	renderResults();
	cvShowImage("Result", img_result_);
	cvShowImage("Edge", img_edge_);
}

bool TrackerBase::initialize()
{
	if(!init_)
	{
		std::cerr << "Initialization flag is not set." << std::endl;
	}

	frame_num_after_init_ = 0;
	return false;
}

bool TrackerBase::getImage()
{
	img_input_ = cam_->getImage();
	if(img_input_ == NULL) return false;
	if(img_input_->nChannels == 1)
	{
		cvCopy(img_input_, img_gray_);
		cvCvtColor(img_gray_, img_result_, CV_GRAY2RGB);
	}
	else
	{
		cvCvtColor(img_input_, img_gray_, CV_RGB2GRAY);
		//cvCopy(img_input_, img_result_);
		cvCvtColor(img_gray_, img_result_, CV_GRAY2RGB);
	}

	return true;
}

bool TrackerBase::initCamera(std::string &cam_name, std::string &intrinsic, std::string &distortion, int width, int height)
{
	if(cam_) delete cam_;
	if(cam_name.compare("normal") == 0 || cam_name.compare("fire-i") == 0 || cam_name.compare("flea") == 0 || cam_name.compare("openni") == 0 || cam_name.compare("ach") == 0)
		cam_ = new CCamera(cam_name, intrinsic, distortion, width, height);
	else  // assume 'cam_name' is a path to an image sequence
	{
		std::string jpg = "jpg";
		cam_ = new CCamera(cam_name, false, 0, intrinsic, distortion, jpg);
	}
	return (true);
}

bool TrackerBase::initEdgeTracker(int width, int height, CvMat* intrinsic, int maxd, bool limityrot)
{
	if(edge_tracker_) delete edge_tracker_;
	edge_tracker_ = new CEdgeTracker(width, height, intrinsic, maxd, limityrot);
	return (true);
}

bool TrackerBase::initObjectModel(std::string name, int width, int height, CvMat* intrinsic, float sample_step, int maxd, bool dulledge, CEdgeTracker* edge_tracker)
{
	if(obj_model_) delete obj_model_;
	obj_model_ = new CObjectModel(name, width, height, intrinsic, sample_step, maxd, dulledge, edge_tracker);
	obj_model_->loadObjectCADModel(name);
	return (true);
}

bool TrackerBase::initImages(int width, int height)
{
	if(img_gray_) cvReleaseImage(&img_gray_);
	img_gray_ = cvCreateImage(cvSize(width, height), 8, 1);
	if(img_result_) cvReleaseImage(&img_result_);
	img_result_ = cvCreateImage(cvSize(width, height), 8, 3);
	if(img_edge_) cvReleaseImage(&img_edge_);
	img_edge_ = cvCreateImage(cvSize(width, height), 8, 3);

	return (true);
}

std::string TrackerBase::handleRequest(std::string& req)
{
	if(req.substr(0, 4).compare("1004") == 0) // request current pose (SE(3) in meter)
	{
		net_ = false;
		mutex_.lock();
		std::stringstream ss;
		ss << "1 "; // means success
		for(int i=0; i<16; i++)
			ss << std::fixed << std::setprecision(6) << CV_MAT_ELEM(*pose_, float, i/4, i%4) << " ";
		ss << std::endl;
		mutex_.unlock();
		return ss.str();
	}
	else if(req.substr(0, 4).compare("1005") == 0) // start without initialization
	{
		net_ = false;
		init_ = false;
		return std::string("1\n");
	}
	else if(req.substr(0, 4).compare("2000") == 0) // request re-initialization
	{
		init_ = true;
		return std::string("1\n");
	}
	else if(req.substr(0, 4).compare("3000") == 0) // request to end tracker
	{
		run_ = false;
		return std::string("1\n");
	}

	return std::string("");
}

void TrackerBase::networkFunc()
{
	std::cout << "Starting up a network server" << std::endl;

	try
	{
		boost::asio::io_service io_service;

		tcp::acceptor acceptor(io_service, tcp::endpoint(tcp::v4(), 1701));

		tcp::socket socket(io_service);
		acceptor.accept(socket);

		for (;;)
		{
			boost::array<char, 128> buf;
			boost::system::error_code error;
			size_t len = socket.read_some(boost::asio::buffer(buf), error);
			if (error == boost::asio::error::eof)
				break; // Connection closed cleanly by peer.
			else if (error)
				throw boost::system::system_error(error); // Some other error.

			std::stringstream ss;
			ss.write(buf.data(), len);
			std::string tmp = ss.str();
			std::string msg = handleRequest(tmp);

			boost::system::error_code ignored_error;
			boost::asio::write(socket, boost::asio::buffer(msg), boost::asio::transfer_all(), ignored_error);
		}
	}
	catch (std::exception& e)
	{
		std::cerr << e.what() << std::endl;
	}

	return;
}

