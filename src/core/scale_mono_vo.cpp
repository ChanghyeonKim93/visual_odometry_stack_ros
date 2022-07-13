#include "core/scale_mono_vo.h"

/**
 * @brief Scale mono VO object
 * @details Constructor of a scale mono VO object 
 * @param mode mode == "dataset": dataset mode, mode == "rosbag": rosbag mode. (callback based)
 * @return none
 * @author Changhyeon Kim (hyun91015@gmail.com)
 * @date 10-July-2022
 */
ScaleMonoVO::ScaleMonoVO(std::string mode, std::string directory_intrinsic)
: cam_(nullptr), system_flags_(), dataset_(), frame_prev_(nullptr) {
	std::cout << "Scale mono VO starts\n";
	
	unsigned short a =0;
	printf("%0x\n",a);
	
	// Initialize camera
	cam_ = std::make_shared<Camera>();
	Landmark::cam_ = cam_;
	Frame::cam_    = cam_;

	if(mode == "dataset"){
		std::cout <<"ScaleMonoVO - 'dataset' mode.\n";
		std::string dir_dataset = "D:/#DATASET/kitti/data_odometry_gray";
		std::string dataset_num = "00";

		this->loadCameraIntrinsic_KITTI_IMAGE0(dir_dataset + "/dataset/sequences/" + dataset_num + "/intrinsic.yaml");

		// Get dataset filenames.
		dataset_loader::getImageFileNames_KITTI(dir_dataset, dataset_num, dataset_);

		runDataset(); // run while loop
	}
	else if(mode == "rosbag"){
		std::cout << "ScaleMonoVO - 'rosbag' mode.\n";
		
		this->loadCameraIntrinsic(directory_intrinsic);
		// wait callback ...
	}
	else 
		throw std::runtime_error("ScaleMonoVO - unknown mode.");

	// Initialize feature extractor (ORB-based)
	extractor_ = std::make_shared<FeatureExtractor>();
	int n_bins_u     = params_.feature_extractor.n_bins_u;
	int n_bins_v     = params_.feature_extractor.n_bins_v;
	float THRES_FAST = params_.feature_extractor.thres_fastscore;
	float radius     = params_.feature_extractor.radius;
	extractor_->initParams(cam_->cols(), cam_->rows(), n_bins_u, n_bins_v, THRES_FAST, radius);

	// Initialize feature tracker (KLT-based)
	tracker_ = std::make_shared<FeatureTracker>();

	// Initialize motion estimator
	motion_estimator_ = std::make_shared<MotionEstimator>();

};

/**
 * @brief Scale mono VO destructor
 * @details Destructor of a scale mono VO object 
 * @return none
 * @author Changhyeon Kim (hyun91015@gmail.com)
 * @date 10-July-2022
 */
ScaleMonoVO::~ScaleMonoVO() {
	std::cout << "Scale mono VO is terminated.\n";
};


/**
 * @brief Run scale mono vo on the dataset.
 * @details 낱장 이미지 파일들로 저장되어있는 (ex. KITTI) 데이터에 대해 scale mono vo 알고리즘 구동. 
 * @return void
 * @author Changhyeon Kim (hyun91015@gmail.com)
 * @date 10-July-2022
 */
void ScaleMonoVO::runDataset() {
	
	cv::Mat img0 = cv::imread(dataset_.image_names[0], cv::IMREAD_GRAYSCALE);
	cv::Mat img0f;
	img0.convertTo(img0f, CV_32FC1);
	std::cout << "input image type: " << image_processing::type2str(img0f) << std::endl;
	/*while (true) {

	}*/
};

/**
 * @brief load monocular camera intrinsic parameters from yaml file.
 * @details 카메라의 intrinsic parameter (fx,fy,cx,cy, distortion)을 얻어온다. 
 * @param dir file directory
 * @return void
 * @author Changhyeon Kim (hyun91015@gmail.com)
 * @date 11-July-2022
 */
void ScaleMonoVO::loadCameraIntrinsic_KITTI_IMAGE0(const std::string& dir) {

	cv::FileStorage fs(dir, cv::FileStorage::READ);
	if (!fs.isOpened()) throw std::runtime_error("intrinsic file cannot be found!\n");

	int rows_tmp, cols_tmp;

	cv::Mat cvK_tmp, cvD_tmp;
	rows_tmp = fs["cam0.height"];
	cols_tmp = fs["cam0.width"];
	fs["cam0.K"] >> cvK_tmp;
	fs["cam0.D"] >> cvD_tmp;
	cam_->initParams(cols_tmp, rows_tmp, cvK_tmp, cvD_tmp);
	
	std::cout << " - 'loadCameraIntrinsicMono()' - loaded.\n";
};

/**
 * @brief load monocular camera intrinsic parameters from yaml file.
 * @details 카메라의 intrinsic parameter (fx,fy,cx,cy, distortion)을 얻어온다. 
 * @param dir file directory
 * @return void
 * @author Changhyeon Kim (hyun91015@gmail.com)
 * @date 11-July-2022
 */
void ScaleMonoVO::loadCameraIntrinsic(const std::string& dir) {
	cv::FileStorage fs(dir, cv::FileStorage::READ);
	if (!fs.isOpened()) throw std::runtime_error("intrinsic file cannot be found!\n");

	int rows, cols;
	rows = fs["Camera.height"];	cols = fs["Camera.width"];

	float fx, fy, cx, cy;
	fx = fs["Camera.fx"];	fy = fs["Camera.fy"];
	cx = fs["Camera.cx"];	cy = fs["Camera.cy"];

	float k1,k2,k3,p1,p2;
	k1 = fs["Camera.k1"];	k2 = fs["Camera.k2"];	k3 = fs["Camera.k3"];
	p1 = fs["Camera.p1"];	p2 = fs["Camera.p2"];

	cv::Mat cvK_tmp;
	cvK_tmp = cv::Mat(3,3,CV_32FC1);
	cvK_tmp.at<float>(0,0) = fx;	cvK_tmp.at<float>(0,1) = 0.0f;	cvK_tmp.at<float>(0,2) = cx;
	cvK_tmp.at<float>(1,0) = 0.0f;	cvK_tmp.at<float>(1,1) = fy;	cvK_tmp.at<float>(1,2) = cy;
	cvK_tmp.at<float>(2,0) = 0.0f;	cvK_tmp.at<float>(2,1) = 0.0f;	cvK_tmp.at<float>(2,2) = 1.0f;
	
	cv::Mat cvD_tmp;
	cvD_tmp = cv::Mat(1,5,CV_32FC1);
	cvD_tmp.at<float>(0,0) = k1;
	cvD_tmp.at<float>(0,1) = k2;
	cvD_tmp.at<float>(0,2) = p1;
	cvD_tmp.at<float>(0,3) = p2;
	cvD_tmp.at<float>(0,4) = k3;

	if(cam_ == nullptr) throw std::runtime_error("cam_ is not allocated.");
	cam_->initParams(cols, rows, cvK_tmp, cvD_tmp);

	std::cout << "fx: " << cam_->fx() <<", "
			  << "fy: " << cam_->fy() <<", "
			  << "cx: " << cam_->cx() <<", "
			  << "cy: " << cam_->cy() <<", "
			  << "cols: " << cam_->cols() <<", "
			  << "rows: " << cam_->rows() <<"\n";
	
	std::cout << " - 'loadCameraIntrinsic()' - loaded.\n";
};



/**
 * @brief function to track a new image
 * @details 새로 들어온 이미지의 자세를 구하는 함수. 만약, scale mono vo가 초기화되지 않은 경우, 해당 이미지를 초기 이미지로 설정. 
 * @param img 입력 이미지 (CV_8UC1)
 * @param timestamp 입력 이미지의 timestamp.
 * @return none
 * @author Changhyeon Kim (hyun91015@gmail.com)
 * @date 10-July-2022
 */
void ScaleMonoVO::trackImageMy(const cv::Mat& img, const double& timestamp){

	// 현재 이미지에 대한 새로운 Frame 생성
	FramePtr frame_curr = std::make_shared<Frame>();
	this->saveFrames(frame_curr);
	
	// 이미지 undistort (KITTI라서 할 필요 X)
	bool flag_do_undistort = false;
	cv::Mat img_undist;
	if(flag_do_undistort) cam_->undistort(img, img_undist);
	else img.copyTo(img_undist);

	// frame_curr에 img_undist와 시간 부여
	frame_curr->setImageAndTimestamp(img_undist, timestamp);

	if( !system_flags_.flagVOInit ) { // 초기화 미완료
		if( !system_flags_.flagFirstImageGot ) { // 최초 이미지	
			// Get the first image
			const cv::Mat& I0 = frame_curr->getImage();

			// Extract pixels
			PixelVec       pxvec0;
			LandmarkPtrVec lmvec0;

			extractor_->resetWeightBin();
			extractor_->extractORBwithBinning(I0, pxvec0);
			
			// 초기 landmark 생성
			lmvec0.reserve(pxvec0.size());
			for(auto p : pxvec0) 
				lmvec0.push_back(std::make_shared<Landmark>(p, frame_curr));
			
			// Related Landmark와 tracked pixels를 업데이트
			frame_curr->setPtsSeen(pxvec0);
			frame_curr->setRelatedLandmarks(lmvec0);

			this->saveLandmarks(lmvec0);	

			if( true )
				this->showTracking("img_features", frame_curr->getImage(), pxvec0, PixelVec(), PixelVec());

			// 첫 이미지 업데이트 완료
			system_flags_.flagFirstImageGot = true;
		}
		else { // 최초 첫 이미지는 들어왔으나, 아직 초기화가 되지 않은 상태.
			   // 초기화는 맨 첫 이미지 (첫 키프레임) 대비, 제대로 추적 된 landmark가 60 퍼센트 이상이며, 
			   // 추적된 landmark 각각의 최대 parallax 가 1도 이상인 경우 초기화 완료.	

			// 이전 프레임의 pixels 와 lmvec0을 가져온다.
			const PixelVec&       pxvec0 = frame_prev_->getPtsSeen();
			const LandmarkPtrVec& lmvec0 = frame_prev_->getRelatedLandmarkPtr();
			const cv::Mat&        I0     = frame_prev_->getImage();
			const cv::Mat&        I1     = frame_curr->getImage();
			
			// frame_prev_ 의 lms 를 현재 이미지로 track.
			PixelVec pxvec1_track;
			MaskVec  maskvec1_track;
			tracker_->trackBidirection(I0, I1, pxvec0, params_.feature_tracker.thres_error, params_.feature_tracker.thres_bidirection,
							           pxvec1_track, maskvec1_track);

			// // Tracking 결과를 반영하여 pxvec1_alive, lmvec1_alive를 정리한다.
			PixelVec       pxvec0_alive;
			PixelVec       pxvec1_alive;
			LandmarkPtrVec lmvec1_alive;
			int cnt_alive = 0;
			for(int i = 0; i < pxvec1_track.size(); ++i){
				if( maskvec1_track[i]) {
					pxvec0_alive.push_back(pxvec0[i]);
					pxvec1_alive.push_back(pxvec1_track[i]);
					lmvec1_alive.push_back(lmvec0[i]);
					++cnt_alive;
				}
				else lmvec0[i]->setDead(); // track failed. Dead point.
			}
			std::cout << "# of alive : " << cnt_alive << " / " << maskvec1_track.size() << std::endl;
		
			// 1-point RANSAC 을 이용하여 outlier를 제거한다.
			MaskVec maskvec_1p;
			motion_estimator_->fineInliers1PointHistogram(pxvec0_alive, pxvec1_alive, cam_, maskvec_1p);

			PixelVec       pxvec0_1p;
			PixelVec       pxvec1_1p;
			LandmarkPtrVec lmvec1_1p;
			int cnt_1p = 0;
			for(int i = 0; i < maskvec_1p.size(); ++i){
				if( maskvec_1p[i]) {
					pxvec0_1p.push_back(pxvec0_alive[i]);
					pxvec1_1p.push_back(pxvec1_alive[i]);
					lmvec1_1p.push_back(lmvec1_alive[i]);
					++cnt_1p;
				}
				else lmvec1_alive[i]->setDead(); // track failed. Dead point.
			}

			// pts0 와 pts1을 이용, 5-point algorithm 으로 모션 & X0 를 구한다.
			// 만약 mean optical flow의 중간값이 약 1 px 이하인 경우, 정지 상태로 가정하고 스킵.
			MaskVec maskvec_inlier(pxvec0_1p.size());
			PointVec X0_inlier(pxvec0_1p.size());
			Rot3 R10;
			Pos3 t10;
			if( !motion_estimator_->calcPose5PointsAlgorithm(pxvec0_1p, pxvec1_1p, cam_, R10, t10, X0_inlier, maskvec_inlier) ) {
				throw std::runtime_error("calcPose5PointsAlgorithm() is failed.");
			}
			
			// Frame_curr의 자세를 넣는다.
			PoseSE3 Tck; Tck << R10, t10, 0.0f, 0.0f, 0.0f, 1.0f;
			frame_curr->setPose(Tck);

			// tracking, 5p algorithm, newpoint 모두 합쳐서 살아남은 점만 frame_curr에 넣는다
			PixelVec       pxvec0_final;
			PixelVec       pxvec1_final;
			LandmarkPtrVec lmvec1_final;
			cnt_alive = 0;
			int cnt_parallax_ok = 0;
			for(int i = 0; i < pxvec0_1p.size(); ++i){
				if( maskvec_inlier[i] ) {
					lmvec1_1p[i]->addObservationAndRelatedFrame(pxvec1_1p[i], frame_curr);
					if(lmvec1_1p[i]->getMaxParallax() > params_.map_update.thres_parallax) {
						++cnt_parallax_ok;
						lmvec1_1p[i]->set3DPoint(X0_inlier[i]);
					}
					pxvec0_final.push_back(pxvec0_1p[i]);
					pxvec1_final.push_back(pxvec1_1p[i]);
					lmvec1_final.push_back(lmvec1_1p[i]);
					++cnt_alive;
				}
				else lmvec1_1p[i]->setDead(); // 5p algorithm failed. Dead point.
			}
			std::cout << "size check (5p): " << pxvec0_final.size() << ", " << pxvec1_final.size() << ", " << lmvec1_final.size() <<std::endl;

			// lmvec1_final 중, depth가 복원되지 않은 경우 복원해준다.
			std::cout <<" parallax ok : " << cnt_parallax_ok << " / " << cnt_alive << std::endl;

			// 빈 곳에 특징점 pts1_new 를 추출한다.
			PixelVec pxvec1_new;
			extractor_->updateWeightBin(pxvec1_final); // 이미 pts1가 있는 곳은 제외.
			extractor_->extractORBwithBinning(frame_curr->getImage(), pxvec1_new);

			if( true )
				this->showTracking("img_features", frame_curr->getImage(), pxvec0_final, pxvec1_final, pxvec1_new);
			
			if( pxvec1_new.size() > 0 ){
				// 새로운 특징점은 새로운 landmark가 된다.
				for(auto p1_new : pxvec1_new) {
					LandmarkPtr ptr = std::make_shared<Landmark>(p1_new, frame_curr);
					pxvec1_final.emplace_back(p1_new);
					lmvec1_final.push_back(ptr);
					this->saveLandmarks(ptr);	
				}
			}

			// lms1와 pts1을 frame_curr에 넣는다.
			frame_curr->setPtsSeen(pxvec1_final);
			frame_curr->setRelatedLandmarks(lmvec1_final);

			// 초기화를 완료할지 판단
			// lmvec1_final가 최초 관측되었던 (keyframe) 
			bool initialization_done = false;
			int n_lms_keyframe  = keyframe_->getRelatedLandmarkPtr().size();
			int n_lms_alive     = 0;
			float mean_parallax = 0;
			for(int i = 0; i < lmvec1_final.size(); ++i){
				const LandmarkPtr& lm = lmvec1_final[i];
				if( lm->getRelatedFramePtr().front()->getID() == 0 ) {
					++n_lms_alive;
					mean_parallax += lm->getMaxParallax();
				}
			}
			mean_parallax /= (float)n_lms_alive;

			if(mean_parallax > params_.keyframe_update.thres_mean_parallax*110000)
				initialization_done = true;
			
			if(initialization_done){ // lms_tracked_ 의 평균 parallax가 특정 값 이상인 경우, 초기화 끝. 
				// lms_tracked_를 업데이트한다. 
				system_flags_.flagVOInit = true;

				std::cout << "VO initialzed!\n";
			}
		}
	}
	else { // VO initialized. Do track the new image.

	}

	// Replace the 'frame_prev_' with 'frame_curr'
	frame_prev_ = frame_curr;
};

// /**
//  * @brief function to track a new image
//  * @details 새로 들어온 이미지의 자세를 구하는 함수. 만약, scale mono vo가 초기화되지 않은 경우, 해당 이미지를 초기 이미지로 설정. 
//  * @param img 입력 이미지 (CV_8UC1)
//  * @param timestamp 입력 이미지의 timestamp.
//  * @return none
//  * @author Changhyeon Kim (hyun91015@gmail.com)
//  * @date 10-July-2022
//  */
// void ScaleMonoVO::trackImage(const cv::Mat& img, const double& timestamp){
// 	// 현재 이미지에 대한 새로운 Frame 생성
// 	FramePtr frame_curr = std::make_shared<Frame>();
// 	all_frames_.push_back(frame_curr);

// 	// 이미지 undistort (KITTI라서 할 필요 X)
// 	bool flag_do_undistort = false;
// 	cv::Mat img_undist;
// 	if(flag_do_undistort) cam_->undistort(img, img_undist);
// 	else img.copyTo(img_undist);

// 	// frame_curr에 img_undist와 시간 부여
// 	frame_curr->setImageAndTimestamp(img_undist, timestamp);

// 	if( !system_flags_.flagVOInit ) { // 초기화 미완료
// 		if( !system_flags_.flagFirstImageGot ) { // 최초 이미지
// 			// It is the first keyframe
// 			frame_curr->makeThisKeyframe();
// 			this->updateKeyframe(frame_curr);
			
// 			// Get the first image
// 			const cv::Mat& I0 = frame_curr->getImage();

// 			// Extract pixels
// 			PixelVec       pxvec0;
// 			LandmarkPtrVec lmvec0;

// 			extractor_->resetWeightBin();
// 			extractor_->extractORBwithBinning(I0, pxvec0);
			
// 			// Set initial landmarks
// 			lmvec0.reserve(pxvec0.size());
// 			for(auto p : pxvec0) 
// 				lmvec0.push_back(std::make_shared<Landmark>(p, frame_curr));
			
// 			// Related Landmark와 tracked pixels를 업데이트
// 			frame_curr->setRelatedLandmarks(lmvec0);
// 			frame_curr->setPtsSeen(pxvec0);

// 			if( true )
// 				this->showTracking("img_features", frame_curr->getImage(), pxvec0, PixelVec(), PixelVec());

// 			// 첫 이미지 업데이트 완료
// 			system_flags_.flagFirstImageGot = true;
// 		}
// 		else { // 최초 첫 이미지는 들어왔으나, 아직 초기화가 되지 않은 상태.
// 			   // 초기화는 맨 첫 이미지 (첫 키프레임) 대비, 제대로 추적 된 landmark가 60 퍼센트 이상이며, 
// 			   // 추적된 landmark 각각의 최대 parallax 가 1도 이상인 경우 초기화 완료.
			
// 			// 이전 프레임의 pixels 와 lmvec0을 가져온다.
// 			const PixelVec&       pxvec0 = frame_prev_->getPtsSeen();
// 			const LandmarkPtrVec& lmvec0 = frame_prev_->getRelatedLandmarkPtr();
			
// 			// frame_prev_ 의 lms 를 현재 이미지로 track.
// 			PixelVec pxvec1_track;
// 			MaskVec  maskvec1_track;
// 			tracker_->trackBidirection(frame_prev_->getImage(), frame_curr->getImage(), pxvec0, params_.feature_tracker.thres_error, params_.feature_tracker.thres_bidirection,
// 							           pxvec1_track, maskvec1_track);

// 			// // Tracking 결과를 반영하여 pxvec1_alive, lmvec1_alive를 정리한다.
// 			PixelVec       pxvec0_alive;
// 			PixelVec       pxvec1_alive;
// 			LandmarkPtrVec lmvec1_alive;
// 			int cnt_alive = 0;
// 			for(int i = 0; i < pxvec1_track.size(); ++i){
// 				if( maskvec1_track[i]) {
// 					pxvec0_alive.push_back(pxvec0[i]);
// 					lmvec1_alive.push_back(lmvec0[i]);
// 					pxvec1_alive.push_back(pxvec1_track[i]);
// 					++cnt_alive;
// 				}
// 				else lmvec0[i]->setDead(); // track failed. Dead point.
// 			}
// 			std::cout << "# of alive : " << cnt_alive << " / " << maskvec1_track.size() << std::endl;
			
// 			// pts0 와 pts1을 이용, 5-point algorithm 으로 모션 & X0 를 구한다.
// 			MaskVec maskvec_inlier(pxvec0_alive.size());
// 			PointVec X0_inlier(pxvec0_alive.size());
// 			Rot3 R10;
// 			Pos3 t10;
// 			if( !motion_estimator_->calcPose5PointsAlgorithm(pxvec0_alive, pxvec1_alive, cam_, R10, t10, X0_inlier, maskvec_inlier) ) {
// 				throw std::runtime_error("calcPose5PointsAlgorithm() is failed.");
// 			}

// 			// Frame_curr의 자세를 넣는다.
// 			PoseSE3 Tck; Tck << R10, t10, 0.0f, 0.0f, 0.0f, 1.0f;
// 			frame_curr->setPose(Tck);

// 			// tracking, 5p algorithm, newpoint 모두 합쳐서 살아남은 점만 frame_curr에 넣는다
// 			LandmarkPtrVec lmvec1_final;
// 			PixelVec       pxvec1_final;
// 			cnt_alive = 0;
// 			int cnt_parallax_ok = 0;
// 			std::cout << pxvec1_alive.size() << "," << maskvec_inlier.size() << std::endl;
// 			for(int i = 0; i < pxvec1_alive.size(); ++i){
// 				if( maskvec_inlier[i] ) {
// 					lmvec1_alive[i]->addObservationAndRelatedFrame(pxvec1_alive[i], frame_curr);
// 					if(lmvec1_alive[i]->getMaxParallax() > params_.map_update.thres_parallax) {
// 						++cnt_parallax_ok;
// 						lmvec1_alive[i]->set3DPoint(X0_inlier[i]);
// 					}
// 					lmvec1_final.push_back(lmvec1_alive[i]);
// 					pxvec1_final.push_back(pxvec1_alive[i]);
// 					++cnt_alive;
// 				}
// 				else lmvec1_alive[i]->setDead(); // 5p algorithm failed. Dead point.
// 			}

// 			// lmvec1_final 중, depth가 복원되지 않은 경우 복원해준다.
// 			std::cout <<" parallax ok : " << cnt_parallax_ok << " / " << cnt_alive << std::endl;

// 			// 빈 곳에 특징점 pts1_new 를 추출한다.
// 			PixelVec pxvec1_new;
// 			extractor_->updateWeightBin(pxvec1_final); // 이미 pts1가 있는 곳은 제외.
// 			extractor_->extractORBwithBinning(frame_curr->getImage(), pxvec1_new);

// 			if( pxvec1_new.size() > 0 ){
// 				// 새로운 특징점은 새로운 landmark가 된다.
// 				for(auto p1_new : pxvec1_new) {
// 					LandmarkPtr ptr = std::make_shared<Landmark>(p1_new, frame_curr);
// 					lmvec1_final.push_back(ptr);
// 					pxvec1_final.emplace_back(p1_new);
// 				}
// 				std::cout << "pts1_new size: " << pxvec1_new.size() <<", pxvec1_final size: " << pxvec1_final.size() << std::endl;
// 			}

// 			// lms1와 pts1을 frame_curr에 넣는다.
// 			frame_curr->setRelatedLandmarks(lmvec1_final);
// 			frame_curr->setPtsSeen(pxvec1_final);

// 			if( true )
// 				this->showTracking("img_features", frame_curr->getImage(), pxvec0_alive, pxvec1_final, pxvec1_new);
			

// 			// 초기화를 완료할지 판단
// 			// lmvec1_final가 최초 관측되었던 (keyframe) 
// 			bool initialization_done = false;
// 			int n_lms_keyframe  = keyframe_->getRelatedLandmarkPtr().size();
// 			int n_lms_alive     = 0;
// 			float mean_parallax = 0.0f;
// 			for(int i = 0; i < lmvec1_final.size(); ++i){
// 				const LandmarkPtr& lm = lmvec1_final[i];
// 				if( lm->getRelatedFramePtr().front()->getID() == 0 ) {
// 					++n_lms_alive;
// 					mean_parallax += lm->getMaxParallax();
// 				}
// 			}
// 			mean_parallax /= (float)n_lms_alive;

// 			if(mean_parallax > params_.keyframe_update.thres_mean_parallax)
// 				initialization_done = true;
			
// 			if(initialization_done){ // lms_tracked_ 의 평균 parallax가 특정 값 이상인 경우, 초기화 끝. 
// 				// lms_tracked_를 업데이트한다. 
// 				system_flags_.flagVOInit = true;

// 				std::cout << "VO initialzed!\n";
// 			}
// 		}
// 	}
// 	else { // VO initialized. Do track the new image.
// 		const double dt = timestamp - frame_prev_->getTimestamp();
// 		std::cout << "dt_img: " << dt << " sec." << std::endl;
		
// 		// Pose prior 구하기
// 		PoseSE3 Twc_m2 = all_frames_[all_frames_.size()-2]->getPose();
// 		PoseSE3 Twc_m1 = all_frames_.back()->getPose();
// 		PoseSE3 dT_m2m1 = Twc_m2.inverse()*Twc_m1;

// 		PoseSE3 Twc_prior = Twc_m1*dT_m2m1;

// 		// 이전 프레임의 pixels 와 lmvec0을 가져온다.
// 		const PixelVec&       pxvec0 = frame_prev_->getPtsSeen();
// 		const LandmarkPtrVec& lmvec0 = frame_prev_->getRelatedLandmarkPtr();

// 		LandmarkPtrVec lmvec0_has_depth;
// 		LandmarkPtrVec lmvec0_no_depth;
// 		PixelVec       pxvec1_prior(lmvec0.size());
// 		for(int i = 0; i < lmvec0.size(); ++i){
// 			const LandmarkPtr& lm = lmvec0[i];
// 			if(lm->getTriangulated()){
// 				const Point& Xw = lm->get3DPoint();
// 				Pixel p_prior;
// 				Point X_prior = Twc_prior.block<3,3>(0,0)*Xw + Twc_prior.block<3,1>(0,3);
// 				float invz = 1.0f/X_prior(2);
// 				p_prior.x = cam_->fx()*X_prior(0)*invz + cam_->cx();
// 				p_prior.y = cam_->fy()*X_prior(1)*invz + cam_->cy();
				
// 				lmvec0_has_depth.push_back(lm);
// 				pxvec1_prior[i] = p_prior;
// 			}
// 			else {
// 				lmvec0_no_depth.push_back(lm);
// 				pxvec1_prior[i] = pxvec0[i];
// 			}
// 		}
		
// 		// frame_prev_ 의 lms 를 현재 이미지로 track.
// 		PixelVec pxvec1_track;
// 		MaskVec  maskvec1_track;
// 		tracker_->trackBidirectionWithPrior(frame_prev_->getImage(), frame_curr->getImage(), pxvec0, pxvec1_prior, params_.feature_tracker.thres_error, params_.feature_tracker.thres_bidirection,
// 											pxvec1_track, maskvec1_track);

// 		// Tracking 결과를 반영하여 pxvec1_alive, lmvec1_alive를 정리한다.
// 		PixelVec       pxvec0_alive;
// 		PixelVec       pxvec1_alive;
// 		LandmarkPtrVec lmvec1_alive;
// 		int cnt_alive = 0;
// 		for(int i = 0; i < pxvec1_track.size(); ++i){
// 			if( maskvec1_track[i]) {
// 				pxvec0_alive.push_back(pxvec0[i]);
// 				lmvec1_alive.push_back(lmvec0[i]);
// 				pxvec1_alive.push_back(pxvec1_track[i]);
// 				++cnt_alive;
// 			}
// 			else lmvec0[i]->setDead(); // track failed. Dead point.
// 		}
// 		std::cout << "# of alive : " << cnt_alive << " / " << maskvec1_track.size() << std::endl;
		
// 		// PnP 알고리즘을 이용해 모션을 구한다.
// 		// MaskVec maskvec_pnpinlier;
// 		// Rot3 Rwc_pnp = Twc_prior.block<3,3>(0,0); // initialize
// 		// Pos3 twc_pnp = Twc_prior.block<3,1>(0,3);
// 		// if( !motion_estimator_->calcPosePnPAlgorithm(Xw, pxvec1_alive, cam_, Rwc_pnp, twc_pnp, maskvec_pnpinlier)){
// 		// 	throw std::runtime_error("calcPosePnPAlgorithm() is failed.");
// 		// }

// 		// Frame의 자세를 넣는다.
// 		// PoseSE3 Tck; Tck << R10, t10, 0.0f, 0.0f, 0.0f, 1.0f;
// 		// frame_curr->setPose(Tck);

// 		// inlier만 남긴다.

// 		// inlier 중, parallax 조건 만족 && !isTriangulated 점에 대해 깊이를 복원한다. (world 기준으로) 


// 		// 빈 곳에 점을 추출한다.

// 		// Frame에 related point 와 모든 것을 추가한다.
// 	}

// 	// Add the newly incoming frame into the frame stack
// 	std::cout << "The number of all frames: " << all_frames_.size() << std::endl;

// 	// Replace the 'frame_prev_' with 'frame_curr'
// 	frame_prev_ = frame_curr;
// 	std::cout << "frame prev is updated \n";
// };


/**
 * @brief Update keyframe with an input frame
 * @details 새로운 frame으로 Keyframe을 업데이트 & all_keyframes_ 에 저장
 * @param frame Keyframe이 될 frame
 * @return void
 * @author Changhyeon Kim (hyun91015@gmail.com)
 * @date 12-July-2022
 */
void ScaleMonoVO::updateKeyframe(const FramePtr& frame){
	keyframe_ = frame;
	this->all_keyframes_.push_back(keyframe_);
};

void ScaleMonoVO::saveLandmarks(const LandmarkPtrVec& lms){
	for(auto lm : lms)	
		all_landmarks_.push_back(lm);

	std::cout << "# of all accumulated landmarks: " << all_landmarks_.size() << std::endl;
};

void ScaleMonoVO::saveLandmarks(const LandmarkPtr& lm){
	all_landmarks_.push_back(lm);
	std::cout << "# of all accumulated landmarks: " << all_landmarks_.size() << std::endl;
};

void ScaleMonoVO::saveFrames(const FramePtrVec& frames){
	for(auto f : frames)
		all_frames_.push_back(f);
	std::cout << "# of all accumulated frames   : " << all_frames_.size() << std::endl;
};

void ScaleMonoVO::saveFrames(const FramePtr& frame){
	all_frames_.push_back(frame);
	std::cout << "# of all accumulated frames   : " << all_frames_.size() << std::endl;
};

void ScaleMonoVO::showTracking(const std::string& window_name, const cv::Mat& img, const PixelVec& pts0, const PixelVec& pts1, const PixelVec& pts1_new){
	cv::namedWindow(window_name);
	cv::Mat img_draw;
	img.copyTo(img_draw);
	cv::cvtColor(img_draw, img_draw, CV_GRAY2RGB);
	for(int i = 0; i < pts1.size(); ++i){
		cv::line(img_draw,pts0[i],pts1[i], cv::Scalar(0,255,255),1);
	}
	for(int i = 0; i < pts0.size(); ++i) {
		cv::circle(img_draw, pts0[i], 3.0, cv::Scalar(0,0,0),2); // alived magenta
		cv::circle(img_draw, pts0[i], 2.0, cv::Scalar(255,0,255),1); // alived magenta
	}
	for(int i = 0; i < pts1.size(); ++i){
		cv::circle(img_draw, pts1[i], 3.0, cv::Scalar(0,0,0),2); // green tracked points
		cv::circle(img_draw, pts1[i], 2.0, cv::Scalar(0,255,0),1); // green tracked points
	}
	for(int i = 0; i < pts1_new.size(); ++i){
		cv::circle(img_draw, pts1_new[i], 3.0, cv::Scalar(0,0,0), 2); // blue new points
		cv::circle(img_draw, pts1_new[i], 2.0, cv::Scalar(255,0,0),1); // blue new points
	}
	
	cv::imshow(window_name, img_draw);
	cv::waitKey(10);
};
