/** \file vde_camera_test.cpp
 *  \date 2021-11-18
 *
 * For decoding video streams using ffmpeg see "send/receive encoding and
 * decoding API overview"
 * (https://ffmpeg.org/doxygen/3.2/group__lavc__encdec.html)
 *
 * Use the following command to view saved video file:
 * > ffplay -flags2 showall -f h264 h20t_video.h264
 */

#include <pthread.h>

#include <dji_camera_image.hpp>
#include <dji_camera_manager.hpp>
#include <dji_linux_helpers.hpp>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>

// definition is not loaded from cmake automagically
// hence definition is done manually in
// c_cpp_properties.json
#ifdef OPEN_CV_INSTALLED
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>

#include "opencv2/opencv.hpp"
// using namespace cv;
#endif

#include <libavcodec/avcodec.h>

// using namespace DJI::OSDK;
// using namespace DJI::OSDK::Telemetry;

class AodCamera
{
public:
	/*! @brief The ShootPhoto mode itself can have several modes. The default
	 * value is SINGLE.
	 */
	enum CameraState
	{
		/** \brief Camera is busy taking picture. */
		CAMERA_BUSY,
		/** \brief Camera is ready to take picture. */
		CAMERA_READY
	};

	/** \brief Internal state of the camera.
	 *
	 * 	This state is altered during shootPhotoCallback, but since access of
	 *	variable should be atomic a mutex lock shouldn't be required.
	 */
	CameraState state;

	/** \brief Flag indicating if a file list request has completed. */
	bool file_list_updated;

	/** \brief Flag indicating that the file list has changed in size or type.
	 */
	bool file_list_changed;

	/** \brief Flag indicating that the file data transfer has finished.
	 *
	 * Query last_err to check if transfer was successfull.
	 */
	bool file_data_transfer_done;

	DJI::OSDK::FilePackage file_list;

	/**
	 * @brief File handle for video recording.
	 *
	 */
	FILE* p_video_file;

	/** \brief Constructor.
	 *
	 * On Object instantiation init is called to initialize the camera.
	 *
	 * \param cm Pointer to CameraManger from vehicle object.
	 * \param as Pointer to AdvancedSensing from vehicle object.
	 */
	AodCamera(DJI::OSDK::CameraManager* cm, DJI::OSDK::AdvancedSensing* as);

	/** \brief Constructor.
	 *
	 * On Object instantiation init is called to initialize the camera.
	 *
	 * \param cm Pointer to CameraManger from vehicle object.
	 */
	AodCamera(DJI::OSDK::CameraManager* cm) : AodCamera(cm, NULL){};

	/** \brief Initialize Zenmuse H20T at payload position 0.
	 *
	 * \returns true if initialization was successful, false otherwise.
	 */
	bool init();

	/** \brief Check if camera is initialized.
	 *
	 * \returns true if camera is initialized, false otherwise.
	 */
	bool isInitialized();

	/** \brief Check if cemera is ready to take a picture
	 *
	 * \returns true if camera is ready to take a picture, false otherwise.
	 */
	bool isReady();

	/** \brief Check if cemera is busy taking a picture
	 *
	 * \returns true if camera is busy taking a picture, false otherwise.
	 */
	bool isBusy();

	/** \brief Trigger taking a picture asynchronously. */
	void triggerPhoto();

	/** \brief Convert MediaFileType to human readable string.
	 *
	 * \returns MediaFileType as human readbale string.
	 */
	static std::string fileTypeEnum2String(DJI::OSDK::MediaFileType type);

	/** \brief Callback function for setting camera mode.
	 *
	 * \param ret_code Function return code.
	 * \param user_data Pointer to AodCanera object
	 */
	static void setModeCallback(ErrorCode::ErrorCodeType ret_code,
								UserData user_data);

	/** \brief Callback function for getting camera mode.
	 *
	 * \param ret_code Function return code.
	 * \param mode Camera work mode as reported by camera/vehicle.
	 * \param user_data Pointer to AodCanera object.
	 */
	static void getModeCallback(ErrorCode::ErrorCodeType ret_code,
								DJI::OSDK::CameraModule::WorkMode mode,
								UserData user_data);

	/** \brief Callback function for camera trigger for taking a photo.
	 *
	 * \param ret_code Function return code.
	 * \param user_data Pointer to AodCanera object.
	 */
	static void shootPhotoCallback(DJI::OSDK::ErrorCode::ErrorCodeType ret_code,
								   DJI::OSDK::UserData user_data);

	/** \brief Callback function for file list request.
	 *
	 * \param ret_code Function process state.
	 * \param file_list List of file on the camera's SD-Card.
	 * \param user_data Pointer to AodCanera object.
	 */
	static void fileListCallback(E_OsdkStat ret_code,
								 const FilePackage file_list,
								 DJI::OSDK::UserData user_data);

	/** \brief Callback function for file data request.
	 *
	 * \param ret_code Function process state.
	 * \param user_data Pointer to AodCanera object.
	 */
	static void fileDataCallback(E_OsdkStat ret_code, void* user_data);

	static void videoStreamCallback(uint8_t* buf, int buf_len, void* user_data);

	/** \brief Get the camera name as assigned during initialization.
	 *
	 * \returns Name of the camera as assigned during initialization.
	 */
	std::string getName();

	/** \brief Get camera's busy state.
	 *
	 * During taking of a picture the camera enters a busy state. Once the
	 * camera has finished taking a picture and can is able to take a picture
	 * once again the state changes to ready.
	 *
	 * \returns CameraState::Busy while camera is busy taking a picture,
	 * 			CameraState::Ready otherwise.
	 */
	CameraState getState();

	/** \brief Get the last captured camera error.
	 *
	 * \returns the last captured camera error.
	 */
	ErrorCode::ErrorCodeType getLastError();

	/** \brief Request list of files from camera (non-blocking) */
	void requestFileList();

	/** \brief 	Check if request to retrieve list of files was successful and
	 * 			list of files has been updated.
	 *
	 * \returns true if update of file list was successful, false otherwise.
	 */
	bool isFileListUpdated();

	/** \brief 	Check if request to retrieve list of files is still pending.
	 *
	 * \returns true if update of file list is pending, false otherwise.
	 */
	bool isFileListUpdatePending();

	/** \brief	Check if list of files has changed (due to call to
	 * 			requestFileList).
	 *
	 * \returns true if list of files has changed, false otherwise,
	 */
	bool hasFileListChanged();

	/** \brief	Request transition to work mode: SHOOT_PHOTO. */
	void requestWorkModePhoto();

	/** \brief	Request transition to work mode: RECORD_VIDEO. */
	void requestWorkModeVideo();

	/** \brief	Request currently active work mode. */
	void requestCurrentWorkMode();

	/** \brief	Evaluates last_err and completes work mode transition. */
	void completeWorkModeTransition();

	/** \brief 	Evaluates last_err and updates the current work mode according
	 * to parameter in callback.
	 *
	 * \param mode	Camera work mode as received by getModeCallback.
	 */
	void updateWorkMode(DJI::OSDK::CameraModule::WorkMode mode);

	/** \brief Check if current work mode is set to SHOOT_PHOTO. */
	bool isCurrentWorkModePhoto();

	/** \brief Check if current work mode is set to RECORD_VIDEO. */
	bool isCurrentWorkModeVideo();

	/** \brief	Check if work mode transition to requested work mode is
	 * 			complete.
	 */
	bool isWorkModeTransitionComplete();

	/** \brief Try to obtain the right to download files.
	 *
	 * Tries to obtain the download right for two seconds.
	 *
	 * \returns true if right to download files could be obtained, false
	 *			otherwise.
	 */
	bool obtainDownloadRight();

	/** \brief Function to set class' last_err field.
	 *
	 * \param err ErrorCodeType to set last_err to.
	 */
	void setLastError(ErrorCode::ErrorCodeType err);

	//
	// File List Functions
	//

	/** \brief 	Get the number of files in the file list acquired via
	 * 			requestFileList.
	 */
	unsigned int getNumberOfFilesInFileList();

	/** \brief 	Get the name of the file with the given index in the file list.
	 *
	 * \returns the name of the file at the given index in the file list.
	 */
	std::string getNameOfFileInFileList(unsigned int file_list_index);

	/** \brief 	Get the file from the camera identified by the given index of
	 * 			the file list and save it under the specified file name.
	 *
	 * Evaluate last_err to check if download was initiated successfully.
	 *
	 * Asynchronous call. Once finished callback function is called and the
	 * corresponding flag is set.
	 */
	void getFileFromCamera(unsigned int file_list_index, std::string file_name);

	/** \brief 	Get the file from the camera identified by the given index of
	 * 			the file list.
	 *
	 * Evaluate last_err to check if download was initiated successfully.
	 *
	 * Asynchronous call. Once finished callback function is called and the
	 * corresponding flag is set.
	 */
	void getFileFromCamera(unsigned int file_list_index);

	/** \brief 	Get the last file from camera identified by the given index of
	 * 			the file list.
	 *
	 * Evaluate last_err to check if download was initiated successfully.
	 *
	 * Asynchronous call. Once finished callback function is called and the
	 * corresponding flag is set.*/
	void getLastFileFromCamera();

	/** \brief Check if file transfer from camera is complete.
	 *
	 * Evaluate last_err to check if transfer was successful.
	 */
	bool isFileTransferComplete();

	/** \brief Print file list to string.
	 *
	 * \returns a string containing a human readable list of files on the
	 * 			camera.
	 */
	std::string sprintFileList();

	//
	// Video Functions
	//

	/**
	 * @brief	Change source of h264 video stream.
	 *
	 * @param src Video stream source (wide, zoom, ir)
	 */
	void changeVideoSource(LiveView::LiveViewCameraSource src);

	/**
	 * @brief	Start h264 video stream.
	 */
	void startVideoStream();

	/**
	 * @brief	Stop h264 video stream.
	 */
	void stopVideoStream();

private:
	/** \brief 	Pointer to CameraManager as passed during instantiation of
	 * 			AodCamera-object.
	 */
	DJI::OSDK::CameraManager* camera_manager;

	DJI::OSDK::AdvancedSensing* advanced_sensing;

	/** \brief Flag indicating if camera was successfully initialized. */
	bool initialized;

	DJI::OSDK::CameraModule::WorkMode work_mode_requested;
	DJI::OSDK::CameraModule::WorkMode work_mode;

	/** \brief Last evaluated error code. */
	ErrorCode::ErrorCodeType last_err;
};

AodCamera::AodCamera(DJI::OSDK::CameraManager* cm,
					 DJI::OSDK::AdvancedSensing* as)
{
	this->camera_manager = cm;
	this->advanced_sensing = as;
	// camera busy is cleared once camera is initialized successfully.
	this->state = CameraState::CAMERA_BUSY;
	this->work_mode = DJI::OSDK::CameraModule::WorkMode::WORK_MODE_UNKNOWN;
	this->work_mode_requested =
		DJI::OSDK::CameraModule::WorkMode::WORK_MODE_UNKNOWN;
	this->file_list_updated = false;
	this->file_data_transfer_done = false;
	this->last_err = ErrorCode::SysCommonErr::Success;
	this->p_video_file = NULL;
	this->initialized = this->init();
}

bool AodCamera::init()
{
	this->last_err = this->camera_manager->initCameraModule(
		DJI::OSDK::PAYLOAD_INDEX_0, "H20T");

	if (this->last_err != ErrorCode::SysCommonErr::Success)
	{
		DERROR("Init Camera module H20T.C failed. Error code: 0x%1X",
			   this->last_err);
		ErrorCode::printErrorCodeMsg(this->last_err);
		return false;
	}

	// Set ready state
	this->state = CameraState::CAMERA_READY;

	return true;
}

bool AodCamera::isInitialized() { return this->initialized; }
bool AodCamera::isReady()
{
	return this->state == AodCamera::CameraState::CAMERA_READY;
}

ErrorCode::ErrorCodeType AodCamera::getLastError() { return this->last_err; }

void AodCamera::setLastError(ErrorCode::ErrorCodeType err)
{
	this->last_err = err;
}

void AodCamera::requestFileList()
{
	// starting file list request so clear updated flag
	this->file_list_updated = false;
	this->file_list_changed = false;

	if (this->obtainDownloadRight())
	{
		this->last_err = this->camera_manager->startReqFileList(
			DJI::OSDK::PAYLOAD_INDEX_0, fileListCallback, this);

		if (this->last_err != ErrorCode::SysCommonErr::Success)
		{
			DERROR("Error requesting file list! Error code: 0x%1X",
				   this->last_err);
			ErrorCode::printErrorCodeMsg(this->last_err);
		}
	}
}

bool AodCamera::isFileListUpdated() { return this->file_list_updated; }
bool AodCamera::isFileListUpdatePending() { return !this->file_list_updated; }

bool AodCamera::hasFileListChanged() { return this->file_list_changed; }

void AodCamera::requestWorkModePhoto()
{
	this->work_mode_requested = DJI::OSDK::CameraModule::WorkMode::SHOOT_PHOTO;

	this->camera_manager->setModeAsync(DJI::OSDK::PAYLOAD_INDEX_0,
									   this->work_mode_requested,
									   setModeCallback, this);
}

void AodCamera::requestWorkModeVideo()
{
	this->work_mode_requested = DJI::OSDK::CameraModule::WorkMode::RECORD_VIDEO;

	this->camera_manager->setModeAsync(DJI::OSDK::PAYLOAD_INDEX_0,
									   this->work_mode_requested,
									   setModeCallback, this);
}

void AodCamera::requestCurrentWorkMode()
{
	this->work_mode_requested =
		DJI::OSDK::CameraModule::WorkMode::WORK_MODE_UNKNOWN;

	this->camera_manager->getModeAsync(DJI::OSDK::PAYLOAD_INDEX_0,
									   getModeCallback, this);
}

void AodCamera::completeWorkModeTransition()
{
	if (this->last_err == ErrorCode::SysCommonErr::Success)
	{
		this->work_mode = this->work_mode_requested;
	}
	else
	{
		this->work_mode = DJI::OSDK::CameraModule::WorkMode::WORK_MODE_UNKNOWN;
	}
}

void AodCamera::updateWorkMode(DJI::OSDK::CameraModule::WorkMode mode)
{
	if (this->last_err == ErrorCode::SysCommonErr::Success)
	{
		this->work_mode = mode;
	}
	else
	{
		this->work_mode = DJI::OSDK::CameraModule::WorkMode::WORK_MODE_UNKNOWN;
	}
}

bool AodCamera::isCurrentWorkModePhoto()
{
	return this->work_mode == DJI::OSDK::CameraModule::WorkMode::SHOOT_PHOTO;
}

bool AodCamera::isCurrentWorkModeVideo()
{
	return this->work_mode == DJI::OSDK::CameraModule::WorkMode::RECORD_VIDEO;
}

bool AodCamera::isWorkModeTransitionComplete()
{
	return this->work_mode == this->work_mode_requested;
}

bool AodCamera::obtainDownloadRight()
{
	ErrorCode::ErrorCodeType ret;

	DSTATUS("Trying to obtain read permission to camera memory from camera.");

	ret = this->camera_manager->obtainDownloadRightSync(
		DJI::OSDK::PAYLOAD_INDEX_0, true /* obtain right to download */, 2);

	if (ret != ErrorCode::SysCommonErr::Success)
	{
		DERROR(
			"Could not obtain read permission for camera storage. "
			"Error code: 0x%1X",
			ret);
		ErrorCode::printErrorCodeMsg(ret);
		return false;
	}
	return true;
}

std::string AodCamera::getName()
{
	if (!this->initialized)
		return std::string();

	std::string cameraName;
	this->last_err = this->camera_manager->getCameraModuleName(
		DJI::OSDK::PAYLOAD_INDEX_0, cameraName);

	if (this->last_err != ErrorCode::SysCommonErr::Success)
	{
		DERROR("Could not get camera name! Error code: 0x%1X", this->last_err);
		ErrorCode::printErrorCodeMsg(this->last_err);
		return std::string();
	}

	return cameraName;
}

void AodCamera::triggerPhoto()
{
	if (this->isReady())
	{
		DSTATUS("Taking single photo with camera at payload index 0 (H20T)");

		this->state = AodCamera::CameraState::CAMERA_BUSY;

		this->camera_manager->startShootPhotoAsync(
			DJI::OSDK::PAYLOAD_INDEX_0,
			DJI::OSDK::CameraModule::ShootPhotoMode::SINGLE,
			AodCamera::shootPhotoCallback, (void*)this);
	}
	else
	{
		DERROR("Could not take photo. Camera is busy!");
	}
}

void AodCamera::setModeCallback(ErrorCode::ErrorCodeType ret_code,
								UserData user_data)
{
	AodCamera* ap = (AodCamera*)user_data;

	ap->last_err = ret_code;
	ap->completeWorkModeTransition();
}

void AodCamera::getModeCallback(ErrorCode::ErrorCodeType ret_code,
								DJI::OSDK::CameraModule::WorkMode mode,
								UserData user_data)
{
	AodCamera* ap = (AodCamera*)user_data;
	ap->last_err = ret_code;
	ap->updateWorkMode(mode);
}

void AodCamera::shootPhotoCallback(DJI::OSDK::ErrorCode::ErrorCodeType ret_code,
								   DJI::OSDK::UserData user_data)
{
	AodCamera* ap = (AodCamera*)user_data;
	ap->state = AodCamera::CameraState::CAMERA_READY;
	ap->setLastError(ret_code);

	DSTATUS("shootPhotoCallback return code: %lu", ret_code);

	if (ret_code != ErrorCode::SysCommonErr::Success)
	{
		DERROR(
			"Could not take a picture. Error code: "
			"0x%1X",
			ret_code);
		ErrorCode::printErrorCodeMsg(ret_code);
	}
	else
	{
		DSTATUS("shootPhotoCallback called");
	}
}

void AodCamera::fileListCallback(E_OsdkStat ret_code,
								 const FilePackage file_list,
								 DJI::OSDK::UserData user_data)
{
	AodCamera* ap = (AodCamera*)user_data;

	if (ret_code != E_OsdkStat::OSDK_STAT_OK)
	{
		DERROR(
			"Error receiving file list. Error code: "
			"0x%1X",
			ret_code);
	}
	else
	{
		// check if new file list is different from previous file list.
		if (file_list.type == ap->file_list.type
			|| file_list.media.size() == ap->file_list.media.size())
		{
			// set flag accordingly
			ap->file_list_changed = true;
		}

		ap->file_list = file_list;	// copy over new file list
		// set flag that file list has been updated
		ap->file_list_updated = true;
	}
}

void AodCamera::fileDataCallback(E_OsdkStat ret_code, void* user_data)
{
	AodCamera* ap = (AodCamera*)user_data;

	if (ret_code == OSDK_STAT_OK)
	{
		DSTATUS("Download file [%s] successfully.");
	}
	else
	{
		DERROR("Download file data failed.");
	}

	ap->setLastError(ret_code);
	ap->file_data_transfer_done = true;
}

void AodCamera::videoStreamCallback(uint8_t* buf, int buf_len, void* user_data)
{
	AodCamera* ap = (AodCamera*)user_data;

	fwrite(buf, 1, buf_len, ap->p_video_file);

	return;
}

unsigned int AodCamera::getNumberOfFilesInFileList()
{
	return this->file_list.media.size();
}

std::string AodCamera::getNameOfFileInFileList(unsigned int file_list_index)
{
	if (this->file_list.media.size() > file_list_index)
	{
		return this->file_list.media[file_list_index].fileName;
	}
	return std::string();
}

void AodCamera::getFileFromCamera(unsigned int file_list_index,
								  std::string file_name)
{
	DSTATUS("Download file @ index %d", file_list_index);

	DJI::OSDK::MediaFile file = this->file_list.media[file_list_index];

	this->file_data_transfer_done = false;

	if (this->obtainDownloadRight())
	{
		this->last_err = this->camera_manager->startReqFileData(
			DJI::OSDK::PAYLOAD_INDEX_0, file.fileIndex, file_name,
			fileDataCallback, this);

		if (this->last_err != ErrorCode::SysCommonErr::Success)
		{
			DERROR(
				"File Data Request failed using "
				"'startReqFileData'. Error code: 0x%1X",
				this->last_err);
			ErrorCode::printErrorCodeMsg(this->last_err);
		}
	}
}

void AodCamera::getFileFromCamera(unsigned int file_list_index)
{
	// create local file name
	std::stringstream s_local_filename;
	s_local_filename << "./" << this->file_list.media[file_list_index].fileName;

	this->getFileFromCamera(file_list_index, s_local_filename.str());
}

void AodCamera::getLastFileFromCamera()
{
	this->getFileFromCamera(this->getNumberOfFilesInFileList() - 1);
}

bool AodCamera::isFileTransferComplete()
{
	return this->file_data_transfer_done;
}

std::string AodCamera::sprintFileList()
{
	std::stringstream ss;
	for (std::size_t i = 0; i < this->getNumberOfFilesInFileList(); ++i)
	{
		ss << "File " << i << ": "
		   << (this->file_list.media[i].valid ? "Valid" : "Invalid")
		   << " File at index " << this->file_list.media[i].fileIndex
		   << " named " << this->file_list.media[i].fileName << "("
		   << this->fileTypeEnum2String(this->file_list.media[i].fileType)
		   << ")" << std::endl;
	}

	return ss.str();
}

void AodCamera::changeVideoSource(LiveView::LiveViewCameraSource src)
{
	if (this->advanced_sensing != NULL)
	{
		this->advanced_sensing->changeH264Source(
			LiveView::LiveViewCameraPosition::OSDK_CAMERA_POSITION_NO_1, src);
	}
	else
	{
		DERROR("Could not change video source due to null pointer!");
	}
}

void AodCamera::startVideoStream()
{
	if (this->advanced_sensing != NULL)
	{
		this->p_video_file = fopen("./h20t_video.h264", "w");
		if (this->p_video_file == NULL)
		{
			DERROR("Opening file for video recording failed!\n");
			return;
		}

		LiveView::LiveViewErrCode err;
		err = this->advanced_sensing->startH264Stream(
			LiveView::LiveViewCameraPosition::OSDK_CAMERA_POSITION_NO_1,
			AodCamera::videoStreamCallback, this);

		if (err != LiveView::LiveViewErrCode::OSDK_LIVEVIEW_PASS)
		{
			DERROR("Error starting H264 stream! Error code: 0x%1X", err);
			ErrorCode::printErrorCodeMsg(err);
		}
	}
	else
	{
		DERROR("Could not start video stream due to null pointer!");
	}
}

void AodCamera::stopVideoStream()
{
	if (this->advanced_sensing != NULL)
	{
		this->advanced_sensing->stopH264Stream(
			LiveView::LiveViewCameraPosition::OSDK_CAMERA_POSITION_NO_1);

		fflush(this->p_video_file);
		fclose(this->p_video_file);
	}
	else
	{
		DERROR("Could not stop video source due to null pointer!");
	}

	this->p_video_file = NULL;
}

void show_rgb_image_cb(CameraRGBImage img, void* p)
{
	std::string name = std::string(reinterpret_cast<char*>(p));
	cout << "#### Got image from:\t" << name << endl;
#ifdef OPEN_CV_INSTALLED
	cv::Mat mat(img.height, img.width, CV_8UC3, img.rawData.data(),
				img.width * 3);
	cv::cvtColor(mat, mat, cv::COLOR_RGB2BGR);
	cv::imshow(name, mat);
	cv::waitKey(1);
#endif
}

std::string AodCamera::fileTypeEnum2String(DJI::OSDK::MediaFileType type)
{
	switch (type)
	{
	case DJI::OSDK::MediaFileType::JPEG:
		return std::string("JPEG");
	case DJI::OSDK::MediaFileType::DNG:
		return std::string("DNG");
	case DJI::OSDK::MediaFileType::MOV:
		return std::string("MOV");
	case DJI::OSDK::MediaFileType::MP4:
		return std::string("MP4");
	case DJI::OSDK::MediaFileType::PANORAMA:
		return std::string("PANORAMA");
	case DJI::OSDK::MediaFileType::TIFF:
		return std::string("TIFF");
	case DJI::OSDK::MediaFileType::UL_CTRL_INFO:
		return std::string("UL_CTRL_INFO");
	case DJI::OSDK::MediaFileType::UL_CTRL_INFO_LZ4:
		return std::string("UL_CTRL_INFO_LZ4");
	case DJI::OSDK::MediaFileType::AUDIO:
		return std::string("AUDIO");
	default:
		return std::string("UNKNOWN");
	}
}

void opencvImgWaitkeyTask(bool& run)
{
	while (run)
	{
		cv::Mat img = cv::imread("./__temp.jpg", cv::IMREAD_COLOR);
		if (!img.empty())
		{
			// 4056 x 3040
			cv::resize(img, img, cv::Size(1014, 760));
			cv::imshow("Display Image", img);
		}
		cv::waitKey(100);  // using separate thread
	}
}

int main(int argc, char** argv)
{
	/*! Setup the OSDK: Read config file, create vehicle, activate. */
	LinuxSetup linuxEnvironment(argc, argv, true /*enable advanced sensing*/);
	Vehicle* vehicle = linuxEnvironment.getVehicle();

	if (vehicle == NULL)
	{
		DERROR("Vehicle not initialized, exiting.");
		return -1;
	}

	// configure advanced sensing
	const char* acm_dev =
		linuxEnvironment.getEnvironment()->getDeviceAcm().c_str();
	vehicle->advancedSensing->setAcmDevicePath(acm_dev);

	// Get CameraManager handle
	CameraManager* cm = vehicle->cameraManager;
	AodCamera aod_camera(vehicle->cameraManager, vehicle->advancedSensing);

	if (!aod_camera.isInitialized())
		return 1;

	DSTATUS("Camera module name: %s", aod_camera.getName());

	// start thread to call waitkey vor opencv image display
	bool run_thread = true;
	thread t1(opencvImgWaitkeyTask, std::ref(run_thread));
	std::string s;
	while (true)
	{
		std::cout << std::endl;
		std::cout << "Menu" << std::endl << "----" << std::endl;
		std::cout << "> [t] Take a picture" << std::endl
				  << "> [l] List files on camera" << std::endl
				  << "> [d] Download file from camera" << std::endl
				  << "> [i] Display image from camera" << std::endl
				  << "> [r] Record video from camera" << std::endl
				  << "> [v] Camera live view" << std::endl
				  << "> [q] Quit" << std::endl;
		char user_input = 0;
		// std::cin >> user_input;
		while (getline(cin, s) && !s.empty())
		{
			if (std::stringstream(s) >> user_input)
				break;
		}

		switch (user_input)
		{
		case 't':  // trigger taking a photo
			aod_camera.triggerPhoto();

			while (!aod_camera.isReady())
			{
				OsdkOsal_TaskSleepMs(250);
			}
			break;
		case 'l':  // request file list and display
			aod_camera.requestFileList();

			DSTATUS("Requesting file list ");
			while (aod_camera.isFileListUpdatePending())
			{
				std::cout << '.';
				OsdkOsal_TaskSleepMs(100);
			}
			std::cout << " DONE";

			if (aod_camera.getLastError() != ErrorCode::SysCommonErr::Success)
			{
				DERROR("Could not get file list. Error code: 0x%1X",
					   aod_camera.getLastError());
				ErrorCode::printErrorCodeMsg(aod_camera.getLastError());
			}

			std::cout << aod_camera.sprintFileList() << std::endl;

			break;
		case 'd':  // download file
		{
			unsigned int user_file_idx = 0;
			do
			{
				std::cout << "> Enter file index to download [0-"
						  << aod_camera.getNumberOfFilesInFileList() - 1
						  << "]: ";
				std::cin >> user_file_idx;
			} while (user_file_idx >= aod_camera.getNumberOfFilesInFileList());

			aod_camera.getFileFromCamera(user_file_idx);

			if (aod_camera.getLastError() == ErrorCode::SysCommonErr::Success)
			{
				int max_wait_count = 0;
				while (!aod_camera.isFileTransferComplete()
					   && max_wait_count < 10)
				{
					max_wait_count++;
					OsdkOsal_TaskSleepMs(500);
				}

				if (max_wait_count < 10)
					DSTATUS("File download complete.");
				else
					DSTATUS("File download timed out!");
			}
			break;
		}
		case 'i':  // display image
		{
			unsigned user_file_idx = 0;

			if (aod_camera.getNumberOfFilesInFileList() == 0)
			{
				DSTATUS("File list is empty.");
				break;
			}

			do
			{
				std::cout << "> Enter file index to display [0-"
						  << aod_camera.getNumberOfFilesInFileList() - 1
						  << "]: ";
				//
				std::cin.clear();
				std::fflush(stdin);
				getline(cin, s);
				if (!(std::stringstream(s) >> user_file_idx))
					user_file_idx = aod_camera.getNumberOfFilesInFileList() - 1;
			} while (user_file_idx >= aod_camera.getNumberOfFilesInFileList());

			aod_camera.getFileFromCamera(user_file_idx, "./__temp.jpg");

			if (aod_camera.getLastError() == ErrorCode::SysCommonErr::Success)
			{
				int max_wait_count = 0;
				while (!aod_camera.isFileTransferComplete()
					   && max_wait_count < 10)
				{
					max_wait_count++;
					OsdkOsal_TaskSleepMs(500);
				}

				if (max_wait_count < 10)
					DSTATUS("File download complete.");
				else
					DSTATUS("File download timed out!");
			}
			break;
		}
		case 'r':
		{
			char video_source;
			do
			{
				std::cout << "> Select video source [W ide, Z oom, T hermal]: ";
				std::cin.clear();
				std::fflush(stdin);
				getline(cin, s);
				if (!(std::stringstream(s) >> video_source))
					std::cout << "Invalid option!" << std::endl;
			} while (video_source != 'W' && video_source != 'w'
					 && video_source != 'Z' && video_source != 'z'
					 && video_source != 'T' && video_source != 't');

			LiveView::LiveViewCameraSource cam_src;

			switch (video_source)
			{
			case 'W':
			case 'w':
				cam_src = LiveView::LiveViewCameraSource::
					OSDK_CAMERA_SOURCE_H20T_WIDE;
				DSTATUS("Selected camera source: wide");
				break;
			case 'Z':
			case 'z':
				cam_src = LiveView::LiveViewCameraSource::
					OSDK_CAMERA_SOURCE_H20T_ZOOM;
				DSTATUS("Selected camera source: zoom");
				break;
			case 'T':
			case 't':
				cam_src =
					LiveView::LiveViewCameraSource::OSDK_CAMERA_SOURCE_H20T_IR;
				DSTATUS("Selected camera source: ir");
				break;
			}

			aod_camera.changeVideoSource(cam_src);

			aod_camera.startVideoStream();

			DSTATUS(
				"Sleeping 30 seconds while recording stream asynchronously.");

			unsigned int seconds_elapsed = 0;

			while (seconds_elapsed < 30)
			{
				sleep(1);
				seconds_elapsed++;
				std::cout << "." << std::flush;
			}
			std::cout << std::endl;

			aod_camera.stopVideoStream();
			break;
		}
		case 'q':
			run_thread = false;
			t1.join();
			DSTATUS("Good bye!");
			return 0;
		default:
			DSTATUS("Unnown or unimplemented command (%c)!", user_input);
			break;
		}

		if (false)	// camera control and photo
		{
			DJI::OSDK::CameraModule::WorkMode mode = printCameraWorkMode(cm);

			// Get camera work mode
			if (mode == DJI::OSDK::CameraModule::WorkMode::SHOOT_PHOTO)
			{
				aod_camera.triggerPhoto();

				while (!aod_camera.isReady())
				{
					OsdkOsal_TaskSleepMs(250);
				}

				sleep(5);

				// Request file list if shooting photo was successful
				if (aod_camera.getLastError()
					== ErrorCode::SysCommonErr::Success)
				{
					aod_camera.requestFileList();

					while (aod_camera.isFileListUpdatePending())
					{
						OsdkOsal_TaskSleepMs(250);
					}

					if (aod_camera.getLastError()
						!= ErrorCode::SysCommonErr::Success)
					{
						DERROR("Could not get file list. Error code: 0x%1X",
							   aod_camera.getLastError());
						ErrorCode::printErrorCodeMsg(aod_camera.getLastError());
					}
					else
					{
						aod_camera.getLastFileFromCamera();

						if (aod_camera.getLastError()
							== ErrorCode::SysCommonErr::Success)
						{
							int max_wait_count = 0;
							while (!aod_camera.isFileTransferComplete()
								   && max_wait_count < 10)
							{
								max_wait_count++;
								OsdkOsal_TaskSleepMs(500);
							}

							if (max_wait_count < 10)
								DSTATUS("File download complete.");
							else
								DSTATUS("File download timed out!");
						}
					}
				}
			}
		}
	}
	return 0;
}
