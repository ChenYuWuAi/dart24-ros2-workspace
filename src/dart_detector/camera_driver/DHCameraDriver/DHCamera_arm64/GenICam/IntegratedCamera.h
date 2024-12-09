#ifndef __GENICAM_CINTEGRATED_CAMERA_H__
#define __GENICAM_CINTEGRATED_CAMERA_H__

#include "GenICam/Defs.h"
#include "GenICam/Camera.h"
#include "GenICam/EventSubscribe.h"
#include "Memory/SharedPtr.h"

GENICAM_NAMESPACE_BEGIN

class CIntegratedCamera;
typedef Memory::TSharedPtr<CIntegratedCamera> CIntegratedCameraPtr;

/// \~chinese
/// \brief 集成的相机对象
/// \defgroup Camera 相机共用操作接口
/// \~english
/// \brief Integrated camera object
/// \defgroup Camera  Camera's Common operating interface

/// @{

/// \~chinese
/// \brief Class CIntegratedCamera 集成相机对象接口类，表示一台相机，所有协议类型都可共用的接口。
/// \~english
/// \brief Class CIntegratedCamera Integrated camera interface class, it means a camera has an interface which all protocol types are supported 
class GENICAM_API CIntegratedCamera
{
public:
	/// \~chinese
	/// \brief 集成的相机对象构造函数，同一个cameraPtr对应的是同一个集成的相机对象
	/// \param [in] cameraPtr 
	/// \return 集成的相机智能指针对象
	/// \~english
	/// \brief integrated camera object constructor function, same ICameraPtr is correspond to same camera object
	/// \param [in] cameraPtr 
	/// \return smart pointer to integrated camera object
	static CIntegratedCameraPtr getInstance(const ICameraPtr &cameraPtr);
	
	/// \~chinese
	/// \brief 获取相应序号的相机 
	/// \param [in] index 序号  设备发现时的序号从0开始
	/// \return 集成的相机智能指针对象
	/// \~english
	/// \brief get integrated camera which has corresponding index
	/// \param [in] index  index of camera(Start from 0)
	/// \return smart point of integrated camera object
	static CIntegratedCameraPtr getInstanceByIndex(int32_t index);
	
public:
	
	///枚举：物理接口类型扩展
	enum EInterfaceTypeEx
	{
		interfaceTypeGige = 0x00000001,	       ///< \~chinese 网卡接口类型  		///< \~english NIC type
		interfaceTypeUsb3 = 0x00000002,	       ///< \~chinese USB3.0接口类型		///< \~english USB3.0 interface type
		interfaceTypeCL   = 0x00000004, 	   ///< \~chinese CAMERALINK接口类型	///< \~english cameralink interface type
		interfaceTypePCIe = 0x00000008,        ///< \~chinese PCIe接口类型          ///< \~english PCIe interface type
		interfaceTypeAll  = 0x00000000,        ///< \~chinese 忽略接口类型			///< \~english all types interface type
		interfaceInvalidType = 0xFFFFFFFF      ///< \~chinese 无效接口类型			///< \~english invalid interface type
	};
	
	enum ECameraAccessPermission
	{
		accessPermissionOpen = 0,		///< \~chinese GigE相机没有被连接        ///< \~english The GigE vision device isn't connected to any application. 
		accessPermissionExclusive, 	///< \~chinese 独占访问权限        ///< \~english Exclusive Access Permission   
		accessPermissionControl, 	///< \~chinese 非独占可读访问权限  ///< \~english Non-Exclusive Readbale Access Permission  
		accessPermissionControlWithSwitchover,  	///< \~chinese Control access with switchover enabled.        ///< \~english Control access with switchover enabled.	
		accessPermissionUnknown = 254,  	///< \~chinese 无法确定        ///< \~english Value not known; indeterminate.   	
		accessPermissionUndefined     	///< \~chinese 未定义访问权限      ///< \~english Undefined Access Permission
	};
	
	enum ECameraType
	{
		typeGige = 0,       ///< \~chinese GIGE相机				///< \~english GigE camera
		typeU3v = 1,        ///< \~chinese USB3.0相机			///< \~english USB3.0 camera
		typeCL = 2,         ///< \~chinese CAMERALINK 相机		///< \~english Cameralink camera
        typePCIe = 3,       ///< \~chinese PCIe相机             ///< \~english PCIe Camera
		typeUndefined = 255 ///< \~chinese 未知类型				///< \~english Undefined camera
	};
	
	enum EGrabStrategy
	{
		grabStrartegySequential  = 0,	///< \~chinese 顺序取流 		///< \~english The images are processed in the order of their arrival
		grabStrartegyLatestImage = 1,   ///< \~chinese 获取最新的图片	///< \~english get latest image
		grabStrartegyUndefined   		///< \~chinese 未定义			///< \~english undefined
	};
	
	typedef struct pixelFormat_t
	{
		uint64_t		pixelFormat;
		Infra::CString	str;
	} pixelFormat_t;

	enum EBalanceRatioType
	{
		BalanceRatio_Red	= 0,		///< \~chinese 红色白平衡比率 	///< \~english Red BalanceRatio
		BalanceRatio_Green	= 1,		///< \~chinese 绿色白平衡比率 	///< \~english Green BalanceRatio
		BalanceRatio_Blue	= 2,		///< \~chinese 蓝色白平衡比率 	///< \~english Blue BalanceRatio
		BalanceRatioUndefined			///< \~chinese 未定义			///< \~english undefined
	};

	enum EGevGVSPExtendedIDMode
	{
		ExtendedIDMode_Off	= 0,		///< \~chinese 块数据关闭 		///< \~english ChunkData Off
		ExtendedIDMode_ON	= 1,		///< \~chinese 绿色白平衡比率 	///< \~english ChunkData On
		ExtendedIDModeUndefined			///< \~chinese 未定义			///< \~english undefined
	};

	typedef struct GevGVSPExtendedIDMode_t
	{
		EGevGVSPExtendedIDMode	extendedIDMode;
		Infra::CString			str;
	} GevGVSPExtendedIDMode_t;

	enum ETriggerSelector
	{
		TriggerSelector_FrameStart			= 0,		///< \~chinese 帧触发 	///< \~english TriggerSelector FrameStart
		TriggerSelector_AcquisitionStart	= 1,		///< \~chinese 采集触发 ///< \~english TriggerSelector AcquisitionStart
		TriggerSelector_LineStart			= 2,		///< \~chinese 线触发 	///< \~english TriggerSelector LineStart
		TriggerSelectorUndefined						///< \~chinese 未定义	///< \~english undefined
	};

	typedef struct TriggerSelector_t
	{
		ETriggerSelector	triggerSelector;
		Infra::CString		str;
	} TriggerSelector_t;

	enum ETriggerMode
	{
		TriggerMode_Off = 0,					///< \~chinese 触发模式关闭 	///< \~english TriggerMode Off
		TriggerMode_On	= 1,					///< \~chinese 触发模式打开		///< \~english TriggerMode On
		TriggerModeUndefined					///< \~chinese 未定义	///< \~english undefined
	};

	typedef struct TriggerMode_t
	{
		ETriggerMode		triggerMode;
		Infra::CString		str;
	} TriggerMode_t;

	enum ETriggerSource
	{
		TriggerSource_Software = 0,		///< \~chinese 软触发源		///< \~english TriggerSource Software
		TriggerSource_Line1 = 1,		///< \~chinese Line1触发源 	///< \~english TriggerSource Line1
		TriggerSource_Line2 = 2,		///< \~chinese Line2触发源 	///< \~english TriggerSource Line2
		TriggerSource_Line3 = 3,		///< \~chinese Line3触发源 	///< \~english TriggerSource Line3
		TriggerSource_Line4 = 4,		///< \~chinese Line4触发源 	///< \~english TriggerSource Line4
		TriggerSource_Line5 = 5,		///< \~chinese Line5触发源 	///< \~english TriggerSource Line5
		TriggerSource_Line6 = 6,		///< \~chinese Line6触发源 	///< \~english TriggerSource Line6
		TriggerSourceUndefined			///< \~chinese 未定义		///< \~english undefined
	};

	typedef struct TriggerSource_t
	{
		ETriggerSource	triggerSource;
		Infra::CString	str;
	} TriggerSource_t;

	enum ETriggerActivation
	{
		TriggerActivation_RisingEdge	= 0,		///< \~chinese 硬触发有效为上升沿 	///< \~english TriggerActivation RisingEdge
		TriggerActivation_FallingEdge	= 1,		///< \~chinese 硬触发有效为下降沿	///< \~english TriggerActivation FallingEdge
		TriggerActivationUndefined					///< \~chinese 未定义	///< \~english undefined
	};

	typedef struct TriggerActivation_t
	{
		ETriggerActivation	TriggerActivation;
		Infra::CString	str;
	} TriggerActivation_t;

	enum ELineSelector
	{
		LineSelector_Line0 = 0,		///< \~chinese Line0触发线 	///< \~english LineSelector Line0
		LineSelector_Line1 = 1,		///< \~chinese Line1触发线 	///< \~english LineSelector Line1
		LineSelector_Line2 = 2,		///< \~chinese Line2触发线 	///< \~english LineSelector Line2
		LineSelector_Line3 = 3,		///< \~chinese Line3触发线 	///< \~english LineSelector Line3
		LineSelector_Line4 = 4,		///< \~chinese Line4触发线 	///< \~english LineSelector Line4
		LineSelector_Line5 = 5,		///< \~chinese Line5触发线 	///< \~english LineSelector Line5
		LineSelector_Line6 = 6,		///< \~chinese Line6触发线 	///< \~english LineSelector Line6
		LineSelectorUndefined		///< \~chinese 未定义	///< \~english undefined
	};

	typedef struct LineSelector_t
	{
		ELineSelector	lineSelector;
		Infra::CString	str;
	} LineSelector_t;

	enum ELineSource
	{
		LineSource_ExposureActive		= 0,			///< \~chinese 曝光有效		///< \~english ExposureActive
		LineSource_FrameTriggerWait		= 1,			///< \~chinese 帧触发等待 	///< \~english FrameTriggerWait
		LineSource_Timer0Active			= 2,			///< \~chinese Timer有效 	///< \~english Timer0Active
		LineSource_UserOutput0			= 3,			///< \~chinese 用户输出 	///< \~english UserOutput0
		LineSource_AcquisitionTriggerWait = 4,			///< \~chinese 采集触发等待 ///< \~english AcquisitionTriggerWait
		LineSource_LightTrigger			= 5,			///< \~chinese 硬触发 		///< \~english LightTrigger
		LineSource_LineSelectorUndefined				///< \~chinese 未定义		///< \~english undefined
	};

	typedef struct LineSource_t
	{
		ELineSource	lineSource;
		Infra::CString	str;
	} LineSource_t;

	enum EUserOutputSelector
	{
		UserOutput0 = 0,			///< \~chinese 用户输出User0	///< \~english LineSelector Line1
		UserOutput1 = 1,			///< \~chinese 用户输出User1 	///< \~english LineSelector Line1
		UserOutput2 = 2,			///< \~chinese 用户输出User2 	///< \~english LineSelector Line2
		UserOutput3 = 3,			///< \~chinese 用户输出User3 	///< \~english LineSelector Line3
		UserOutput4 = 4,			///< \~chinese 用户输出User4 	///< \~english LineSelector Line4
		UserOutputSelectorUndefined	///< \~chinese 未定义			///< \~english undefined
	};

	typedef struct UserOutputSelector_t
	{
		EUserOutputSelector	userOutputSelector;
		Infra::CString	str;
	} UserOutputSelector_t;

	/// \~chinese
	/// \brief 设备流回调函数声明:每次回调送出一帧数据
	/// CFrame 回调时主动推送的帧对象
	/// \~english
	/// \brief Callback function declaration of stream: Send out one frame at each time use callback function
	/// CFrame The frame which will be active pushed out during the callback	
	typedef Infra::TSignal1<const CFrame&> Signal;	
	typedef Signal::Proc Proc;
	
public:
	
	/// \~chinese
	/// \brief  得到相机类型
	/// \return 返回相机类型
	/// \~english
	/// \brief  get Camera Type
	/// \return the Camera Type
	ECameraType getType() const;
	
	/// \~chinese
	/// \brief  获取相机名称
	/// \return 返回相机名称
	/// \~english
	/// \brief  get Camera Name
	/// \return camera's name
	const char * getName() const;
	
	/// \~chinese
	/// \brief  获取相机的唯一标识.
	/// \return 唯一标识相机的字符串.
	/// \~english
	/// \brief  get Camera Unique key
	/// \return string of this camera
	const char * getKey() const;
	
	/// \~chinese
	/// \brief  获取相机的自定义标识 DeviceUserID.
	/// \return 用户自定义的唯一标识相机的字符串.
	/// \~english
	/// \brief  get user defined ID of camera
	/// \return string of this camera	
	const char * getDeviceUserID() const;
	
	/// \~chinese
	/// \brief  设置相机的自定义标识 DeviceUserID.
	/// \param [in] pDeviceUserID 相机自定义标识
	/// \return 设置成功返回true,否则返回false
	/// \~english
	/// \brief  set user defined ID for camera
	/// \param [in] pDeviceUserID user defined ID
	/// \return success or failure		
	bool setDeviceUserID(const char * pDeviceUserID);

	/// \~chinese
	/// \brief 获取相机制造厂商名称
	/// \return 返回相机制造厂商名称
	/// \~english
	/// \brief get camera's vendor
	/// \return camera's vendor
	const char * getVendorName() const;
	
	/// \~chinese
	/// \brief 获取相机型号
	/// \return 返回相机型号
	/// \~english
	/// \brief get camera's model name
	/// \return camera's model name
	const char * getModelName() const;

	/// \~chinese
	/// \brief 获取相机序列号
	/// \return 返回相机序列号
	/// \~english
	/// \brief get camera's serial number
	/// \return camera's serial number
	const char * getSerialNumber() const;		
	
	/// \~chinese
	/// \brief 获取相机固件版本
	/// \return 返回相机固件版本
	/// \~english
	/// \brief get camera's device version
	/// \return camera's device version
	const char * getDeviceVersion() const;

	/// \~chinese
	/// \brief 获取相机的ipAddress
	/// \return 返回ipAddress 相机的IP地址
	/// \~english
	/// \brief	get camera's ipAddress
	/// \return camera's ipAddress
	const char * getIpAddress() const;

	/// \~chinese
	/// \brief 返回相机的子网掩码
	/// \return 返回subnetMask 相机的子网掩码
	/// \~english
	/// \brief return camera's subnetMask
	/// \return camera's subnetMask
	const char * getSubnetMask() const;

	/// \~chinese
	/// \brief 返回相机的网关
	/// \return 返回 subsetMask 相机的网关
	/// \~english
	/// \brief return camera's gateway
	/// \return camera's gateway	
	const char * getGateway() const;

	/// \~chinese
	/// \brief 获取相机的 macAddress
	/// \return 返回macAddress 相机的Mac地址
	/// \~english
	/// \brief return camera's MacAddress
	/// \return camera's MacAddress		
	const char * getMacAddress() const;
	
    /// \~chinese
	/// \brief  修改设备IP,非GigE相机调用该接口会返回false
	///1、调用该函数时如果newSubnetMask和newGateway都设置了有效值，则以此有效值为准;
	///2、调用该函数时如果newSubnetMask和newGateway都设置了NULL，则内部实现时用它所连接网卡的子网掩码和网关代替
	///3、调用该函数时如果newSubnetMask和newGateway两者中其中一个为NULL，另一个非NULL，则返回失败
	/// 该接口只针对通过网卡接入的设备，对于其余设备，调用此接口无意义
	/// \param [in] newIpAddress 新的IP地址信息
	/// \param [in] newSubnetMask 新的掩码信息
	/// \param [in] newGateway 新的网关信息
	/// \return 是否成功
	/// \~english
	/// \brief  modify device IP; If it is not GigE camera and still call this function then it will return false	
	/// \brief 1、When callback this function, if the values of newSubnetMask and newGateway are both valid then we consider the value is correct
	/// \brief 2、When callback this function, if the values of newSubnetMask and newGateway are both NULL, then these values will be replaced by the subnetmask and gateway of NIC which this camera connect to
	/// \brief 3、When callback this function, if there is one value of newSubnetMask or newGateway is NULL and the other one is not NULL, then return failed
	/// \brief this interface only serve the devices wchich is connected to NIC. As for other devices, callback this function won't work and return failed
	/// \param [in] newIpAddress new ip address 
	/// \param [in] newSubnetMask new subnetMask
	/// \param [in] newGateway new Gateway
	/// \retval < 0 modify ip failed
	/// \retval 0   modify ip success	
	bool forceIpAddress (const char* newIpAddress, const char* newSubnetMask = NULL, const char* newGateway = NULL);
	
	/// \~chinese
	/// \brief 获取相机的当前访问权限
	/// \return 如果获取成功，返回的枚举值代表相机的当前访问权限，如果失败，返回的枚举值为accessPermissionUnknown
	/// \~english
	/// \brief	get current access permission of camera
	/// \return success:return current access permission of camera, fail:return accessPermissionUnknown 
	ECameraAccessPermission getAccessPermission() const;

	/// \~chinese
	/// \brief  获取相机所属的接口名
	/// \return 相机所属的接口名
	/// \~english
	/// \brief  get the name of interface whice is connect to camera
	/// \return Interface Name
	const char * getInterfaceName() const;

	/// \~chinese
	/// \brief  获取相机所连接的接口类型
	/// \return 相机所连接的接口类型 
	/// \~english
	/// \brief  get the type of interface which is connect to camera
	/// \return the type of interface	
	EInterfaceTypeEx getInterfaceType() const;
	
    /// \~chinese
	/// \brief  连接设备
	/// 与相机建立连接关系，如Gige Vision协议的相机即是获得控制权限、创建控制通道
	/// \param [in] accessPermission 打开相机控制通道时默认为control access 权限
	/// \return 是否连接成功
	/// \~english
	/// \brief  connect device
	/// \param [in] accessPermission only accessPermissionControl supported currently
	/// \retval < 0 connection failed
	/// \retval 0   connection successful
	bool open(ECameraAccessPermission accessPermission = accessPermissionControl);
		
	/// \~chinese
	/// \brief 断开连接
	/// 断开与相机的连接关系，如Gige Vision协议的相机即是释放控制权限、销毁控制通道
	/// \return 是否断开成功
	/// \~english
	/// \brief disconnect camera, for example, for GigE camera which means release the control permission of camera, and distroy the control channel
	/// \retval < 0 disconnection failed
	/// \retval 0   disconnection successful	
	bool close();
		
	/// \~chinese
	/// \brief 判断相机当前是否处于打开状态
	/// \return 相机处于打开状态，返回true；否则返回false
	/// \~english
	/// \brief check the camera is opened or not
	/// \retval < 0 camera is not open
	/// \retval 0   camera is open	
	bool isOpen();
	
	/// \~chinese
	/// \brief 设置相机为自由拉流模式
	/// \return 设置成功返回true,否则返回false
	/// \~english
	/// \brief set camera's operating in Continuous mode
	/// \return success or failure
	bool setContinuousModeOn();
	
	/// \~chinese
	/// \brief 设置触发方式
	/// \param [in] triggerSource 触发方式
	/// \return 执行成功返回true,否则返回false	
	/// \~english
	/// \brief set Selects the type of trigger to configure
	/// \param [in] triggerSource
	/// \return success or failure	
	bool setTriggerSelector(const ETriggerSelector triggerSelector);

	/// \~chinese
	/// \brief 设置相机触发模式
	/// \param [in] triggerMode 触发模式On/Off
	/// \return 设置成功返回true,否则返回false
	/// \~english
	/// \brief set Controls if the selected trigger is active
	/// \param [in] triggerMode On/Off
	/// \return success or failure	
	bool setTriggerMode(const ETriggerMode triggerMode);

	/// \~chinese
	/// \brief 给相机下发一次软触发命令
	/// \return 执行成功返回true,否则返回false
	/// \~english
	/// \brief send a software trigger command to camera
	/// \return success or failure		
	bool executeSoftTriggerCmd();

	/// \~chinese
	/// \brief 设置相机触发源
	/// \param [in] triggerSource 软触发/硬触发1/硬触发2/硬触发3/硬触发4/硬触发5/硬触发6
	/// \return 设置成功返回true,否则返回false
	/// \~english
	/// \brief set Specifies the internal signal or physical input Line to use as the trigger source
	/// \param [in] triggerSource  softWare/Line1/Line2/Line3/Line4/Line5/Line6
	/// \return success or failure	
	bool setTriggerSource(const ETriggerSource triggerSource);

	/// \~chinese
	/// \brief 设置相机硬触发有效范围
	/// \param [in] triggerActivation 硬触发有效（上升沿）/硬触发有效（下降沿）
	/// \return 设置成功返回true,否则返回false
	/// \~english
	/// \brief set Specifies the activation mode of the trigger
	/// \param [in] triggerActivation  RisingEdge/FallingEdge
	/// \return success or failure	
	bool setTriggerActivation(const ETriggerActivation triggerActivation);

	/// \~chinese
	/// \brief 设置相机触发延迟时间
	/// \param [in] triggerDelay 触发延迟时间
	/// \return 设置成功返回true,否则返回false
	/// \~english
	/// \brief set Specifies the delay in microseconds (us) to apply after the trigger reception before activating it
	/// \param [in] triggerDelay
	/// \return success or failure	
	bool setTriggerDelay(const double triggerDelay);

	/// \~chinese
	/// \brief 开始抓图
	/// \param [in] maxImagesGrabbed 允许最多的抓图数，达到指定抓图数后停止抓图，如果为0，表示忽略此参数连续抓图
	/// \param [in] strategy 抓图策略,默认按缓存队列中的顺序抓图
	/// \return 返回抓图是否成功
	/// \~english
	/// \brief Start grabbing
	/// \param [in] maxImagesGrabbed Maximum images allowed to grab, once it reaches the limit then stop grabbing; If it is 0, then ignore this parameter and start grabbing continuously
	/// \param [in] strategy Image grabbing strategy; Image grabbing according to the order in buffer queue is default
	/// \return success or fail
	bool startGrabbing(uint64_t maxImagesGrabbed = 0,EGrabStrategy strategy = grabStrartegySequential);
					
	/// \~chinese
	/// \brief 停止抓图
	/// \return 返回停止抓图是否成功	
	/// \~english
	/// \brief Stop grabbing
	/// \return success or fail
	bool stopGrabbing();
							
	/// \~chinese
	/// \brief 是否正在抓图
	/// \return 返回是否成功
	/// \~english 
	/// \brief Check whether it is grabbing or not
	/// \return true means is grabbing, false means is not grabbing	
	bool isGrabbing();
	
	/// \~chinese
	/// \brief 获取一帧图像，该接口不支持多线程调用
	/// \param [out] frame 一帧图像
	/// \param [in]  timeoutMS 获取一帧图像的超时时长,单位MS,当值为INFINITE时表示无限等待
	/// \return 返回是否成功
	/// \~english
	/// \brief get a frame image, and this interface does not support multi-threading
	/// \param [out] ppFrame one frame; memory is allocated by function, call GENICAM_Frame to release memory when use up
	/// \param [in]  timeoutMS The timeout value of getting one image, unit is MS; When the value is INFINITE which means infinite wait
	/// \return success or failure
	bool getFrame(CFrame &frame,uint32_t timeoutMS = INFINITE);

	/// \~chinese
	/// \brief 注册数据帧回调函数。该异步获取帧机制和同步获取帧机制互斥，系统中两者只能选其一。
	/// \param [in] proc 数据帧回调函数，建议不要在该函数中处理耗时的操作，否则会阻塞后续数据帧的实时性
	/// \return 返回注册是否成功
	/// \~english
	/// \brief Register data frame callback function; This asynchronous frame acquisition mechanism and synchronous frame acquisition mechanism are mutually exclusive, only one method can be choosed between these two in system
	/// \brief only callback one function 
	/// \param [in] proc Data frame callback function; It is advised to not put time-cosuming operation in this function, otherwise it will block follow-up data frames and affect real time performance
	/// \return success or failure
	bool attachGrabbing(Proc proc);

	/// \~chinese
	/// \brief 去注册数据帧回调函数
	/// \param [in] proc 去注册数据帧回调函数
	/// \return 返回去注册是否成功
	/// \~english
	/// \brief	Unregister data frame callback function
	/// \param [in] proc Unregister data frame callback function
	/// \return success or failure
	bool detachGrabbing(Proc proc);
	
	/// \~chinese
    /// \brief 设置缓存个数,不能在拉流过程中设置
    /// \param [in]  nSize 缓存数量
    /// \return 返回是否成功
	/// \~chinese
    /// \brief Set buffer count, this cannot be set during frame grabbing
    /// \param [in]  nSize buffer count
    /// \return success or failure
    bool setBufferCount(uint32_t nSize);
	
	/// \~chinese
	/// \brief 获取图像宽度
	/// \return 返回图像的宽度，获取失败时返回 -1
	/// \~english
	/// \brief get the width of image
	/// \return the width of image, return -1 when failed
	int64_t getWidth();	

	/// \~chinese
	/// \brief 设置图像宽度
	/// \param [in] value 待设置的图像宽度
	/// \return 设置成功返回true，否则返回false
	/// \~english
	/// \brief set the width of image
	/// \param [in] value width of image
	/// \return success or failure
	bool setWidth(int64_t value);
	
	/// \~chinese
	/// \brief 获取图像高度
	/// \return 返回图像的高度，获取失败时返回 -1
	/// \~english 
	/// \brief get the height of image
	/// \return the height of image, return -1 when failed	
	int64_t getHeight();

	/// \~chinese
	/// \brief 设置图像高度
	/// \param [in] value 待设置的图像高度
	/// \return 设置成功返回true，否则返回false	
	/// \~english
	/// \brief set the height of image
	/// \param [in] value height of image
	/// \return success or failure	
	bool  setHeight(int64_t value);

	/// \~chinese
	/// \brief 获取X方向偏移
	/// \param [out] value 获取到的X方向偏移值
	/// \return 获取成功返回true，否则返回 false
	/// \~english
	/// \brief get offsetX value
	/// \param [out] offsetX value
	/// \return success or failure
	bool getOffsetX(int64_t &value);

	/// \~chinese
	/// \brief 设置X方向偏移
	/// \param [in] value 待设置的X方向偏移值
	/// \return 设置成功返回true，否则返回false	
	/// \~english
	/// \brief set offsetX value
	/// \param [in] offsetX value
	/// \return success or failure	
	bool setOffsetX(int64_t value);

	/// \~chinese
	/// \brief 获取Y方向偏移
	/// \param [out] value 获取到的Y方向偏移值
	/// \return 获取成功返回true，否则返回 false
	/// \~english
	/// \brief get offsetY value
	/// \param [out] offsetY value
	/// \return success or failure
	bool getOffsetY(int64_t &value);

	/// \~chinese
	/// \brief 设置Y方向偏移
	/// \param [in] value 待设置的Y方向偏移值
	/// \return 设置成功返回true，否则返回false	
	/// \~english
	/// \brief set offsetY value
	/// \param [in] offsetY value
	/// \return success or failure	
	bool setOffsetY(int64_t value);
	
	/// \~chinese
	/// \brief 设置ROI区域
	/// \param [in] offsetX 待设置的X方向偏移值
	/// \param [in] offsetY 待设置的Y方向偏移值
	/// \param [in] width 待设置的图像宽度
	/// \param [in] width 待设置的图像高度
	/// \return 设置成功返回true，否则返回false	
	/// \~english
	/// \brief set offsetY value
	/// \param [in] offsetX value
	/// \param [in] offsetY value
	/// \param [in] value width of image
	/// \param [in] value height of image
	/// \return success or failure	
	bool setROI(int64_t offsetX, int64_t offsetY, int64_t width, int64_t height);

	/// \~chinese
	/// \brief 获取增益
	/// \param [out] value 获取到的增益值
	/// \return 获取成功返回true，否则返回 false
	/// \~english
	/// \brief get gain value
	/// \param [out] value gain value
	/// \return success or failure
	bool getGain(double &value);
	
	/// \~chinese
	/// \brief 设置增益
	/// \param [in] value 待设置的增益值
	/// \return 设置成功返回true，否则返回false	
	/// \~english
	/// \brief set gain value
	/// \param [out] value gain value
	/// \return success or failure	
	bool setGain(double value);
	
	/// \~chinese
	/// \brief 获取亮度
	/// \return 返回亮度值，获取失败时返回 -1
	/// \~english
	/// \brief get brightness of image
	/// \return the brightness of image, return -1 when failed	
	int64_t getBrightness();

	/// \~chinese
	/// \brief 设置亮度
	/// \param [in] value 待设置的亮度值
	/// \return 设置成功返回true，否则返回false		
	/// \~english
	/// \brief set brightness of image
	/// \param [in] value brightness value
	/// \return success or failure	
	bool setBrightness(int64_t value);
	
	/// \~chinese
	/// \brief 获取曝光时间
	/// \param [out] value 获取到的曝光时间
	/// \return 获取成功返回true，否则返回false
	/// \~english
	/// \brief get exposure time
	/// \param [out] value exposure time
	/// \return success or failure
	bool getExposureTime(double &value);	
	
	/// \~chinese
	/// \brief 设置曝光时间
	/// \param [in] value 待设置的曝光时间
	/// \return 设置成功返回true，否则返回false		
	/// \~english
	/// \brief set the exposure time 
	/// \param [in] value exposure time
	/// \return success or failure		
	bool setExposureTime(double value);						
	
	/// \~chinese
	/// \brief 获取对比度
	/// \return 返回对比度，获取失败时返回 -1
	/// \~english
	/// \brief get contrast of image
	/// \return the contrast of image, return -1 when failed		
	int64_t getContrast();

	/// \~chinese
	/// \brief 设置对比度
	/// \param [in] value 待设置的对比度
	/// \return 设置成功返回true，否则返回false	
	/// \~english
	/// \brief set contrast of image
	/// \param [in] value contrast time
	/// \return success or failure		
	bool setContrast(int64_t value);
	
	/// \~chinese
	/// \brief 获取X方向是否翻转
	/// \return 返回X方向是否翻转，x方向反转返回true，否则返回false
	/// \~english
	/// \brief check whether reverse X or not
	/// \return true means reverse, false means do not reverse	
	bool getReverseX();									
	
	/// \~chinese
	/// \brief 设置X方向是否翻转
	/// \param [in] isReverse X方向是否翻转，默认设置为反转
	/// \return 设置成功返回true，否则返回false	
	/// \~english
	/// \brief Set whether reverse X or not
	/// \param [in] isReverse Reverse is default
    /// \return success or failure	
	bool setReverseX(bool isReverse = true);			
	
	/// \~chinese
	/// \brief 获取y方向是否翻转
	/// \return 返回y方向是否翻转，y方向反转返回true，否则返回false
	/// \~english
	/// \brief check whether reverse Y or not
	/// \return true means reverse, false means do not reverse		
	bool getReverseY();	

	/// \~chinese
	/// \brief 设置Y方向是否翻转
	/// \param [in] isReverse Y方向是否翻转，默认设置为反转
	/// \return 设置成功返回true，否则返回false	
	/// \~english
	/// \brief Set whether reverse Y or not
	/// \param [in] isReverse true means reverse, false means do not reverse	
	/// \return success or failure		
	bool setReverseY(bool isReverse = true);	

	/// \~chinese
	/// \brief 获取图像的像素格式
	/// \param [out] pixelFormat获取到的图像的像素格式
	/// \return 获取成功返回true，否则返回 false
	/// \~english
	/// \brief get pixelFormat value
	/// \param [out] pixelFormat value
	/// \return success or failure
	bool getPixelFormat(uint64_t& pixelFormat);

	/// \~chinese
	/// \brief 设置图像的像素格式
	/// \param [in] pixelFormat待设置图像的像素格式
	/// \return 设置成功返回true，否则返回false	
	/// \~english
	/// \brief set pixelFormat value
	/// \param [in] pixelFormat value
	/// \return success or failure	
	bool setPixelFormat(const uint64_t pixelFormat);

	/// \~chinese
	/// \brief 获取图像的像素大小
	/// \param [out] pixelSize获取到的图像的像素大小
	/// \return 获取成功返回true，否则返回 false
	/// \~english
	/// \brief get pixelSize value
	/// \param [out] pixelSize value
	/// \return success or failure
	bool getPixelSize(Infra::CString& pixelSize);

	/// \~chinese
	/// \brief 获取红色/绿色/蓝色白平衡比率
	/// \param [in] balanceRatioType白平衡红色/绿色/蓝色
	/// \param [out] balanceRatioValue获取到的红色/绿色/蓝色白平衡比率
	/// \return 获取成功返回true，否则返回 false
	/// \~english
	/// \brief get Red/Green/Blue balanceRatio value
	/// \param [in] balanceRatioType Red/Green/Blue
	/// \param [out] balanceRatioValue value
	/// \return success or failure
	bool getBalanceRatioSelector(EBalanceRatioType balanceRatioType, double& balanceRatioValue);

	/// \~chinese
	/// \brief 设置红色/绿色/蓝色白平衡比率
	/// \param [in] balanceRatioType白平衡红色/绿色/蓝色
	/// \param [in] balanceRatioValue待设置白平衡比率
	/// \return 设置成功返回true，否则返回false	
	/// \~english
	/// \brief set Red/Green/Blue balanceRatio value
	/// \param [in] balanceRatioType Red/Green/Blue
	/// \param [in] balanceRatioValue value
	/// \return success or failure	
	bool setBalanceRatioSelector(EBalanceRatioType balanceRatioType, const double balanceRatioValue);

	/// \~chinese
	/// \brief 获取gamma值
	/// \param [out] value 获取到的曝光时间
	/// \return 获取成功返回true，否则返回false
	/// \~english
	/// \brief get gamma value
	/// \param [out] value value of gamma
	/// \return success or failure		
	bool getGamma(double &value);
	
	/// \~chinese
	/// \brief 设置gamma值
	/// \return 设置gamma值，设置成功返回true，否则返回false
	/// \~english
	/// \brief get gamma value
	/// \param [out] value value of gamma
	/// \return success or failure		
	bool setGamma(double value);

	/// \~chinese
	/// \brief 获取帧率
	/// \param [out] fps 获取到帧率值
	/// \return 获取成功返回true，否则返回false
	/// \~english
	/// \brief get framerate
	/// \param [out]fps value of framerate
	/// \return success or failure		
	bool getAcquisitionFrameRate(double &fps);
	
	/// \~chinese
	/// \brief  设置帧率
	/// \return 设置帧率值，设置成功返回true，否则返回false
	/// \~english
	/// \brief set framerate
	/// \param [in]fps value of framerate
	/// \return success or failure		
	bool setAcquisitionFrameRate(double fps);

	/// \~chinese
	/// \brief 设置帧率有效
	/// \return 设置成功返回true，否则返回false
	/// \~english
	/// \brief set framerate valid
	/// \return success or failure		
	bool setAcquisitionFrameRateEnable(const bool value);

	/// \~chinese
	/// \brief 获取帧率(线阵)
	/// \param [out] fps 获取到帧率值
	/// \return 获取成功返回true，否则返回false
	/// \~english
	/// \brief get framerate
	/// \param [out]fps value of framerate
	/// \return success or failure		
	bool getAcquisitionLineRate(double &fps);

	/// \~chinese
	/// \brief  设置帧率(线阵)
	/// \return 设置帧率值，设置成功返回true，否则返回false
	/// \~english
	/// \brief set framerate
	/// \param [in]fps value of framerate
	/// \return success or failure		
	bool setAcquisitionLineRate(double fps);

	/// \~chinese
	/// \brief 设置帧率有效(线阵)
	/// \return 设置成功返回true，否则返回false
	/// \~english
	/// \brief set framerate valid
	/// \return success or failure		
	bool setAcquisitionLineRateEnable(const bool value);

	/// \~chinese
	/// \brief 获取外部连接相机接线
	/// \param [out] lineSelector 外部连接相机接线
	/// \return 获取成功返回true，否则返回 false
	/// \~english
	/// \brief get the physical line (or pin) of the external device connector to configure.
	/// \param [out] lineSelector physical line
	/// \return success or failure
	bool getLineSelector(ELineSelector& lineSelector);

	/// \~chinese
	/// \brief 设置外部连接相机接线
	/// \param [in] lineSelector 待设置外部连接相机接线
	/// \return 设置成功返回true，否则返回false	
	/// \~english
	/// \brief set the physical line (or pin) of the external device connector to configure.
	/// \param [in] lineSelector physical line
	/// \return success or failure	
	bool setLineSelector(const ELineSelector lineSelector);

	/// \~chinese
	/// \brief 获取空闲控制时间
	/// \param [out] timeAbs 获取到的空闲控制时间
	/// \return 获取成功返回true，否则返回 false
	/// \~english
	/// \brief get the absolute value of the selected line debouncer time in microseconds
	/// \param [out] timeAbs value
	/// \return success or failure
	bool getLineDebouncerTimeAbs(double& timeAbs);

	/// \~chinese
	/// \brief 设置空闲控制时间
	/// \param [in] timeAbs 待设置空闲控制时间
	/// \return 设置成功返回true，否则返回false	
	/// \~english
	/// \brief Sets the absolute value of the selected line debouncer time in microseconds
	/// \param [in] timeAbs value
	/// \return success or failure	
	bool setLineDebouncerTimeAbs(const double timeAbs);

	/// \~chinese
	/// \brief 获取接线源
	/// \param [out] lineSource 获取硬触发源
	/// \return 获取成功返回true，否则返回 false
	/// \~english
	/// \brief get Specifies the internal signal or physical input Line to use as the trigger source
	/// \param [out] lineSource value
	/// \return success or failure
	bool getLineSource(ELineSource& lineSource);

	/// \~chinese
	/// \brief 设置lineSource接线
	/// \param [in] lineSource 待设置接线
	/// \return 设置成功返回true，否则返回false	
	/// \~english
	/// \brief Set Specifies the internal signal or physical input Line to use as the trigger source
	/// \param [in] lineSource value
	/// \return success or failure	
	bool setLineSource(const ELineSource lineSource);

	/// \~chinese
	/// \brief 获取UserOutputValue对应的寄存器输出比特位
	/// \param [out] userout 获取对应的寄存器输出比特位
	/// \return 获取成功返回true，否则返回 false
	/// \~english
	/// \brief get Selects which bit of the User Output register will be set by UserOutputValue
	/// \param [out] userout value
	/// \return success or failure
	bool getUserOutputSelector(EUserOutputSelector& userOutValue);

	/// \~chinese
	/// \brief 设置UserOutputValue对应的寄存器输出比特位
	/// \param [in] userout 待设置对应的寄存器输出比特位
	/// \return 设置成功返回true，否则返回false	
	/// \~english
	/// \brief Set Selects which bit of the User Output register will be set by UserOutputValue
	/// \param [in] userout value
	/// \return success or failure	
	bool setUserOutputSelector(const EUserOutputSelector userOutValue);

	/// \~chinese
	/// \brief 设置UserOutputValue对应的寄存器输出比特位
	/// \param [in] userout 待设置对应的寄存器输出比特位
	/// \return 设置成功返回true，否则返回false	
	/// \~english
	/// \brief Set Selects which bit of the User Output register will be set by UserOutputValue
	/// \param [in] userout value
	/// \return success or failure	
	bool getLineSource(const EUserOutputSelector userOutValue);

	/// \~chinese
	/// \brief 设置UserOutputValue对应的寄存器输出比特位
	/// \param [in] userout 待设置对应的寄存器输出比特位
	/// \return 设置成功返回true，否则返回false	
	/// \~english
	/// \brief Set Selects which bit of the User Output register will be set by UserOutputValue
	/// \param [in] userout value
	/// \return success or failure	
	bool setLineSource(const EUserOutputSelector userOutValue);

	/// \~chinese
	/// \brief 获取UserOutputValue对应的寄存器输出比特位
	/// \param [in] 1/0 value
	/// \return 返回是否获取成功，成功返回true，失败返回false
	/// \~english
	/// \brief Sets the value of the bit selected by UserOutputSelector
	/// \param [in] true/fail value
	/// \return true means get success, false means get fail
	bool getUserOutputValue(bool& value);

	/// \~chinese
	/// \brief 设置UserOutputValue对应的寄存器输出比特位
	/// \param [in] 1/0 value
	/// \return 返回是否设置成功，成功返回true，失败返回false
	/// \~english
	/// \brief Sets the value of the bit selected by UserOutputSelector
	/// \param [in] true/fail value
	/// \return true means set success, false means set fail
	bool setUserOutputValue(bool value);

	/// \~chinese
	/// \brief 获取块数据
	/// \param [out] 关闭/开启 value
	/// \return 返回是否获取成功，成功返回true，失败返回false
	/// \~english
	/// \brief Enables the extended IDs mode
	/// \param [out] Off/On value
	/// \return true means get success, false means get fail
	bool getGevGVSPExtendedIDMode(EGevGVSPExtendedIDMode& extendedIDMode);

	/// \~chinese
	/// \brief 设置块数据
	/// \param [in] 关闭/开启 value
	/// \return 返回是否设置成功，成功返回true，失败返回false
	/// \~english
	/// \brief Enables the extended IDs mode
	/// \param [in] Off/On value
	/// \return true means set success, false means set fail
	bool setGevGVSPExtendedIDMode(const EGevGVSPExtendedIDMode extendedIDMode);

	/// \~chinese
	/// \brief 读用户自定义数据。相机内部保留32768字节用于用户存储自定义数据（此功能针对本品牌相机，其它品牌相机无此功能）。
	/// \param [out] pBuffer 数据缓冲的指针。
	/// \param [in] dwCount 期望读出的字节数
	/// \return 返回用户实际读取到的字节数
	/// \~english
	/// \brief read user-defined data; Inside the camera, there are 32768 bytes reserved for user to store private data (Only our own camera has this function)
	/// \param [out] pBuffer the point of buffer used to store read out data 
	/// \param [in] dwCount byte count expected to be read out
	/// \return return byte count read in fact
	size_t readUserPrivateData(void* pBuffer, size_t dwCount);

	/// \~chinese
	/// \brief 写用户自定义数据。和readUserPrivateData对应
	/// \param [in] pBuffer 数据缓冲的指针。
	/// \param [in] dwCount 期望写入的字节数
	/// \return 返回用户实际写入的字节数
	/// \~english
	/// \brief write user-defined data; Inside the camera, there are 32768 bytes reserved for user to store private data (Only our own camera has this function)
	/// \param [out] pBuffer the point of buffer 
	/// \param [in] dwCount byte count expected to be written
	/// \return return byte count written in fact
	size_t writeUserPrivateData(const void *pBuffer, size_t dwCount);
	
	/// \~chinese
	/// \brief 保存指定格式的配置到指定文件中
	/// \param [in] pConfigFileName 保存的配置文件名称
	/// \return 保存配置成功返回true，否则返回false
	/// \~english
	/// \brief Save specified config to specified file
	/// \param [in] pConfigFileName config file name
	/// \return success or failure
	bool saveConfigFile(const char* pConfigFileName);
	
	/// \~chinese
	/// \brief  加载指定格式的配置文件
	/// \param [in] pConfigFileName 加载的配置文件名称
	/// \return 加载配置成功返回true，否则返回false
	/// \~english
	/// \brief Load specific config file
	/// \param [in] pConfigFileName config file name
	/// \return success or failure
	bool loadConfigFile(const char* pConfigFileName);
	
	/// \~chinese
	/// \brief  设备连接状态事件回调注册
	/// \param [in] proc  设备连接状态事件回调注册函数
	/// \return 注册是否成功
	/// \~english
	/// \brief regist call back function of camera connection status event. only one call back function is supported
	/// \param [in] proc call back function of camera connection status event
	/// \return success or failure	
	bool subscribeConnectArgs(ConnectArgProc const& proc);

	/// \~chinese
	/// \brief  设备连接状态事件回调去注册
	/// \param [in] proc  设备连接状态事件回调去注册函数
	/// \return 去注册是否成功
	/// \~english
	/// \brief unregister call back function of camera connection status event.
	/// \param [in] proc Unregister call back function of camera connection status event
	/// \return success or failure		
	bool unsubscribeConnectArgs(ConnectArgProc const& proc);

	/// \~chinese
	/// \brief  参数更新事件回调注册
	/// \param [in] proc 参数更新注册的事件回调函数
	/// \return 注册是否成功
	/// \~english
	/// \brief regist call back function of parameter update event. only one call back function is supported.
	/// \param [in] proc call back function of parameter update event
	/// \return success or failure		
	bool subscribeParamUpdate(ParamUpdataProc const& proc);

	/// \~chinese
	/// \brief  参数更新事件回调去注册
	/// \param [in] proc 参数更新事件去注册的回调函数
	/// \return 去注册是否成功
	/// \~english
	/// \brief unregister call back function of parameter update event.
	/// \param [in] proc unregister call back function of parameter update event
	/// \return success or failure		
	bool unsubscribeParamUpdate(ParamUpdataProc const& proc);

	/// \~chinese
	/// \brief  流通道事件回调注册
	/// \param [in] proc 流通道事件回调注册函数
	/// \return 注册是否成功
	/// \~english
	/// \brief register call back function of stream channel event. only one call back function is supported.
	/// \param [in] proc register call back function of stream channel event
	/// \return success or failure		
	bool subscribeStreamArg(StreamArgProc const& proc);

	/// \~chinese
	/// \brief  流通道事件回调去注册
	/// \param [in] proc  流通道事件回调去注册函数
	/// \return 去注册是否成功
	/// \~english
	/// \brief unregister call back function of stream channel event.
	/// \param [in] proc unregister call back function of tream channel event
	/// \return success or failure		
	bool unsubscribeStreamArg(StreamArgProc const& proc);
	
	/// \~chinese
	/// \brief  获取相机对象
	/// \return 相机对象指针
	/// \~english
	/// \brief get camera object
	/// \return pointer of camera object
	ICameraPtr getCamera() const;
	
private:
	/// \~chinese
	/// \brief 构造函数
	/// \~english
	/// \brief constructor function
	CIntegratedCamera(const ICameraPtr &cameraPtr);
	
	/// \~chinese
	/// \brief 拷贝构造函数
	/// \~english
	/// \brief copy constructor function
	CIntegratedCamera(const CIntegratedCamera&); 
	
	/// \~chinese
	/// \brief 赋值函数
	/// \~english
	/// \brief evaluate function	
	CIntegratedCamera& operator = (const CIntegratedCamera&);

	/// \~chinese
	/// \brief 析构函数
	/// \~english
	/// \brief destructor function	
	~CIntegratedCamera();
	
	/// \~chinese
	/// \brief 释放本对象接口
	/// \~english
	/// \brief release integrated object interface		
	void destroyObject(CIntegratedCamera* pObject);
	
	class Impl;
	Impl *m_pImpl;
};

/// @}

GENICAM_NAMESPACE_END

#endif//__GENICAM_CINTEGRATED_CAMERA_H__
