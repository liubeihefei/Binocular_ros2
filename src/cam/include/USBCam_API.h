






/** @mainpage 概述
 *
 *@section 当前API
 *@version Version 1.5.3(build: July 10 2025 15:37:12)
 *
 *@section 文档发布记录
 *@subsection Ver1 2022-03-24(1.00)
 *@subsection Ver2 2025-07-02(1.10)
 *-# 版本更新
 *-# 更新内容：新增设备分辨率格式&分辨率大小&帧率信息获取
 *@subsection Ver3 2025-07-10(1.53)
 *-# 版本更新
 *-# 更新内容：优化IO周期时间,尤其是高帧率
*/



#ifndef USBCam_API_H
#define USBCam_API_H


#include <stdio.h>
#include <stdint.h>
#include <float.h>
#include <math.h>

#include <stdio.h>
#include <stdint.h>
#include <float.h>
#include <math.h>
#include <linux/videodev2.h>
#include <memory.h>



#ifndef SUCCESS_RETURN
#define SUCCESS_RETURN              (0)
#define FAIL_RETURN                 (-1)

#define NULL_RETURN                 (-2)
#define MES_PAIR_ERROR_RETURN       (-4)
#endif


#define String_Max_Lenght (256)

/**@brief 	设备系统信息枚举时获取
*/

typedef struct _v4l2_dev_sys_data_t
{




    char        *device;

    char        *name;

    char        *driver;
    /**@brief 	设备物理路径
    **@details	在多个设备情况下区分设备
    */
    char        *location;

    uint32_t    vendor;

    uint32_t    product;

    int32_t     valid;

    int32_t     current;

    uint64_t    busnum;

    uint64_t    devnum;

    /**@brief 	设备VID
    **@details
    */
    uint16_t                Vid;

    /**@brief 	设备PID
    **@details
    */
    uint16_t                Pid;

    /**@brief 	设备Bcd
    **@details
    */
    uint16_t                Bcd;

    char                    iManufacturer   [String_Max_Lenght];

    char                    iProduct        [String_Max_Lenght];

    char                    iSerialNumber   [String_Max_Lenght];
    /**@brief 	设备文件路径
    **@details	/dev/video*
    */
    char                    Device_Path     [String_Max_Lenght];

} v4l2_dev_sys_data_t,*p_V4l2_dev_sys_data_t;


/**@brief 	数据流格式
*/
struct Pix_Format
{
    /**@brief 	数据流格式 0：MJPEG; 1:YUYV
    */
    uint32_t    u_PixFormat;
    /**@brief 	数据流帧宽度
    */
    uint32_t    u_Width;
    /**@brief 	数据流帧高度
    */
    uint32_t    u_Height;
    /**@brief 	数据流帧率
    */
    uint32_t    u_Fps;
};



/**@brief 	帧数据
*/
struct Frame_Buffer_Data
{

    struct  v4l2_buffer buffer;

    /**@brief 	数据流格式
    */
    struct  Pix_Format          PixFormat;


    /**@brief 	获取帧的时间
    */
    struct timeval      Frame_Time;


    int32_t             fd;

    char                device_path[String_Max_Lenght];

    /**@brief 	帧数据缓存
    */
    void*               pMem;


    uint32_t            buff_Length;

    uint32_t            buff_Offset;

    uint32_t            index;
};
/**@defgroup TSTC_Camera_API API
*@{
*/

#ifdef __cplusplus
extern "C" {
#endif


const char*
TST_USBCam_Log();


void
TST_USBCam_Set_Log(const char*str,uint8_t size);


char*
TST_USBCam_Get_Cache();


/**@brief 	以ID的方式，查找配对的设备数量
*@param[in] ppDevList		返回设备列表索引方式(*ppDevList)[0~返回设备的数量]
*@param[in] uVID	目标设备VID
*@param[in] uPID	目标设备PID
*@note	当uVID和uPID 同时都为0xFFFF时，将返回所有USB Camera
*@return -1:没有设备
*@return val>=0:匹配的设备数量
*/
int32_t
TST_USBCam_DEVICE_FIND_ID     (v4l2_dev_sys_data_t**ppDevList,
                            uint16_t uVID,
                            uint16_t uPID);



/**@brief 	创建一个设备节点
*@param[in] sDevinfo		设备信息
*@return 	:设备节点(这一步并不会打开设备，因此不会返回失败)
*/
void*
TST_USBCam_CREATE_DEVICE_POINT     (v4l2_dev_sys_data_t sDevinfo);

/**@brief 	删除一个设备节点
*@param[in] pDevPoint		设备节点
*@return 	:(无返回)
*/

void
TST_USBCam_DELETE_DEVICE_POINT     (void*pDevPoint);



/**@brief	获取数据流状态(用于等待数据流启动)
*@param[in] pDevPoint		设备节点
*@param[in] bBlock			阻塞模式，节点资源存在互斥锁(0:如果没有帧资源，不等待帧资源立即返回NULL。1:如果没有帧资源，等待帧资源返回)
*@return 	NULL_RETURN：	节点为空
*@return 	>=0：			资源数量
*/

int32_t
TST_USBCam_Video_STREAM_STATUS         (void*pDevPoint,bool block);



/**@brief 	从帧缓存中获取一帧数据(请在另一条线程中使用)
*@param[in] pDevPoint		设备节点
*@param[in] bBlock			阻塞模式，节点资源存在互斥锁(0:如果没有帧资源，不等待帧资源立即返回NULL。1:如果没有帧资源，等待帧资源返回)
*@return 	NULL：	没有获取到帧数据
*@return 	!NULL:	帧数据
*/
struct Frame_Buffer_Data*
TST_USBCam_GET_FRAME_BUFF          	(void*pDevPoint,    bool bBlock);


/**@brief 	把已经使用的帧数据保存至帧资源(回收帧资源)
*@param[in] pDevPoint		设备节点
*@param[in] pFrame			帧资源，由同一设备节点TST_USBCam_GET_FRAME_BUFF()获取的非空资源
*@return 	无返回
*/
void
TST_USBCam_SAVE_FRAME_RES          	(void*pDevPoint,struct Frame_Buffer_Data *pFrame);


/**@brief 	USBCam数据流处理(创建缓存->枚举配置数据流->打开数据流->循环读取计算->关闭数据流->清空缓存,需要在线程中挂起)
*@param[in] pDevPoint		设备节点
*@param[in] arg             用于设置分辨率格式、分辨率大小
*@return 	NULL_RETURN：	节点为空
*@return 	FAIL_RETURN：	处理异常(没有初始化或初始化失败)
*@return 	SUCCESS_RETURN：成功
*/
int32_t
TST_USBCam_Video_DEAL_WITH             (void*pDevPoint,Pix_Format arg);

/**@brief 	USBCam数据流循环(请在另一条线程中使用，或于TST_USBCam_Video_DEAL_WITH前调用)
*@param[in] pDevPoint		设备节点
*@param[in] iVal			等待帧事件次数(当 iVAl == -1时无限循环，当 iVAl == 0时TST_USBCam_Video_DEAL_WITH()在处理完最后一帧后关闭数据流返回。其他值处理帧数量)
*@return 	无返回
*/
void
TST_USBCam_EVENT_LoopMode              (void*pDevPoint,int32_t iVal);


/**@brief 	USBCam节点配置初始化(打开设备文件,在TST_USBCam_Video_DEAL_WITH前调用)
*@param[in] pDevPoint				设备节点
*@param[in] fd						设备打开描述符
*@return 	SUCCESS_RETURN：		初始化成功
*@return 	FAIL_RETURN：			失败打开设备异常
*/
int32_t
TST_USBCam_Video_DEAL_WITH_INIT        (void*pDevPoint,int32_t fd);

/**@brief 	USBCam节点配置解除(释放控制节点->关闭设备文件)
*@param[in] pDevPoint				设备节点
*@return 	无返回
*/
void
TST_USBCam_Video_DEAL_WITH_UNINIT      (void*pDevPoint);

/**@brief 	USBCam PU控制设置参数目标值
*@param[in] id	PU ID
*@param[in] val	目标值
*@return 	SUCCESS_RETURN；成功；
*@return	FAIL_RETURN：失败；
*@return	NULL_RETURN 设备节点为空
*/
int32_t
TST_USBCam_PU_Set        (void*pDevPoint,int32_t id,int32_t val);

/**@brief 	USBCam PU控制获取参数当前值
*@param[in] id	PU ID
*@param[in] val	当前值
*@return 	SUCCESS_RETURN；成功；
*@return	FAIL_RETURN：失败；
*@return	NULL_RETURN 设备节点为空
*/
int32_t
TST_USBCam_PU_Get        (void*pDevPoint,int32_t id,int32_t *val);

/**@brief 							格式&分辨率&帧率 信息获取
*@param[in] pDevPoint				视频流节点
*@param[out] *pix_format			格式&分辨率&帧率
*@return 	SUCCESS_RETURN			成功
*@return	NULL_RETURN 			设备节点为空
*/
int32_t
TST_USBCam_Get_Format_List			(void*pDevPoint, Pix_Format *pix_format);

/**@brief 							格式&分辨率&帧率 列表长度获取
*@param[in] pDevPoint				视频流节点
*@param[out] *pix_format			格式&分辨率&帧率
*@return 	非NULL_RETURN			长度
*@return	NULL_RETURN 			设备节点为空
*/
int32_t
TST_USBCam_Get_Format_List_Size	(void*pDevPoint);

/** @} end of TSTC_USBCam_API */

#ifdef __cplusplus
}
#endif


#endif // CAMERA_API_H
