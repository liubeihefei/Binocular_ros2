#include <pthread.h>
#include <system_error>
#include <unistd.h>
#include <memory>
#include <signal.h>
#include <fcntl.h>

#include "USBCam_API.h"		//videoStreaming Control

// ros相关头文件
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

//图像处理相关头文件
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

static int32_t api_get_thread_policy (pthread_attr_t *attr)
{
    int32_t policy;

    int32_t rs = pthread_attr_getschedpolicy (attr, &policy);

    //assert (rs == 0);

    switch (policy)
    {
        case SCHED_FIFO:
            fprintf (stderr,"policy = SCHED_FIFO\n");
            break;
        case SCHED_RR:
            fprintf (stderr,"policy = SCHED_RR");
            break;
        case SCHED_OTHER:
            fprintf (stderr,"policy = SCHED_OTHER\n");
            break;
        default:
            fprintf (stderr,"policy = UNKNOWN\n");
            break;
    }


    return policy;
}

static void api_set_thread_policy (pthread_attr_t *attr)
{
    //int rs = pthread_attr_setschedpolicy (attr, policy);

    api_get_thread_policy (attr);

    struct sched_param param;

     pthread_attr_setschedpolicy(attr,SCHED_FIFO);
     //设置调度参数
     param.sched_priority = 99;

     pthread_attr_setschedparam(attr,&param);

     pthread_attr_setinheritsched(attr,PTHREAD_EXPLICIT_SCHED);

     pthread_detach(pthread_self());

}

void Set_Thread_attr()
{


    pthread_attr_t attr;
    int32_t     rs;
    rs = pthread_attr_init(&attr);

    if(rs == -1)
    {
        fprintf(stderr,"Error pthread_attr_init \r\n");
    }

    // 这里设置进程实际上写死，就是先进先出队列
    api_set_thread_policy (&attr);


}

// 设置视频流，传入设备指针
void* USBCam_STREAM_DEAL(void*pUSBCam)
{
    Set_Thread_attr();

    Pix_Format Cfg;

    Cfg.u_PixFormat = 0;// MJPEG格式
    Cfg.u_Width = 3840;
    Cfg.u_Height = 1080;
    Cfg.u_Fps = 30;

    TST_USBCam_Video_DEAL_WITH         (pUSBCam,Cfg);

    /************************************************
    节点配置解除
    ************************************************/
    TST_USBCam_Video_DEAL_WITH_UNINIT  (pUSBCam);// 关闭设备

    TST_USBCam_DELETE_DEVICE_POINT     (pUSBCam);// 删除设备

    return NULL;

}

void *GP3DStram = NULL;

bool EXIT = 1;

void SIG_QUIT(int signal)
{
    // 外部发送终止信号，如ctrl+c
    if(
       (signal == SIGINT) ||
       (signal == SIGQUIT)
       )
    {
        EXIT = 0;

        TST_USBCam_EVENT_LoopMode(GP3DStram,0);// 处理完最后一帧返回
    }

}

class ImagePublisher : public rclcpp::Node
{
public:
    // 构造函数,有一个参数为节点名称
    ImagePublisher(std::string name) : Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "%s节点启动.", name.c_str());
        // 创建发布者
        left_pub = this->create_publisher<sensor_msgs::msg::Image>("left", 1);
        right_pub = this->create_publisher<sensor_msgs::msg::Image>("right", 1);
    }

    // 转换图像
    std::vector<sensor_msgs::msg::Image> convert_frame_to_ros_image(Frame_Buffer_Data* pFrame) {
        sensor_msgs::msg::Image ros_image;
        
        // 设置头信息
        ros_image.header.stamp = this->now();
        ros_image.header.frame_id = std::to_string(pFrame->index);
        
        // 设置图像基本信息
        ros_image.height = pFrame->PixFormat.u_Height;
        ros_image.width = pFrame->PixFormat.u_Width;
        
        // 根据像素格式处理
        if(pFrame->PixFormat.u_PixFormat == 0) {
            // 将JPEG数据转换为OpenCV Mat
            std::vector<uint8_t> jpeg_data(
                static_cast<uint8_t*>(pFrame->pMem),
                static_cast<uint8_t*>(pFrame->pMem) + pFrame->buffer.bytesused
            );
            
            cv::Mat image = cv::imdecode(jpeg_data, cv::IMREAD_COLOR);

            //裁剪图像
            cv::Mat left = image(cv::Range(0, 1080), cv::Range(0, 1920));
            cv::Mat right = image(cv::Range(0, 1080), cv::Range(1920, 3840));

            std::vector<sensor_msgs::msg::Image> result;
            auto new_left = cv_bridge::CvImage(
                std_msgs::msg::Header(),
                "bgr8",  // 转换为 BGR8 格式
                left
            );

            auto new_right = cv_bridge::CvImage(
                std_msgs::msg::Header(),
                "bgr8",  // 转换为 BGR8 格式
                right
            );

            auto left_msg = new_left.toImageMsg();
            left_msg->header.stamp = this->now();
            left_msg->header.frame_id = std::to_string(pFrame->index);

            auto right_msg = new_right.toImageMsg();
            right_msg->header.stamp = this->now();
            right_msg->header.frame_id = std::to_string(pFrame->index);

            result.push_back(*left_msg);
            result.push_back(*right_msg);

            // if (image.empty()) {
            //     RCLCPP_ERROR(this->get_logger(), "Failed to decode JPEG image");
            // }
            
            // // 使用 cv_bridge 转换为 ROS 消息
            // auto cv_bridge_img = cv_bridge::CvImage(
            //     std_msgs::msg::Header(),
            //     "bgr8",  // 转换为 BGR8 格式
            //     image
            // );
            
            // auto msg = cv_bridge_img.toImageMsg();
            // msg->header.stamp = this->now();
            // msg->header.frame_id = std::to_string(pFrame->index);

            // sensor_msgs::msg::Image image_copy = *msg;  // 拷贝

            return result;
    }
    }

    //发布图像
    void publish(std::vector<sensor_msgs::msg::Image> imgs)
    {
        left_pub->publish(imgs[0]);
        right_pub->publish(imgs[1]);
    }

private:
    // 声明话题发布者
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_pub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr right_pub;
};


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);  // 必须先初始化

    //创建节点
    auto node = std::make_shared<ImagePublisher>("cam");

    // 应该是使ctrl+c能终止所有进程
    signal(SIGINT,SIG_QUIT);
    signal(SIGQUIT,SIG_QUIT);

    int32_t fd;
	
    /************************************************
    video stream control
    ************************************************/

    v4l2_dev_sys_data_t *pdevinfo   = NULL;

    // 两个0xFFFF返回可用设备数量
    int32_t ret =   TST_USBCam_DEVICE_FIND_ID     (&pdevinfo,0xFFFF,0xFFFF);

    if((ret == NULL_RETURN)||(ret == 0))
    {
		fprintf(stdout, "NULL Device\r\n");
		
        return -1;
    }

    fprintf(stdout,"device find result %d\r\n",ret);

    // 打印每个设备的信息
    for(int32_t i = 0;i < ret;i++)
    {
        fprintf(stdout, "VID %04x\r\n", pdevinfo[i].Vid);

        fprintf(stdout, "PID %04x\r\n", pdevinfo[i].Pid);

        fprintf(stdout, "BCD %04x\r\n", pdevinfo[i].Bcd);

        fprintf(stdout, "ST0 %s\r\n",   pdevinfo[i].iManufacturer);

        fprintf(stdout, "ST1 %s\r\n",   pdevinfo[i].iProduct);

        fprintf(stdout, "ST2 %s\r\n",   pdevinfo[i].iSerialNumber);

        fprintf(stdout, "location %s\r\n",   pdevinfo[i].location);
    }

    // 创建设备，这里默认用第一个设备，打印发现确实usb相机在第一个
    void* pUSBCam =
    TST_USBCam_CREATE_DEVICE_POINT     (pdevinfo[0]);

    fd = open(pdevinfo[0].Device_Path, O_RDWR | O_NONBLOCK, 0);
	
	/************************************************
        在进行开启视频流之前，需打开设备，并将设备描述符传入，如过之前有打开过，将之前打开过的设备描述符传入即可
    ************************************************/

    // 初始化设备
    int32_t returnVAL = TST_USBCam_Video_DEAL_WITH_INIT     (pUSBCam,fd);

    if(returnVAL != SUCCESS_RETURN)
    {
        fprintf(stderr,"TST_USBCam_Video_DEAL_WITH_INIT Fail ret %d\r\n",returnVAL);

        return -1;
    }

    Pix_Format *ppix_format;

    // 获取格式、分辨率、帧率
    int32_t pix_format_size = TST_USBCam_Get_Format_List_Size(pUSBCam);

    fprintf(stderr,"pix_format_size:%d\r\n",pix_format_size);

    ppix_format = new Pix_Format[pix_format_size];

    TST_USBCam_Get_Format_List        (pUSBCam,ppix_format);

    // 打印所有的格式信息（不是指当前相机，是指所有可能的格式、分辨率、帧率组合）
    for(int32_t i = 0 ; i < pix_format_size ; i++)
    {
        fprintf(stderr," %C%C%C%C %dx%d %d fps\r\n",
               ppix_format[i].u_PixFormat>>0 &0xFF,
               ppix_format[i].u_PixFormat>>8 &0xFF,
               ppix_format[i].u_PixFormat>>16 &0xFF,
               ppix_format[i].u_PixFormat>>24 &0xFF,
               ppix_format[i].u_Width,
               ppix_format[i].u_Height,
               ppix_format[i].u_Fps);
    }

    pthread_t threadId;

    // 相机的格式、分辨率、帧率在创建进程时设置
    pthread_create(&threadId,   NULL,   USBCam_STREAM_DEAL,  pUSBCam);

    // 设置成阻塞等待相机资源
    TST_USBCam_Video_STREAM_STATUS(pUSBCam,1);
	
	/************************************************
    在视频流开启后才能进行Processing Unit Control
    ************************************************/
    int32_t pu_val;

    TST_USBCam_PU_Get(pUSBCam,V4L2_CID_EXPOSURE_AUTO,&pu_val);

    fprintf(stderr,"V4L2_EXPOSURE_AUTO val:%d\r\n",pu_val);

    //TST_USBCam_PU_Set(pUSBCam,V4L2_CID_EXPOSURE_AUTO,3); //1:manual 3:AUTO

    if(pu_val == 1)
    {
        TST_USBCam_PU_Get(pUSBCam,V4L2_CID_EXPOSURE_ABSOLUTE,&pu_val);

        fprintf(stderr,"V4L2_CID_EXPOSURE_ABSOLUTE val:%d\r\n",pu_val);

        TST_USBCam_PU_Set(pUSBCam,V4L2_CID_EXPOSURE_ABSOLUTE,pu_val);
    }
    TST_USBCam_PU_Set(pUSBCam,V4L2_CID_EXPOSURE_ABSOLUTE,0);

    while(EXIT)
    {
        // 获取一帧图像
        Frame_Buffer_Data*pFrame = TST_USBCam_GET_FRAME_BUFF(pUSBCam, 0);

        if(pFrame != NULL)
        {
            char path[256];
#if 1
            if(pFrame->PixFormat.u_PixFormat == 0)
                {
                    // snprintf(path,256,"%d.jpg",pFrame->index);
                    std::string temp = "/home/horsefly/下载/project/src/cam/temp/" + std::to_string(pFrame->index) + ".jpg";
                    const char* cstr = temp.c_str();
                    snprintf(path, 256, cstr);
                }
            else
                snprintf(path,256,"%d.bmp",pFrame->index);

            // FILE    *File_fd = fopen(path,"w+");

            // if(File_fd == NULL)
            // {
            //     continue;
            // }

            // //写入图像
            // fwrite(pFrame->pMem,pFrame->buffer.bytesused,1,File_fd);
            // fflush(File_fd);
            // fclose(File_fd);

            //发布图像
            std::vector<sensor_msgs::msg::Image> pub = node->convert_frame_to_ros_image(pFrame);
            node->publish(pub);

#endif
            if(pFrame->index >= 10000)
            {
                // fprintf(stderr,"何意味？");
                EXIT = 0;
            }

            fprintf(stderr,"pFrame->index:%02d PixFormat.u_Fps.:%d\r\n",pFrame->index,pFrame->PixFormat.u_Fps);
            TST_USBCam_SAVE_FRAME_RES(pUSBCam,pFrame);
        }
    }

    return 0;

}
