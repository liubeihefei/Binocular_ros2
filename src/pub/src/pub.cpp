// 相机头文件
#include "cam.hpp"

// ros相关头文件
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

// 图像处理相关头文件
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

// 全局变量
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

        // 处理完最后一帧返回
        TST_USBCam_EVENT_LoopMode(GP3DStram, 0);
    }

}

// 发布节点类
class ImagePublisher : public rclcpp::Node
{
private:
    // 声明左右目发布者
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_pub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr right_pub;

public:
    // 构造函数，有一个参数为节点名称
    ImagePublisher(std::string name) : Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "%s节点启动.", name.c_str());
        // 初始化左右目发布者
        left_pub = this->create_publisher<sensor_msgs::msg::Image>("left", 10);
        right_pub = this->create_publisher<sensor_msgs::msg::Image>("right", 10);
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

            return result;
    }
    }

    // 非回调式发布图像
    void publish(std::vector<sensor_msgs::msg::Image>& imgs)
    {
        left_pub->publish(imgs[0]);
        right_pub->publish(imgs[1]);
    }

    // 取图
    int getImg()
    {
        // 使ctrl+c能终止所有进程
        signal(SIGINT,SIG_QUIT);
        signal(SIGQUIT,SIG_QUIT);

        int32_t fd;
        v4l2_dev_sys_data_t *pdevinfo = NULL;

        // 两个0xFFFF返回可用设备数量
        int32_t ret = TST_USBCam_DEVICE_FIND_ID(&pdevinfo, 0xFFFF, 0xFFFF);

        // 无设备则退出
        if((ret == NULL_RETURN) || (ret == 0))
        {
            fprintf(stdout, "NULL Device\r\n");
            
            return -1;
        }

        // 显示可用设备数量
        fprintf(stdout, "device find result %d\r\n", ret);

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
        void* pUSBCam = TST_USBCam_CREATE_DEVICE_POINT(pdevinfo[0]);

        fd = open(pdevinfo[0].Device_Path, O_RDWR | O_NONBLOCK, 0);
        
        /************************************************
            在进行开启视频流之前，需打开设备，并将设备描述符传入，如过之前有打开过，将之前打开过的设备描述符传入即可
        ************************************************/

        // 初始化设备
        int32_t returnVAL = TST_USBCam_Video_DEAL_WITH_INIT(pUSBCam, fd);

        // 初始化设备失败则退出
        if(returnVAL != SUCCESS_RETURN)
        {
            fprintf(stderr, "TST_USBCam_Video_DEAL_WITH_INIT Fail ret %d\r\n", returnVAL);

            return -1;
        }

        Pix_Format *ppix_format;

        // 获取格式、分辨率、帧率
        int32_t pix_format_size = TST_USBCam_Get_Format_List_Size(pUSBCam);

        fprintf(stderr,"pix_format_size:%d\r\n",pix_format_size);

        ppix_format = new Pix_Format[pix_format_size];

        TST_USBCam_Get_Format_List(pUSBCam, ppix_format);

        // 打印所有的格式信息（是指当前相机所有可能的格式、分辨率、帧率组合）
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
        pthread_create(&threadId, NULL, USBCam_STREAM_DEAL, pUSBCam);

        // 设置成阻塞等待相机资源
        TST_USBCam_Video_STREAM_STATUS(pUSBCam, 1);
        
        /************************************************
        在视频流开启后才能进行Processing Unit Control
        ************************************************/
        int32_t pu_val;

        TST_USBCam_PU_Get(pUSBCam, V4L2_CID_EXPOSURE_AUTO, &pu_val);

        fprintf(stderr, "V4L2_EXPOSURE_AUTO val:%d\r\n", pu_val);

        //TST_USBCam_PU_Set(pUSBCam,V4L2_CID_EXPOSURE_AUTO,3); //1:manual 3:AUTO

        if(pu_val == 1)
        {
            TST_USBCam_PU_Get(pUSBCam, V4L2_CID_EXPOSURE_ABSOLUTE, &pu_val);

            fprintf(stderr, "V4L2_CID_EXPOSURE_ABSOLUTE val:%d\r\n", pu_val);

            TST_USBCam_PU_Set(pUSBCam, V4L2_CID_EXPOSURE_ABSOLUTE, pu_val);
        }
        TST_USBCam_PU_Set(pUSBCam, V4L2_CID_EXPOSURE_ABSOLUTE, 0);

        while(EXIT)
        {
            // 获取一帧图像
            Frame_Buffer_Data * pFrame = TST_USBCam_GET_FRAME_BUFF(pUSBCam, 0);

            if(pFrame != NULL)
            {
                // 写入图像
                // char path[256];

                // if(pFrame->PixFormat.u_PixFormat == 0)
                //     {
                //         // snprintf(path, 256,"%d.jpg", pFrame->index);
                //         std::string temp = "temp/" + std::to_string(pFrame->index) + ".jpg";
                //         const char* cstr = temp.c_str();
                //         snprintf(path, 256, cstr);
                //     }
                // else
                //     snprintf(path,256,"%d.bmp", pFrame->index);

                // FILE * File_fd = fopen(path, "w+");

                // if(File_fd == NULL)
                // {
                //     continue;
                // }

                // fwrite(pFrame->pMem,pFrame->buffer.bytesused,1,File_fd);
                // fflush(File_fd);
                // fclose(File_fd);

                // 发布图像
                std::vector<sensor_msgs::msg::Image> pub = this->convert_frame_to_ros_image(pFrame);
                this->publish(pub);

                // if(pFrame->index >= 10000)
                // {
                //     EXIT = 0;
                // }

                fprintf(stderr,"pFrame->index:%02d PixFormat.u_Fps.:%d\r\n",pFrame->index,pFrame->PixFormat.u_Fps);

                // 及时释放？
                TST_USBCam_SAVE_FRAME_RES(pUSBCam,pFrame);
            }
        }
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    // 创建节点
    auto node = std::make_shared<ImagePublisher>("ImagePublisher");

    if(node->getImg())
        return -1;
 
    return 0;
}
