// 相机头文件
#include "cam.hpp"

// ros相关头文件
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"

// 图像处理相关头文件
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

// Socket相关头文件
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <vector>
#include <sys/stat.h>
#include <chrono>

// 全局变量
void *GP3DStram = NULL;
bool EXIT = 1;

// Socket相关全局变量
int socket_fd = -1;
bool socket_connected = false;

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
        
        // 关闭socket连接
        if (socket_fd >= 0) {
            close(socket_fd);
            socket_fd = -1;
            socket_connected = false;
        }
    }
}

// 检查socket文件是否存在
bool socket_exists() {
    struct stat buffer;
    return (stat("/tmp/bino_img.sock", &buffer) == 0);
}

// 连接到Unix Socket服务器（使用阻塞模式）
bool connect_to_socket() {
    if (socket_connected && socket_fd >= 0) {
        return true; // 已经连接
    }
    
    // 检查socket文件是否存在
    if (!socket_exists()) {
        return false; // socket文件不存在，服务器没启动
    }
    
    // 创建socket
    socket_fd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (socket_fd < 0) {
        fprintf(stderr, "Failed to create socket: %s\n", strerror(errno));
        return false;
    }
    
    // 设置为阻塞模式（默认就是阻塞，但为了明确设置）
    int flags = fcntl(socket_fd, F_GETFL, 0);
    fcntl(socket_fd, F_SETFL, flags & ~O_NONBLOCK);
    
    // 连接到服务器
    struct sockaddr_un addr;
    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, "/tmp/bino_img.sock", sizeof(addr.sun_path) - 1);
    
    // 连接（阻塞模式，会等待连接完成或失败）
    int result = connect(socket_fd, (struct sockaddr*)&addr, sizeof(addr));
    if (result < 0) {
        if (errno != ECONNREFUSED && errno != ENOENT) {
            fprintf(stderr, "Failed to connect to socket: %s\n", strerror(errno));
        }
        close(socket_fd);
        socket_fd = -1;
        return false;
    }
    
    socket_connected = true;
    fprintf(stderr, "Connected to socket server at /tmp/bino_img.sock\n");
    return true;
}

// 检查socket连接状态并尝试重连
void check_and_reconnect_socket() {
    if (!socket_connected) {
        // 尝试连接
        connect_to_socket();
    }
    // 对于阻塞socket，不需要检查连接状态
    // 如果连接断开，下次发送时会收到错误
}

// 通过socket发送图像数据（按照要求格式：8字节时间戳 + 8字节图像大小 + 图像数据）
bool send_image_via_socket(const cv::Mat& image, uint64_t timestamp) {
    if (!socket_connected || socket_fd < 0) {
        return false; // 没有连接
    }
    
    // 计算图像数据大小（使用step确保考虑内存对齐）
    uint64_t image_size = image.step * image.rows;
    
    // 调试信息
    static int frame_count = 0;
    frame_count++;
    fprintf(stderr, "[Socket #%d] Sending image: %dx%d, size: %lu bytes\n",
           frame_count, image.cols, image.rows, image_size);
    
    // 准备发送缓冲区（头部+图像数据）
    size_t total_size = 16 + image_size; // 16字节头部 + 图像数据
    std::vector<uint8_t> send_buffer(total_size);
    
    // 填充头部：8字节时间戳 + 8字节图像大小
    uint64_t* header = reinterpret_cast<uint64_t*>(send_buffer.data());
    header[0] = timestamp;    // 时间戳
    header[1] = image_size;   // 图像大小
    
    // 复制图像数据
    memcpy(send_buffer.data() + 16, image.data, image_size);
    
    // 一次性发送所有数据（阻塞发送）
    const uint8_t* data_ptr = send_buffer.data();
    size_t remaining = total_size;
    
    while (remaining > 0) {
        ssize_t bytes_sent = send(socket_fd, data_ptr, remaining, MSG_NOSIGNAL);
        
        if (bytes_sent <= 0) {
            if (bytes_sent < 0) {
                // 发送错误
                if (errno == EPIPE || errno == ECONNRESET) {
                    fprintf(stderr, "Socket connection broken: %s\n", strerror(errno));
                } else {
                    fprintf(stderr, "Failed to send data: %s\n", strerror(errno));
                }
            }
            
            // 关闭连接
            close(socket_fd);
            socket_fd = -1;
            socket_connected = false;
            return false;
        }
        
        data_ptr += bytes_sent;
        remaining -= bytes_sent;
    }
    
    fprintf(stderr, "[Socket #%d] Image sent successfully: %lu bytes\n", 
           frame_count, total_size);
    return true;
}

// 发布节点类
class ImagePublisher : public rclcpp::Node
{
private:
    // 声明左右目发布者
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_pub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr right_pub;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_pub;

    // 声明一个字符串发布者，跟图像同时发布，用于观察频率
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr str_pub;

    // 存储最新的左右目图像
    sensor_msgs::msg::Image left_img;
    sensor_msgs::msg::Image right_img;

    sensor_msgs::msg::Image img;

    // 控制发布频率（按目前来看，只对MPEG 3280*1080 30FPS进行控制，只需取对15取余为0的帧即可做到2hz）
    int cnt = 6;

    // 计算频率/间隔
    uint64_t last = 0;

public:
    // 构造函数，有一个参数为节点名称
    ImagePublisher(std::string name) : Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "%s节点启动.", name.c_str());
        
        // 检查socket服务器是否可用
        if (socket_exists()) {
            fprintf(stderr, "Socket server found at /tmp/bino_img.sock, will connect when needed\n");
        } else {
            fprintf(stderr, "No socket server found at /tmp/bino_img.sock\n");
        }
        
        // // 初始化左右目发布者
        // left_pub = this->create_publisher<sensor_msgs::msg::Image>("/bino/left", 10);
        // right_pub = this->create_publisher<sensor_msgs::msg::Image>("/bino/img", 10);

        img_pub = this->create_publisher<sensor_msgs::msg::Image>("/bino/img", 10);

        str_pub = this->create_publisher<std_msgs::msg::String>("hz", 10);
    }

    // 转换图像，双目各自发布
    void convert_frame_to_ros_image_split(Frame_Buffer_Data* pFrame) {
        // 统一时间戳
        auto stamp = this->now();

        // 打印两次发布图片时间间隔
        uint64_t nanoseconds = stamp.nanoseconds();
        if(last == 0)
            fprintf(stderr,"first pub\r\n");
        else
            fprintf(stderr,"img pub interval %.2fms\r\n", (nanoseconds - last) / 1e6);
        last = nanoseconds;

        // 获取图像宽高
        int width = pFrame->PixFormat.u_Width;
        int height = pFrame->PixFormat.u_Height;
        
        // 处理MJPEG格式
        if(pFrame->PixFormat.u_PixFormat == 0) {
            // 将MJPEG数据转换为OpenCV Mat
            std::vector<uint8_t> jpeg_data(
                static_cast<uint8_t*>(pFrame->pMem),
                static_cast<uint8_t*>(pFrame->pMem) + pFrame->buffer.bytesused
            );
            
            cv::Mat image = cv::imdecode(jpeg_data, cv::IMREAD_COLOR);

            //裁剪图像
            cv::Mat left = image(cv::Range(0, height), cv::Range(0, width / 2));
            cv::Mat right = image(cv::Range(0, height), cv::Range(width / 2, width));

            cv::Mat left_flip, right_flip;
            cv::flip(left, left_flip, -1);
            cv::flip(right, right_flip, -1);

            left = right_flip;
            right = left_flip;

            // 通过socket发送左右图像（按新格式）
            if (socket_connected) {
                send_image_via_socket(left, nanoseconds);
                send_image_via_socket(right, nanoseconds);
            }

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
            left_msg->header.stamp = stamp;
            left_msg->header.frame_id = std::to_string(pFrame->index);

            auto right_msg = new_right.toImageMsg();
            right_msg->header.stamp = stamp;
            right_msg->header.frame_id = std::to_string(pFrame->index);

            left_img = *left_msg;
            right_img = *right_msg;
        }
        // 处理YUYV格式
        else{
            int width = pFrame->PixFormat.u_Width;
            int height = pFrame->PixFormat.u_Height;
            
            // 创建YUYV格式的Mat
            cv::Mat yuyv_image(height, width, CV_8UC2, pFrame->pMem);
            
            // 转换为BGR格式
            cv::Mat bgr_image;
            cv::cvtColor(yuyv_image, bgr_image, cv::COLOR_YUV2BGR_YUYV);
            
            // 裁剪图像为左右两个
            cv::Mat left = bgr_image(cv::Range(0, height), cv::Range(0, width/2));
            cv::Mat right = bgr_image(cv::Range(0, height), cv::Range(width/2, width));

            // 通过socket发送左右图像（按新格式）
            if (socket_connected) {
                send_image_via_socket(left, nanoseconds);
                send_image_via_socket(right, nanoseconds);
            }

            // 转换为ROS图像消息
            auto new_left = cv_bridge::CvImage(
                std_msgs::msg::Header(),
                "bgr8",
                left
            );

            auto new_right = cv_bridge::CvImage(
                std_msgs::msg::Header(),
                "bgr8",
                right
            );

            auto left_msg = new_left.toImageMsg();
            left_msg->header.stamp = stamp;
            left_msg->header.frame_id = std::to_string(pFrame->index);

            auto right_msg = new_right.toImageMsg();
            right_msg->header.stamp = stamp;
            right_msg->header.frame_id = std::to_string(pFrame->index);

            left_img = *left_msg;
            right_img = *right_msg;
        }
    }

    // 转换图像，双目合并发布
    void convert_frame_to_ros_image_all(Frame_Buffer_Data* pFrame) {
        // 统一时间戳
        auto stamp = this->now();

        // 打印两次发布图片时间间隔
        uint64_t nanoseconds = stamp.nanoseconds();
        if(last == 0)
            fprintf(stderr,"first pub\r\n");
        else
            fprintf(stderr,"img pub interval %.2fms\r\n", (nanoseconds - last) / 1e6);
        last = nanoseconds;
        
        cv::Mat final_image;
        
        // 处理MJPEG格式
        if(pFrame->PixFormat.u_PixFormat == 0) {
            // 将MJPEG数据转换为OpenCV Mat
            std::vector<uint8_t> jpeg_data(
                static_cast<uint8_t*>(pFrame->pMem),
                static_cast<uint8_t*>(pFrame->pMem) + pFrame->buffer.bytesused
            );
            
            cv::Mat image = cv::imdecode(jpeg_data, cv::IMREAD_COLOR);

            cv::flip(image, final_image, -1);
        }
        // 处理YUYV格式
        else{
            int width = pFrame->PixFormat.u_Width;
            int height = pFrame->PixFormat.u_Height;
            
            // 创建YUYV格式的Mat
            cv::Mat yuyv_image(height, width, CV_8UC2, pFrame->pMem);
            
            // 转换为BGR格式
            cv::cvtColor(yuyv_image, final_image, cv::COLOR_YUV2BGR_YUYV);
        }
        
        // 通过socket发送完整图像（按新格式）
        if (socket_connected) {
            send_image_via_socket(final_image, nanoseconds);
        }

        // 转换为ROS图像消息
        auto new_img = cv_bridge::CvImage(
            std_msgs::msg::Header(),
            "bgr8",
            final_image
        );

        auto img_msg = new_img.toImageMsg();
        img_msg->header.stamp = stamp;
        img_msg->header.frame_id = std::to_string(pFrame->index);

        img = *img_msg;
    }

    // 非回调式发布图像，双目各自发布
    void publish_split()
    {
        left_pub->publish(left_img);
        right_pub->publish(right_img);

        std_msgs::msg::String s;
        s.data = "oi";
        str_pub->publish(s);
    }

    // 非回调式发布图像，双目合并发布
    void publish_all()
    {
        img_pub->publish(img);

        std_msgs::msg::String s;
        s.data = "oi";
        str_pub->publish(s);
    }

    // 循环取图
    int getImg()
    {
        // 使ctrl+c能终止所有进程
        signal(SIGINT, SIG_QUIT);
        signal(SIGQUIT, SIG_QUIT);

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

        fprintf(stderr,"pix_format_size:%d\r\n", pix_format_size);

        ppix_format = new Pix_Format[pix_format_size];

        TST_USBCam_Get_Format_List(pUSBCam, ppix_format);

        // 打印所有的格式信息（是指当前相机所有可能的格式、分辨率、帧率组合）
        for(int32_t i = 0; i < pix_format_size; i++)
        {
            fprintf(stderr," %c%c%c%c %dx%d %d fps\r\n",
                ppix_format[i].u_PixFormat>>0 & 0xFF,
                ppix_format[i].u_PixFormat>>8 & 0xFF,
                ppix_format[i].u_PixFormat>>16 & 0xFF,
                ppix_format[i].u_PixFormat>>24 & 0xFF,
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
            // 检查和维护socket连接
            check_and_reconnect_socket();
            
            // 监测相机是否在线
            int32_t ret = TST_USBCam_DEVICE_FIND_ID(&pdevinfo, 0xFFFF, 0xFFFF);

            // 无设备则退出
            if((ret == NULL_RETURN) || (ret == 0))
            {
                fprintf(stdout, "NULL Device\r\n");
                
                EXIT = 0;
            }

            // 获取一帧图像
            Frame_Buffer_Data * pFrame = TST_USBCam_GET_FRAME_BUFF(pUSBCam, 0);

            if(pFrame != NULL)
            {
                // 控制频率发布图像
                if(pFrame->index % cnt == 0)
                {
                    // this->convert_frame_to_ros_image_split(pFrame);
                    // this->publish_split();

                    this->convert_frame_to_ros_image_all(pFrame);
                    this->publish_all();

                    // this->convert_frame_to_ros_image_split(pFrame);
                    // this->publish_split();
                }

                // 及时释放？
                TST_USBCam_SAVE_FRAME_RES(pUSBCam, pFrame);
            }
        }
        
        // 清理socket资源
        if (socket_fd >= 0) {
            close(socket_fd);
            socket_fd = -1;
            socket_connected = false;
        }
        
        return 0;
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    // 创建节点
    auto node = std::make_shared<ImagePublisher>("ImagePublisher");

    if(node->getImg() == -1)
        return -1;
 
    return 0;
}