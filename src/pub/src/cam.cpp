#include "cam.hpp"

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

