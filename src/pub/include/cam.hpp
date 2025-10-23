#include <pthread.h>
#include <system_error>
#include <unistd.h>
#include <memory>
#include <signal.h>
#include <fcntl.h>

// videoStreaming Control
#include "USBCam_API.h"


// 函数声明
static int32_t api_get_thread_policy (pthread_attr_t *attr);

static void api_set_thread_policy (pthread_attr_t *attr);

void Set_Thread_attr();

void* USBCam_STREAM_DEAL(void* pUSBCam);

void SIG_QUIT(int signal);
