#include <Can/Can.h>

int main()
{
    int a = 1;
    printf("(1*10)/100 = %d\n", (1*10)/100);
    printf("(1*10)%100 = %d\n", (1*10)%100);
    printf("(0xFF*10)/100 = 0x%X\n", 0xFF*0.1);
    printf("(0xFF*10)/100 = 0x%X\n", (0xFF*10)/100);
    printf("(0xFF*10)%100 = 0x%X\n", (0xFF*10)%100);
    // 创建 SocketCAN 套接字
    int socketDescriptor = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socketDescriptor == -1) {
        std::cerr << "Failed to create socket" << std::endl;
        return 1;
    }
    
    // 绑定到 can0 接口
    struct ifreq ifr;
    std::strcpy(ifr.ifr_name, "can0");
    if (ioctl(socketDescriptor, SIOCGIFINDEX, &ifr) == -1) {
        std::cerr << "Failed to get CAN interface index" << std::endl;
        close(socketDescriptor);
        return 1;
    }
    printf("ifr.ifr_ifindex = 0x%X\n",ifr.ifr_ifindex);

    struct sockaddr_can addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(socketDescriptor, (struct sockaddr*)&addr, sizeof(addr)) == -1) {
        std::cerr << "Failed to bind socket to CAN interface" << std::endl;
        close(socketDescriptor);
        return 1;
    }

    // 等待一定时间以接收 CAN 消息
    int timeoutSeconds = 1;
    int timeoutMicroseconds = 0;
    fd_set readSet;
    FD_ZERO(&readSet);
    FD_SET(socketDescriptor, &readSet);
    struct timeval timeout;
    timeout.tv_sec = timeoutSeconds;
    timeout.tv_usec = timeoutMicroseconds;
    int selectResult = select(socketDescriptor + 1, &readSet, NULL, NULL, &timeout);
    if (selectResult == -1) {
        std::cerr << "Error occurred while waiting for CAN message" << std::endl;
        close(socketDescriptor);
        return 1;
    } else if (selectResult == 0) {
        std::cout << "Timeout occurred while waiting for CAN message" << std::endl;
        // close(socketDescriptor);
        // return 1;
    }

    //准备 CAN 消息
    struct can_frame frame;
    std::memset(&frame, 0, sizeof(frame));
    frame.can_id = 0x610;  // 设置 CAN ID
    frame.can_dlc = 8;     // 设置数据长度为 8 字节

    // 设置数据
    char ctrl[] = {0x2F, 0x01, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00};
    char GetTemperature[8] = {0x40, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00};
    char GetTemperatureAck[4] = {0x4F, 0x00, 0x20, 0x00};
    std::memcpy(frame.data, GetTemperature, 8);

    // 发送 CAN 消息
    if (write(socketDescriptor, &frame, sizeof(frame)) == -1) {
        std::cerr << "Failed to send CAN message" << std::endl;
        close(socketDescriptor);
        return 1;
    }

    // 接收 CAN 消息
    struct can_frame receivedFrame;
    ssize_t bytesRead;
    while(true)
    {
        memset(&receivedFrame, 0, sizeof(can_frame));
        bytesRead = 0;
        bytesRead = read(socketDescriptor, &receivedFrame, sizeof(receivedFrame));
        printf("bytesRead = %ld       ", bytesRead);
        printf("Received CAN response: ");
        for (int i = 0; i < receivedFrame.can_dlc; ++i) {
            printf("0x%02X ", receivedFrame.data[i]);
        }
        
        if(!strncmp((const char*)receivedFrame.data, (const char*)GetTemperatureAck, 4))
        {
            float a = (float)receivedFrame.data[4];
            printf("temprature = %f\n", a);
        }
        printf("\n");
    }
    

    // 关闭套接字
    close(socketDescriptor);

    return 0;
}