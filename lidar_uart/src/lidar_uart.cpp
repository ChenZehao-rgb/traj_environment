#include <ros/ros.h>
#include <libserial/SerialPort.h>
#include <lidar_uart/Distance.h>

using namespace LibSerial;

int main(int argc, char**​ argv) {
    // 初始化 ROS 节点
    ros::init(argc, argv, "lidar_publisher_cpp");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<lidar_uart_cpp::Distance>("lidar_data_cpp", 10);

    // 配置串口参数
    SerialPort serial;
    try {
        serial.Open("/dev/ttyUSB0");  // 设备路径可能需要修改
        serial.SetBaudRate(BAUD_115200);
        serial.SetCharacterSize(CharacterSize::CHAR_SIZE_8);
        serial.SetFlowControl(FlowControl::FLOW_CONTROL_NONE);
        serial.SetParity(Parity::PARITY_NONE);
        serial.SetStopBits(StopBits::STOP_BITS_1);
    } catch (const OpenFailed&) {
        ROS_ERROR("Failed to open serial port!");
        return -1;
    }

    // 数据接收缓冲区
    uint8_t buffer[9] = {0};
    size_t bytes_read = 0;

    ros::Rate rate(100);  // 100Hz
    while (ros::ok()) {
        try {
            // 读取单个字节直到找到帧头
            if (bytes_read == 0) {
                uint8_t byte;
                serial.ReadByte(byte, 0);
                if (byte != 0x59) continue;
                buffer[bytes_read++] = byte;
            }

            // 读取剩余字节
            if (bytes_read < 9) {
                serial.Read(&buffer[bytes_read], 9 - bytes_read, 0);
                bytes_read = 9;
            }

            // 验证帧头
            if (buffer[0] == 0x59 && buffer[1] == 0x59) {
                // 计算校验和
                uint8_t checksum = 0;
                for (int i = 0; i < 8; i++) checksum += buffer[i];
                checksum &= 0xFF;

                if (checksum == buffer[8]) {
                    // 解析数据（小端模式）
                    uint16_t distance = (buffer[3] << 8) | buffer[2];
                    uint16_t strength = (buffer[5] << 8) | buffer[4];

                    // 转换为米（假设传感器返回毫米）
                    lidar_uart_cpp::Distance msg;
                    msg.distance = distance / 1000.0f;
                    msg.strength = strength;

                    pub.publish(msg);
                } else {
                    ROS_WARN("Checksum error");
                }
            }

            // 重置缓冲区
            bytes_read = 0;
            memset(buffer, 0, sizeof(buffer));

        } catch (const ReadTimeout&) {
            ROS_WARN_THROTTLE(1, "Read timeout");
        } catch (...) {
            ROS_ERROR("Serial communication error");
            break;
        }

        rate.sleep();
    }

    serial.Close();
    return 0;
}