//
// Created by liangjun on 25-4-5.
//

#ifndef CAN_DRIVER_H
#define CAN_DRIVER_H

#include <cstdint>
#include <cstring>
#include <unistd.h>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <fcntl.h>

#include <iostream>
#include <utility>
#include <vector>
#include <functional>
#include <mutex>

#include <linux/can.h>
#include <linux/can/raw.h>

#define can_driver1_open       "sudo modprobe can_raw"
#define can_driver2_open       "sudo modprobe mttcan"
#define ip_cmd_close           "sudo ip link set can0 down"
#define ip_cmd_set_can_params  "sudo ip link set can0 type can bitrate 1000000 berr-reporting on"
#define ip_cmd_open            "sudo ip link set can0 up"

class CanDriver {
public:
    using CallbackFunc = std::function<void(const std::vector<uint8_t> &)>;

    explicit CanDriver(const std::string &interface = "can0", int bitrate = 1000000) {
        bitrate_ = bitrate;
        running_ = false;

        openCanInterface(interface, bitrate);

        //创建套接口 sock_fd
        int sock_fd = socket(AF_CAN, SOCK_RAW, CAN_RAW);
        if (sock_fd < 0) {
            throw std::runtime_error("Failed to open CAN socket");
        }

        //将套接字与 can 口绑定
        struct ifreq ifr{};
        strcpy(ifr.ifr_name, interface.c_str());
        if (ioctl(sock_fd, SIOCGIFINDEX, &ifr) < 0) {
            close(sock_fd);
            throw std::runtime_error("Failed to get interface index");
        }
        ifr.ifr_ifindex = if_nametoindex(ifr.ifr_name);

        struct sockaddr_can addr{};
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;
        if (bind(sock_fd, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
            throw std::runtime_error("Failed to bind CAN socket");
        }

        //定义接收过滤规则，加这句就跑不通了，md
        // setsockopt(sock_fd, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);

        //设置read()和write()函数设置为非堵塞方式
        int flags = fcntl(sock_fd, F_GETFL);
        flags |= O_NONBLOCK;
        fcntl(sock_fd, F_SETFL, flags);

        std::cout << "[Can Driver] " << "CAN init success!" << std::endl;
        can_port_ = sock_fd;
    }

    ~CanDriver() {
        running_ = false;
        if (recv_thread_.joinable()) {
            recv_thread_.join();
        }
        close(can_port_);
    }

    CanDriver(CanDriver &&other) noexcept {
        can_port_ = other.can_port_;
        bitrate_ = other.bitrate_;
        running_.exchange(other.running_);
        recv_thread_ = std::move(other.recv_thread_);
        callbacks_ = std::move(other.callbacks_);

        other.can_port_ = -1;
        other.running_ = false;
        if (other.recv_thread_.joinable()) {
            other.recv_thread_.join();
        }
    }

    CanDriver& operator=(CanDriver &&other) noexcept {
        if (this != &other) {
            if (recv_thread_.joinable()) {
                recv_thread_.join();
            }
        }
        close(can_port_);

        can_port_ = other.can_port_;
        bitrate_ = other.bitrate_;
        running_.exchange(other.running_);
        recv_thread_ = std::move(other.recv_thread_);
        callbacks_ = std::move(other.callbacks_);

        other.can_port_ = -1;
        other.running_ = false;

        return *this;
    }

    CanDriver(const CanDriver &) = delete;
    CanDriver &operator=(const CanDriver &) = delete;

    //TODO：单例模式
    //TODO：异常

public:
    bool write(uint32_t can_id, const std::vector<uint8_t> &data) const {
        struct can_frame tx_frame{};

        tx_frame.can_id = can_id;
        tx_frame.can_dlc = std::min(static_cast<int>(data.size()), 8);
        std::memcpy(tx_frame.data, data.data(), tx_frame.can_dlc);

        int len = ::write(can_port_, &tx_frame, sizeof(struct can_frame));
        return len == sizeof(struct can_frame);
    }

    void registerCallback(uint32_t can_id, CallbackFunc callback) {
        std::lock_guard<std::mutex> lock(callback_mutex_);
        callbacks_[can_id] = std::move(callback);
    }

    void spin() {
        running_ = true;
        recv_thread_ = std::thread(&CanDriver::receiveLoop, this);
    }

private:
    void receiveLoop() {
        struct can_frame frame{};
        while (running_) {
            int len = read(can_port_, &frame, sizeof(struct can_frame));
            if (len < 0) {
                continue;
            }

            std::lock_guard<std::mutex> lock(callback_mutex_);
            auto it = callbacks_.find(frame.can_id);
            if (it != callbacks_.end()) {
                std::vector<uint8_t> data(frame.data, frame.data + frame.can_dlc);
                it->second(data);
            }
            //TODO：可以考虑yield
            //std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    void openCanInterface(const std::string &interface, const int bitrate) {
        //NOTE：目前开启 CAN 通信需要在终端输入以下命令
        system(can_driver1_open);
        system(can_driver2_open);
        system(ip_cmd_close);
        std::string set_can_params = "sudo ip link set " + interface + " type can bitrate " + std::to_string(bitrate);
        std::system(set_can_params.c_str());
        std::string open_interface = "sudo ip link set " + interface + " up";
        std::system(open_interface.c_str());
        std::cout << "[Can Driver] " << "CAN set param success!" << std::endl;
    }

private:
    int can_port_;
    int bitrate_ = 1000000;

    std::atomic<bool> running_;
    std::thread recv_thread_;

    std::unordered_map<uint32_t, CallbackFunc> callbacks_;
    std::mutex callback_mutex_;
};

#endif //CAN_DRIVER_H
