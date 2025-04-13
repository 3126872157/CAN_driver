//
// Created by ken on 25-4-6.
//

#ifndef CAN_PROTOCOL_H
#define CAN_PROTOCOL_H

#include <cstdint>
#include <cstring>

#include <vector>
#include <functional>
#include <unordered_map>
#include <mutex>
#include <iostream>

#include "can_driver.h"

class CanProtocol {
public:
    CanProtocol() = default;

    CanProtocol(const std::string &can_interface, const uint32_t bitrate)
        : can_driver_(can_interface, bitrate) {
        can_driver_.spin();
    }

    // Move Constructor
    CanProtocol(CanProtocol &&other) noexcept {
        can_driver_ = std::move(other.can_driver_);
        buffer_map_ = std::move(other.buffer_map_);
    }

    CanProtocol &operator=(CanProtocol &&other) noexcept {
        if (this != &other) {
            can_driver_ = std::move(other.can_driver_);
            buffer_map_ = std::move(other.buffer_map_);
        }
        return *this;
    }

    template<typename MsgType>
    void registerCallback(uint32_t can_id, const std::function<void(const MsgType &)> callback) {
        std::lock_guard<std::mutex> lock(mutex_);
        can_driver_.registerCallback(can_id, [callback,can_id,this](const std::vector<uint8_t> &frame_data) {
            if (frame_data.size() != 8) return false;

            // NOTE：多帧拼接
            // uint8_t id = frame_data[0]
            // uint8_t total_size = frame_data[1];
            // size_t chunk_offset = id * 6;
            //
            // std::lock_guard<std::mutex> lock(mutex_);
            // auto &buffer = buffer_map_[can_id];
            // if (id == 0) {
            //     receive_size_ = 0;
            //     buffer.resize(total_size);
            // }
            //
            // size_t chunk_size = std::min(size_t(6), total_size - chunk_offset);
            // if (chunk_offset + chunk_size <= buffer.size()) {
            //     std::memcpy(buffer.data() + chunk_offset, frame_data.data() + 2, chunk_size);
            //     receive_size_ += chunk_size;
            // }
            // if (receive_size_ >= total_size) {
            //     if (buffer.size() != sizeof(MsgType)) {
            //         std::cerr << "Error: buffer size mismatch: " << buffer.size()
            //                 << " vs " << sizeof(MsgType) << std::endl;
            //         buffer.clear();
            //         return false;
            //     }
            //
            //     MsgType msg;
            //     std::memcpy(&msg, buffer.data(), sizeof(MsgType));
            //
            //     //TODO：调用回调
            //     //callback(msg);
            //
            //     receive_size_ = 0;
            //     buffer.clear();
            //     return true;
            // }

            //NOTE：普通版本
            std::lock_guard<std::mutex> lock_callback(mutex_);
            auto &buffer = buffer_map_[can_id];
            buffer.insert(buffer.end(), frame_data.begin(), frame_data.end());
            if (buffer.size() != sizeof(MsgType)) {
                std::cout << "Error: buffer size mismatch: " << buffer.size()
                        << " vs " << sizeof(MsgType) << std::endl;
                buffer.clear();
                return false;
            }
            MsgType msg;
            std::memcpy(&msg, buffer.data(), sizeof(MsgType));
            callback(msg);
            buffer.clear();
            return true;
        });
    }

    template<typename T>
    std::vector<std::vector<uint8_t> > CanFormat(const T &msg) {
        const auto *data = reinterpret_cast<const uint8_t *>(&msg);
        size_t total_size = sizeof(T);
        std::vector<std::vector<uint8_t> > frames;

        for (size_t offset = 0, id = 0; offset < total_size; offset += 6, ++id) {
            std::vector<uint8_t> frame(8);
            frame[0] = static_cast<uint8_t>(id); // 包编号
            frame[1] = (offset + 6 >= total_size) ? 1 : 0; // 是否最后一包

            size_t chunk_size = std::min(size_t(6), total_size - offset);
            std::memcpy(frame.data() + 2, data + offset, chunk_size);

            frames.push_back(frame);
        }

        return frames;
    }

    //NOTE：重载一个普通版的 CanFormat，没有多帧拼接
    template<typename T>
    void canFormat(std::vector<uint8_t> &pack, const T &data) {
        const auto *bytePtr = reinterpret_cast<const uint8_t *>(&data);
        for (size_t i = 0; i < sizeof(T); ++i) {
            pack.push_back(bytePtr[i]);
        }
    }

    template<typename T>
    bool sendMessage(uint32_t can_id, const T &msg) {
        auto frames = CanFormat(msg);
        for (const auto &frame: frames) {
            if (!can_driver_.write(can_id, frame)) {
                return false;
            }
        }
        return true;
    }

    template<typename MsgType>
    bool parseFrame(const uint32_t can_id, const std::vector<uint8_t> &frame_data) {
        if (frame_data.size() < 2) return false;

        uint8_t id = frame_data[0];
        uint8_t total_size = frame_data[1];
        size_t chunk_offset = id * 6;

        std::lock_guard<std::mutex> lock(mutex_);
        auto &buffer = buffer_map_[can_id];
        if (id == 0) {
            receive_size_ = 0;
            buffer.resize(total_size);
        }

        size_t chunk_size = std::min(size_t(6), total_size - chunk_offset);
        if (chunk_offset + chunk_size <= buffer.size()) {
            std::memcpy(buffer.data() + chunk_offset, frame_data.data() + 2, chunk_size);
            receive_size_ += chunk_size;
        }
        if (receive_size_ >= total_size) {
            if (buffer.size() != sizeof(MsgType)) {
                std::cerr << "Error: buffer size mismatch: " << buffer.size()
                        << " vs " << sizeof(MsgType) << std::endl;
                buffer.clear();
                return false;
            }

            MsgType msg;
            std::memcpy(&msg, buffer.data(), sizeof(MsgType));

            //TODO：调用回调
            //callback(msg);

            receive_size_ = 0;
            buffer.clear();
            return true;
        }

        return false;
    }

private:
    CanDriver can_driver_;

    size_t receive_size_ = 0;
    std::unordered_map<uint32_t, std::vector<uint8_t> > buffer_map_;
    std::mutex mutex_;
};

#endif //CAN_PROTOCOL_H
