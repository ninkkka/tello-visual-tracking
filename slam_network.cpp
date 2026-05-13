#include <iostream>
#include <fstream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <csignal>
#include <atomic>
#include <cerrno>
#include <cstring>
#include <netinet/tcp.h>
#include <signal.h>

#include "System.h"

std::atomic<bool> running(true);
void signal_handler(int) { running = false; }

// ===== НАДЁЖНАЯ ОТПРАВКА ВСЕХ БАЙТ =====
bool send_all(int sock, const void* data, size_t size) {
    const char* ptr = static_cast<const char*>(data);
    size_t total = 0;
    while (total < size) {
        ssize_t sent = send(sock, ptr + total, size - total, MSG_NOSIGNAL);
        if (sent < 0) {
            if (errno == EINTR) continue;
            std::cerr << "[SEND] Error: " << strerror(errno) << std::endl;
            return false;
        }
        if (sent == 0) {
            std::cerr << "[SEND] Connection closed by peer" << std::endl;
            return false;
        }
        total += sent;
    }
    return true;
}

// ===== НАДЁЖНЫЙ ПРИЁМ ВСЕХ БАЙТ =====
bool recv_all(int sock, void* data, size_t size) {
    char* ptr = static_cast<char*>(data);
    size_t total = 0;
    while (total < size) {
        ssize_t recvd = recv(sock, ptr + total, size - total, MSG_WAITALL);
        if (recvd < 0) {
            if (errno == EINTR) continue;
            std::cerr << "[RECV] Error: " << strerror(errno) << std::endl;
            return false;
        }
        if (recvd == 0) {
            std::cerr << "[RECV] Connection closed by peer" << std::endl;
            return false;
        }
        total += recvd;
    }
    return true;
}

int main(int argc, char** argv) {
    if (argc != 3) {
        std::cerr << "Usage: ./slam_network vocabulary settings" << std::endl;
        return 1;
    }

    // Игнорируем SIGPIPE — чтобы процесс не убивался при разрыве соединения
    std::signal(SIGINT, signal_handler);
    std::signal(SIGPIPE, SIG_IGN);

    ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::MONOCULAR, true);

    // ===== ПОДКЛЮЧЕНИЕ К СТРИМЕРУ (порт 9999) =====
    int sock = socket(AF_INET, SOCK_STREAM, 0);
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(9999);
    inet_pton(AF_INET, "127.0.0.1", &addr.sin_addr);
    if (connect(sock, (sockaddr*)&addr, sizeof(addr)) < 0) {
        std::cerr << "Connection to streamer failed" << std::endl;
        return 1;
    }
    int flag = 1;
    setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag));
    std::cout << "Connected to streamer!" << std::endl;

    // ===== ПОДКЛЮЧЕНИЕ К КОНТРОЛЛЕРУ (порт 9997) =====
    int ctrl_sock = socket(AF_INET, SOCK_STREAM, 0);
    sockaddr_in ctrl_addr{};
    ctrl_addr.sin_family = AF_INET;
    ctrl_addr.sin_port = htons(9997);
    inet_pton(AF_INET, "127.0.0.1", &ctrl_addr.sin_addr);
    if (connect(ctrl_sock, (sockaddr*)&ctrl_addr, sizeof(ctrl_addr)) < 0) {
        std::cerr << "Connection to control failed" << std::endl;
        close(sock);
        return 1;
    }
    setsockopt(ctrl_sock, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag));
    std::cout << "Connected to control!" << std::endl;

    std::vector<uchar> buffer;
    int frame_id = 0;

    while (running) {
        // Принимаем размер кадра
        uint32_t size;
        if (!recv_all(sock, &size, sizeof(size))) break;
        buffer.resize(size);
        if (!recv_all(sock, buffer.data(), size)) break;

        cv::Mat img = cv::imdecode(buffer, cv::IMREAD_GRAYSCALE);
        if (img.empty()) continue;

        double ts = frame_id / 30.0;
        cv::Mat Tcw = SLAM.TrackMonocular(img, ts);

        if (!Tcw.empty()) {
            // Приводим к CV_32F на всякий случай
            cv::Mat Tcw32;
            Tcw.convertTo(Tcw32, CV_32F);

            // Копируем в массив
            float Tcw_data[16];
            for (int i = 0; i < 4; i++)
                for (int j = 0; j < 4; j++)
                    Tcw_data[i * 4 + j] = Tcw32.at<float>(i, j);

            // НАДЁЖНАЯ ОТПРАВКА
            if (!send_all(ctrl_sock, Tcw_data, sizeof(Tcw_data))) {
                std::cerr << "Failed to send Tcw, exiting" << std::endl;
                break;
            }

            // Правильная позиция камеры в мире (для отладки)
            cv::Mat Rcw = Tcw32.rowRange(0, 3).colRange(0, 3);
            cv::Mat tcw = Tcw32.rowRange(0, 3).col(3);
            cv::Mat twc = -Rcw.t() * tcw;
            std::cout << "Camera world: " << twc.at<float>(0) << ", "
                      << twc.at<float>(1) << ", " << twc.at<float>(2) << std::endl;
        }
        frame_id++;
    }

    close(sock);
    close(ctrl_sock);
    SLAM.Shutdown();
    std::cout << "SLAM finished." << std::endl;
    return 0;
}
