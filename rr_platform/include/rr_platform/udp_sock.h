#pragma once

#include <iostream>
#include <fstream>
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

namespace rr {

class udp_socket {
    public:
        udp_socket(const std::string& ip, int port);
        ~udp_socket();

        int send(const std::string& msg);
        std::string recv();
        void bind();
    
    private:
        int sockfd_;
        struct sockaddr_in addr_;
};

}