#include <rr_platform/udp_sock.h>

using std::string;
using std::cerr;
using std::endl;

namespace rr {

    udp_socket::udp_socket(const string& ip, int port) {
        addr_.sin_family = AF_INET;
        addr_.sin_port = htons(port);
        addr_.sin_addr.s_addr = inet_addr(ip.c_str());

        sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (sockfd_ < 0) {
            cerr << "Socket creation failed" << endl;
            exit(EXIT_FAILURE);
        }

        
    }
   
    udp_socket::~udp_socket() {
        close(sockfd_);
    }
    
    void udp_socket::connect() {
        if (::connect(sockfd_, (struct sockaddr*)&addr_, sizeof(addr_)) < 0) {
            cerr << "Connection Failed" << endl;
            exit(EXIT_FAILURE);
        }
    }
    
    int udp_socket::send(const string& msg) {
        return sendto(sockfd_, msg.c_str(), msg.size(), 0, (struct sockaddr*)&addr_, sizeof(addr_));
    }

    string udp_socket::recv() {
        char buf[1024];
        int addr_len_ = sizeof(addr_);
        std::cout << "wait for receive" << endl;
        int n = recvfrom(sockfd_, buf, 1024, 0, (struct sockaddr*)nullptr, (socklen_t *) &addr_len_);
        if (n < 0) {
            cerr << "Recv failed" << endl;
            exit(EXIT_FAILURE);
        }
        return string(buf, n);
    }

    void udp_socket::bind() {
        //addr_.sin_addr.s_addr = htonl(INADDR_ANY);
        if (::bind(sockfd_, (const struct sockaddr*)&addr_, sizeof(addr_)) < 0) {
            cerr << "Bind failed" << endl;
            exit(EXIT_FAILURE);
        }
    }
}
