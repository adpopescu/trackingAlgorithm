#ifndef STREAMINGAPP_DENNIS_H
#define STREAMINGAPP_DENNIS_H

#include <unistd.h>
#include <sys/time.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <vector>

#include <string.h>
#include <stdio.h>

#include <Magick++.h>

class StreamingApp{//: public CMOOSApp {
    public:
        StreamingApp(unsigned int jpg_quality, unsigned int udp_packet_size_kb, std::string streaming_target);
        virtual ~StreamingApp();
        bool Init();
        void setDestIP(const char* dest_ip);
        void addDestIP(const char* dest_ip);

        // Send image to streaming client
        void sendImage(unsigned char *img_ptr, int size);
    protected:
        //called when we are starting up..
        bool OnStartUp();

        // Send data via UDP
        bool sendPayloadViaUDP(char* data, size_t size);

        // Init the UDP Socket
        bool initUdp();

        // Close the UDP Socket
        void closeUdp();

        // Send a UDP Packet
        int sendUdp(char *data, size_t len);

        struct timeval last_time_;

        Magick::Image *image_magick_;

        int sock_;
        struct sockaddr_in dest_;

        unsigned int jpg_quality_, udp_packet_size_kb_;
        uint stream_width_, stream_height_;

        std::vector<std::string> dest_ips_;

        static const uint STREAMING_CLIENT_PORT = 45454;

        int counter;
};

#endif
