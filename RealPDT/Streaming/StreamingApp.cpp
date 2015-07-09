#include "StreamingApp.h"

#include "../Globals.h"
#include <algorithm>
#include <iostream>

StreamingApp::StreamingApp(unsigned int jpg_quality, unsigned int udp_packet_size_kb, std::string streaming_target):
    jpg_quality_(jpg_quality), udp_packet_size_kb_(udp_packet_size_kb)/*, image_name_(image_name)*/
{
    if(streaming_target.compare("iPhone") == 0) {
       stream_width_   = 480;
       stream_height_  = 320;
    } else if(streaming_target.compare("iPhoneRetina") == 0) {
        stream_width_  = 960;
        stream_height_ = 640;
    } else if(streaming_target.compare("iPad") == 0) {
        stream_width_  = 1024;
        stream_height_ = 768;
    } else {
        stream_width_  = 480;
        stream_height_ = 320;
    }

    counter = 0;
    Init();
}

bool StreamingApp::Init()
{
    printf("Init\n");

    char geom[1024];
    sprintf(geom, "%dx%d", stream_width_, stream_height_);
    image_magick_ = new Magick::Image(geom, "red");
 
    addDestIP(Globals::stream_dest_IP.c_str());

    bool ok;
    ok = initUdp();
    return ok;
}

StreamingApp::~StreamingApp()
{
    delete image_magick_;
    closeUdp();
}

void StreamingApp::setDestIP(const char* dest_ip)
{
    dest_.sin_addr.s_addr = inet_addr(dest_ip);
}

void StreamingApp::addDestIP(const char* dest_ip)
{
    dest_ips_.push_back(dest_ip);
}

bool StreamingApp::OnStartUp()
{
    return Init();
}

void StreamingApp::sendImage(unsigned char* img_ptr, int size)
{
    // Copy the image to an imagemagick image
    Magick::Blob blob(img_ptr, size);
    image_magick_->magick("RGB");
    image_magick_->depth(8);
    try {
        image_magick_->read(blob);
    } catch(Magick::Exception &error) {
        printf("Error reading image: %s\n", error.what());
        return;
    }

    // Encode to jpeg
    Magick::Blob blob_out;
    image_magick_->magick("JPEG");
    char geom[1024];
    sprintf(geom, "%ux%u", stream_width_, stream_height_);
    image_magick_->quality(50);

    image_magick_->write(&blob_out);

    // Send
    if(!sendPayloadViaUDP((char*)blob_out.data(), blob_out.length())) {
        printf("Error sending image\n");
    }
}

/**************************************************************************
  * this function sends a payload via UDP to dest_ip
  * if the payload is too big for UDP it gets split up
  * (note that a maximum UDP packetsize of 60kb is _assumed_
  * but the maximum size of one UDP packet should be tested
  * at runtime!)
  *
  * the format of the packages is the following:
  * 32 bit unsigned int: frameNumber: each call to this function
  *                                   will result in a unic frame
  *                                   number (frame because it is
  *                                   only used to transmit data for
  *                                   frames right now)
  * 32 bit unsigned int: totalFrameSize: Total size of all payloads of
  *                                   packages for this frame (if the
  *                                   client recieved this amount of
  *                                   bytes, this frame is complete)
  *
  * 32 bit unsigned int: posOfData:   Position in the Frame where to insert
  *                                   current chunk of data.
  *
  * 32 bit unsigned int: payloadSize: Size of the acual data in this package
  *                                   should be size of the package - header, but
  *                                   it gets send anyway because this way it
  *                                   can be detected if package got truncated.
  *
  **************************************************************************/
bool StreamingApp::sendPayloadViaUDP(char* data, size_t size)
{
    static unsigned int frameNumber = 0;
    static char buffer[64*1024];         //max 64kb images
    static char *lastFrame = NULL;
    static size_t lastSize = 0;
    static unsigned int numberOfPackages = 0;
    size_t totalFrameSize = size;

    static unsigned int maxBytesInOneUDPPacket = udp_packet_size_kb_ * 1024;

    frameNumber++;

    delete lastFrame;
    lastFrame = new char[size];
    lastSize = size;

    const unsigned int HEADER_SIZE = 16;
    unsigned int maxPayloadInOneUDPPacket = maxBytesInOneUDPPacket-HEADER_SIZE; // 4*32bit header

    numberOfPackages = (size / maxPayloadInOneUDPPacket);
    if ((size % maxPayloadInOneUDPPacket) != 0) numberOfPackages++;

    for (unsigned int i = 0; i < numberOfPackages; ++i)
    {
        unsigned int posOfData = i*maxPayloadInOneUDPPacket;

        unsigned int *intBuffer = (unsigned int*) buffer;
        intBuffer[0] = frameNumber;
        intBuffer[1] = totalFrameSize;
        intBuffer[2] = posOfData;

        size_t copySize = (size < maxPayloadInOneUDPPacket)? size:maxPayloadInOneUDPPacket;

        intBuffer[3] = copySize;
        memcpy( (buffer+HEADER_SIZE), (data + posOfData), copySize );

        // try to send out our package (busy wait loop :-( )
        for (int j = 0; j < 1000; j++) {
            unsigned long long bytesWritten = sendUdp(buffer, copySize+HEADER_SIZE);

            if (bytesWritten == (copySize+16)) {
                // all data we wanted to get send are send
                break;
            } else {
                // one common problem for not beeing able to send out all packages is a full write buffer of the OS
                // (128KB on linux for example).  so we wait a little bit and try it again
            }
        }

        size -= copySize;
    }

    return true;
}

// Init the UDP Socket
bool StreamingApp::initUdp()
{
    sock_ = socket(AF_INET, SOCK_DGRAM, 0);
    if(sock_ == -1) {
        return false;
    }
    dest_.sin_family = AF_INET;
    dest_.sin_port = htons(STREAMING_CLIENT_PORT);
    return true;
}

// Close the UDP Socket
void StreamingApp::closeUdp()
{
    close(sock_);
}

// Send a UDP Packet
int StreamingApp::sendUdp(char *data, size_t len)
{
    int res = 0;
    for(unsigned int i=0; i<dest_ips_.size(); ++i) {
        dest_.sin_addr.s_addr = inet_addr(dest_ips_[i].c_str());
        res |= sendto(sock_, data, len, 0, (struct sockaddr *)&dest_, sizeof(dest_));
    }
    return res;
}
