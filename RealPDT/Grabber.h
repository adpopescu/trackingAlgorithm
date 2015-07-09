#ifndef GRABBER_H
#define GRABBER_H

#include <OpenNI.h>
#include <pthread.h>

#define MAX_DEPTH 10000

class Grabber
{
public:
    Grabber(void (*call_back_function)(const float *, const unsigned char *));
    ~Grabber();
    virtual uint init();
    void start();
    void stop();

    void (*CallBackFunction)(const float * depth, const unsigned char * image);
protected:
    openni::VideoFrameRef		m_depthFrame;
    openni::VideoFrameRef		m_colorFrame;

    openni::Device			m_device;
    openni::VideoStream			m_depthStream;
    openni::VideoStream			m_colorStream;
    openni::VideoStream**		m_streams;
private:
    bool is_running;
    static void *thread_function(void *owner);
    void calculateHistogram(float* pHistogram, int histogramSize, const openni::VideoFrameRef& frame);
    void grabb(float *pDepth, unsigned char *pImage, int &changedIndex);

    unsigned int		m_width;
    unsigned int		m_height;
    float*	m_pDepth1;
    openni::RGB888Pixel*	m_pImage;

    pthread_t grabb_thread;
};

#endif // GRABBER_H
