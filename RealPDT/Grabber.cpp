// Undeprecate CRT functions
#ifndef _CRT_SECURE_NO_DEPRECATE
#define _CRT_SECURE_NO_DEPRECATE 1
#endif

#include "Grabber.h"

Grabber::Grabber(void (*call_back_function)(const float *, const unsigned char *)):
    CallBackFunction(call_back_function), m_streams(NULL)/*, m_pDepth(NULL)*/, m_pDepth1(NULL), m_pImage(NULL)
{
    is_running = false;
    uint rc = openni::STATUS_OK;

    const char* deviceURI = openni::ANY_DEVICE;

    rc = openni::OpenNI::initialize();

    rc = m_device.open(deviceURI);
    m_device.setDepthColorSyncEnabled(true);
    if (rc != openni::STATUS_OK)
    {
        printf("SimpleViewer: Device open failed:\n%s\n", openni::OpenNI::getExtendedError());
        openni::OpenNI::shutdown();
        exit(1);
    }

    rc = m_depthStream.create(m_device, openni::SENSOR_DEPTH);
    openni::VideoMode vm = m_depthStream.getVideoMode();
    vm.setResolution(640,480);
    m_depthStream.setVideoMode(vm);
    if (rc == openni::STATUS_OK)
    {
        rc = m_depthStream.start();
        if (rc != openni::STATUS_OK)
        {
            printf("SimpleViewer: Couldn't start depth stream:\n%s\n", openni::OpenNI::getExtendedError());
            m_depthStream.destroy();
        }
    }
    else
    {
        printf("SimpleViewer: Couldn't find depth stream:\n%s\n", openni::OpenNI::getExtendedError());
    }

    rc = m_colorStream.create(m_device, openni::SENSOR_COLOR);
    vm = m_colorStream.getVideoMode();
    vm.setResolution(640,480);
    m_colorStream.setVideoMode(vm);
    if (rc == openni::STATUS_OK)
    {
        rc = m_colorStream.start();
        if (rc != openni::STATUS_OK)
        {
            printf("SimpleViewer: Couldn't start color stream:\n%s\n", openni::OpenNI::getExtendedError());
            m_colorStream.destroy();
        }
    }
    else
    {
        printf("SimpleViewer: Couldn't find color stream:\n%s\n", openni::OpenNI::getExtendedError());
    }

    if (!m_depthStream.isValid() || !m_colorStream.isValid())
    {
        printf("SimpleViewer: No valid streams. Exiting\n");
        openni::OpenNI::shutdown();
        exit(2);
    }
    m_device.setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR);
    m_depthStream.setMirroringEnabled(false);
    m_colorStream.setMirroringEnabled(false);

    rc = init();
    if (rc != openni::STATUS_OK)
    {
        openni::OpenNI::shutdown();
        printf("SimpleViewer: Initialization failed.");
        exit(3);
    }
}

Grabber::~Grabber()
{
    delete[] m_pDepth1; delete[] m_pImage;

    if (m_streams != NULL)
    {
        delete []m_streams;
    }
}

uint Grabber::init()
{
    openni::VideoMode depthVideoMode;
    openni::VideoMode colorVideoMode;

    if (m_depthStream.isValid() && m_colorStream.isValid())
    {
        depthVideoMode = m_depthStream.getVideoMode();
        colorVideoMode = m_colorStream.getVideoMode();

        int depthWidth = depthVideoMode.getResolutionX();
        int depthHeight = depthVideoMode.getResolutionY();
        int colorWidth = colorVideoMode.getResolutionX();
        int colorHeight = colorVideoMode.getResolutionY();

        if (depthWidth == colorWidth &&
                depthHeight == colorHeight)
        {
            m_width = depthWidth;
            m_height = depthHeight;
        }
        else
        {
            printf("Error - expect color and depth to be in same resolution: D: %dx%d, C: %dx%d\n",
                   depthWidth, depthHeight,
                   colorWidth, colorHeight);
            return openni::STATUS_ERROR;
        }
    }
    else if (m_depthStream.isValid())
    {
        depthVideoMode = m_depthStream.getVideoMode();
        m_width = depthVideoMode.getResolutionX();
        m_height = depthVideoMode.getResolutionY();
    }
    else if (m_colorStream.isValid())
    {
        colorVideoMode = m_colorStream.getVideoMode();
        m_width = colorVideoMode.getResolutionX();
        m_height = colorVideoMode.getResolutionY();
    }
    else
    {
        printf("Error - expects at least one of the streams to be valid...\n");
        return openni::STATUS_ERROR;
    }

    m_streams = new openni::VideoStream*[2];
    m_streams[0] = &m_depthStream;
    m_streams[1] = &m_colorStream;

    m_pDepth1 = new float[m_width * m_height*sizeof(float)];
    m_pImage = new openni::RGB888Pixel[m_width * m_height];

    return openni::STATUS_OK;
}

void* Grabber::thread_function(void* owner)
{
    Grabber* _this = (Grabber*)owner;
    float* pDepth=new float[_this->m_width*_this->m_height*sizeof(float)];
    unsigned char* pImage=new unsigned char[_this->m_width*_this->m_height*3];
    int ch1=-1,ch2=-1;
    while(_this->is_running)
    {
        _this->grabb(pDepth, pImage,ch1);
        _this->grabb(pDepth, pImage,ch2);
        if(ch1>=0 && ch2>=0 && ch1!=ch2)
            _this->CallBackFunction(pDepth, pImage);
        ch1=ch2=-1;
    }
    delete[] pDepth;
    delete[] pImage;
    pthread_exit(NULL);
}

void Grabber::start()
{
    if(!is_running)
    {
        is_running = true;
        if(pthread_create(&grabb_thread, NULL, thread_function, (void*)this))
        {
            printf("Error: unable to create grabb thread.\n");
            exit(1);
        }
    }
}

void Grabber::stop()
{
    if(is_running)
    {
        is_running = false;

        pthread_join(grabb_thread, NULL);

        printf("Stoping depth stream.\n");
        m_depthStream.stop();
        printf("Destroying depth stream.\n");
        m_depthStream.destroy();
        printf("Stoping color stream.\n");
        m_colorStream.stop();
        printf("Destroying color stream.\n");
        m_colorStream.destroy();
        printf("Closing device.\n");
        m_device.close();
        printf("Shuting down Openni.\n");
        openni::OpenNI::shutdown();
    }
}

void Grabber::calculateHistogram(float* pHistogram, int histogramSize, const openni::VideoFrameRef& frame)
{
    const openni::DepthPixel* pDepth = (const openni::DepthPixel*)frame.getData();
    // Calculate the accumulative histogram (the yellow display...)
    memset(pHistogram, 0, histogramSize*sizeof(float));
    int restOfRow = frame.getStrideInBytes() / sizeof(openni::DepthPixel) - frame.getWidth();
    int height = frame.getHeight();
    int width = frame.getWidth();

    unsigned int nNumberOfPoints = 0;
    for (int y = 0; y < height; ++y)
    {
        for (int x = 0; x < width; ++x, ++pDepth)
        {
            if (*pDepth != 0)
            {
                pHistogram[*pDepth]++;
                nNumberOfPoints++;
            }
        }
        pDepth += restOfRow;
    }
    for (int nIndex=1; nIndex<histogramSize; nIndex++)
    {
        pHistogram[nIndex] += pHistogram[nIndex-1];
    }
    if (nNumberOfPoints)
    {
        for (int nIndex=1; nIndex<histogramSize; nIndex++)
        {
            pHistogram[nIndex] = (256 * (1.0f - (pHistogram[nIndex] / nNumberOfPoints)));
        }
    }
}

void Grabber::grabb(float* pDepth, unsigned char* pImage,int& changedIndex)
{
    bool got_image=false, got_depth=false;
    uint rc = openni::OpenNI::waitForAnyStream(m_streams, 2, &changedIndex);
    if (rc != openni::STATUS_OK)
    {
        printf("Wait failed\n");
        return;
    }

    switch (changedIndex)
    {
    case 0:
        m_depthStream.readFrame(&m_depthFrame);
        got_depth = true;
        break;
    case 1:
        m_colorStream.readFrame(&m_colorFrame);
        got_image = true;
        break;
    default:
        printf("Error in wait\n");
    }

    memset(m_pDepth1, 0, m_width*m_height*sizeof(float));
    memset(m_pImage, 0, m_width*m_height*sizeof(openni::RGB888Pixel));

    // check if we need to draw image frame to texture
    if (m_colorFrame.isValid())
    {
        const openni::RGB888Pixel* pImageRow = (const openni::RGB888Pixel*)m_colorFrame.getData();
        openni::RGB888Pixel* pTexRow = m_pImage + m_colorFrame.getCropOriginY() * m_width;
        int rowSize = m_colorFrame.getStrideInBytes() / sizeof(openni::RGB888Pixel);

        for (int y = 0; y < m_colorFrame.getHeight(); ++y)
        {
            const openni::RGB888Pixel* pImage = pImageRow;
            openni::RGB888Pixel* pTex = pTexRow + m_colorFrame.getCropOriginX();

            for (int x = 0; x < m_colorFrame.getWidth(); ++x, ++pImage, ++pTex)
            {
                *pTex = *pImage;
            }

            pImageRow += rowSize;
            pTexRow += m_width;
        }
        memcpy(pImage,(unsigned char*)m_pImage,m_width*m_height*3);

    }

    if (m_depthFrame.isValid())
    {
        const openni::DepthPixel* pDepthRow = (const openni::DepthPixel*)m_depthFrame.getData();
        float* pTexRow1 = m_pDepth1 + m_depthFrame.getCropOriginY() * m_width;
        int rowSize = m_depthFrame.getStrideInBytes() / sizeof(openni::DepthPixel);

        for (int y = 0; y < m_depthFrame.getHeight(); ++y)
        {
            const openni::DepthPixel* pDepth = pDepthRow;
            float* pTex1 = pTexRow1 + m_depthFrame.getCropOriginX();

            for (int x = 0; x < m_depthFrame.getWidth(); ++x, ++pDepth, /*++pTex,*/++pTex1)
            {
                if (*pDepth != 0)
                {
                    *pTex1 = (float)*pDepth/1000;
                }
            }

            pDepthRow += rowSize;
            pTexRow1 += m_width;
        }
        memcpy(pDepth,m_pDepth1,m_width*m_height*sizeof(float));

    }
}
