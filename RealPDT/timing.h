//
// Created by dtenty on 10/08/15.
//

#ifndef REALPDT_TIMING_H
#define REALPDT_TIMING_H

#include <sys/time.h>
#include <sys/resource.h>

inline double CPUTime()
{
    struct rusage ruse;
    //    getrusage(RUSAGE_SELF,&ruse);
    getrusage(RUSAGE_THREAD,&ruse);

    return ( ruse.ru_utime.tv_sec + ruse.ru_stime.tv_sec +
             1e-6 * (ruse.ru_utime.tv_usec + ruse.ru_stime.tv_usec) );
}

#endif //PROJECT_TIMING_H
