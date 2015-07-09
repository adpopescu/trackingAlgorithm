#include <sys/time.h>

// NOTE: you need to link with -lrt when using this

typedef struct {
	timespec start;
	timespec end;
} Timer;

extern void startTimer(Timer*);
extern void stopTimer(Timer*);
extern double getTimerValue(Timer*);
