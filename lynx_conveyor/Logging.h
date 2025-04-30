#ifndef LOGGING_H
#define LOGGING_H

struct LoggingManagement {
    unsigned long previousLogTime;  // Time of last log
    unsigned long logInterval;      // Interval (ms) between logs, 0 = disabled
};

extern LoggingManagement logging;
extern const unsigned long DEFAULT_LOG_INTERVAL;

void logSystemState();

#endif // LOGGING_H

