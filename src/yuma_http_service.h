#ifndef YUMA_HTTP_SERVICE_H
#define YUMA_HTTP_SERVICE_H

void initYumaHttpService();
bool downloadYumaAlmanac();

// Almanac management helpers
enum { YUMA_WEEK_UNKNOWN = -1 };
int getYumaFileWeek();
int getCurrentGpsWeek();
bool isYumaFileCurrent();
bool ensureYumaAlmanacCurrent();

#endif // YUMA_HTTP_SERVICE_H 