#ifndef LOGUTILS_H_
#define LOGUTILS_H_

//*fix* void StampPrintf(const char* pformat, ...);
void StampPrintf(const char *pformat, ...)
    __attribute__((format(printf, 1, 2)));


#endif
