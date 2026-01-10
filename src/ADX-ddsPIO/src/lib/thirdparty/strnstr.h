#ifndef STRNSTR_H_
#define STRNSTR_H_

//*fix* char * strnstr(const char *s, const char *find, size_t slen);

#ifndef THIRD_PARTY_STRNSTR_H
#define THIRD_PARTY_STRNSTR_H

#include <stddef.h>

#ifndef HAVE_STRNSTR
char *strnstr(const char *s, const char *find, size_t slen);
#endif

#endif


#endif
