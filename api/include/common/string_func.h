//
// Created by waxz on 4/30/23.
//

#ifndef CMAKE_SUPER_BUILD_STRING_FUNC_H
#define CMAKE_SUPER_BUILD_STRING_FUNC_H

#undef SPLIT_STRING
///char* x;
///SPLIT_STRING(TCC_HEADER,":", x,  tcc_add_include_path(tcc, x));
#define SPLIT_STRING(st,delim,idx, x, lambda_run ) {\
char *SPLIT_STRING_dup = strdup(st);       \
char *SPLIT_STRING_ch;                              \
int SPLIT_STRING_idx = 0;\
SPLIT_STRING_ch = strtok(SPLIT_STRING_dup, delim);      \
while (SPLIT_STRING_ch != NULL){           \
idx =  SPLIT_STRING_idx;                            \
x = SPLIT_STRING_ch;                                \
lambda_run;                                         \
SPLIT_STRING_idx ++;                            \
SPLIT_STRING_ch = strtok(NULL, delim);     \
}                                          \
free(SPLIT_STRING_dup);                    \
}


#include <ctype.h>
#include <errno.h>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>


#ifdef __cplusplus
extern "C" {
#endif


    //https://stackoverflow.com/questions/7021725/how-to-convert-a-string-to-integer-in-c
typedef enum {
    STR2INT_SUCCESS,
    STR2INT_OVERFLOW,
    STR2INT_UNDERFLOW,
    STR2INT_INCONVERTIBLE
} str2int_errno;

/* Convert string s to int out.
 *
 * @param[out] out The converted int. Cannot be NULL.
 *
 * @param[in] s Input string to be converted.
 *
 *     The format is the same as strtol,
 *     except that the following are inconvertible:
 *
 *     - empty string
 *     - leading whitespace
 *     - any trailing characters that are not part of the number
 *
 *     Cannot be NULL.
 *
 * @param[in] base Base to interpret string in. Same range as strtol (2 to 36).
 *
 * @return Indicates if the operation succeeded, or why it failed.
 */
inline str2int_errno str2int(int *out, char *s, int base) {
    char *end;
    if (s[0] == '\0' || isspace(s[0]))
        return STR2INT_INCONVERTIBLE;
    errno = 0;
    long l = strtol(s, &end, base);
    /* Both checks are needed because INT_MAX == LONG_MAX is possible. */
    if (l > INT_MAX || (errno == ERANGE && l == LONG_MAX))
        return STR2INT_OVERFLOW;
    if (l < INT_MIN || (errno == ERANGE && l == LONG_MIN))
        return STR2INT_UNDERFLOW;
    if (*end != '\0')
        return STR2INT_INCONVERTIBLE;
    *out = l;
    return STR2INT_SUCCESS;
}

//https://stackoverflow.com/questions/289347/using-strtok-with-a-stdstring/289354#289354
// use original char* st, will crash in c++
inline void split_str(const char *st, char *delim, int (*func)(int, char *)) {
    char *dup = strdup(st);
    char *ch;
    ch = strtok(dup, delim);
    int count = 0;
    while (ch != NULL) {
        int ret = func(count, ch);
        if (ret == -1) {
            break;
        }
        ch = strtok(NULL, delim);
        count++;
    }
    free(dup);
}

//https://stackoverflow.com/questions/10876930/should-one-never-use-static-inline-function
//https://gist.github.com/htfy96/50308afc11678d2e3766a36aa60d5f75
static inline char* str_concat(const char* s1, const char* s2){

    char* dst_str = (char *) malloc(1 + strlen(s1)+ strlen(s2) );
    strcpy(dst_str, s1);
    strcat(dst_str, s2);
    return dst_str;
}

#ifdef __cplusplus
}
#endif
#endif //CMAKE_SUPER_BUILD_STRING_FUNC_H
