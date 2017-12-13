/* inih -- simple .INI file parser

inih is released under the New BSD license (see LICENSE.txt). Go to the project
home page for more info:

https://github.com/benhoyt/inih

*/

#if defined(_MSC_VER) && !defined(_CRT_SECURE_NO_WARNINGS)
#define _CRT_SECURE_NO_WARNINGS
#endif

//#include <stdio.h>
#include <ctype.h>
#include <string.h>

#include "ini.h"
#include "utils.h"
#include "commands.h"

//#if !INI_USE_STACK
//#include <stdlib.h>
//#endif


/* Strip whitespace chars off end of given string, in place. Return s. */
static char* rstrip(char* s)
{
    char* p = s + strlen(s);
    while (p > s && isspace((unsigned char)(*--p)))
        *p = '\0';
    return s;
}

/* Return pointer to first non-whitespace char in given string. */
static char* lskip(const char* s)
{
    while (*s && isspace((unsigned char)(*s)))
        s++;
    return (char*)s;
}

/* Return pointer to first char (of chars) or inline comment in given string,
   or pointer to null at end of string if neither found. Inline comment must
   be prefixed by a whitespace character to register as a comment. */
static char* find_chars_or_comment(const char* s, const char* chars)
{
#if INI_ALLOW_INLINE_COMMENTS
    int was_space = 0;
    while (*s && (!chars || !strchr(chars, *s)) &&
           !(was_space && strchr(INI_INLINE_COMMENT_PREFIXES, *s))) {
        was_space = isspace((unsigned char)(*s));
        s++;
    }
#else
    while (*s && (!chars || !strchr(chars, *s))) {
        s++;
    }
#endif
    return (char*)s;
}


char* ini_buffer_reader(char* str, int num, void* stream)
{
    buffer_ctx* ctx = (buffer_ctx*)stream;
    int idx = 0;
    char newline = 0;

    if (ctx->bytes_left <= 0)
        return NULL;

    for (idx = 0; idx < num - 1; ++idx)
    {
        if (idx == ctx->bytes_left)
            break;

        if (ctx->ptr[idx] == '\n')
            newline = '\n';
        else if (ctx->ptr[idx] == '\r')
            newline = '\r';

        if (newline)
            break;
    }

    memcpy(str, ctx->ptr, idx);
    str[idx] = 0;

    ctx->ptr += idx + 1;
    ctx->bytes_left -= idx + 1;

    if (newline && ctx->bytes_left > 0 &&
            ((newline == '\r' && ctx->ptr[0] == '\n') ||
             (newline == '\n' && ctx->ptr[0] == '\r'))) {
        ctx->bytes_left--;
        ctx->ptr++;
    }
    return str;
}


/* See documentation in header file. */
int ini_parse_stream(ini_reader reader, void* stream, ini_handler handler,
                     void* user)
{
    /* Uses a fair bit of stack (use heap instead if you need to) */
#if INI_USE_STACK
    char line[INI_MAX_LINE];
#else
    char* line;
#endif
    char section[MAX_SECTION] = "";
    char prev_name[MAX_NAME] = "";
//	char msg[mainMAX_MSG_LEN];

    char* start;
    char* end;
    char* name;
    char* value;
    int lineno = 0;
    int error = 0;
    int nSectionNo = 0; // sections counter [ericv]

#if !INI_USE_STACK
    line = (char*)pvPortMalloc(INI_MAX_LINE);
    if (!line) {
        return -2;
    }
#endif

    /* Scan through stream line by line */
    while (reader(line, INI_MAX_LINE, stream) != NULL) {
        lineno++;

        start = line;
#if INI_ALLOW_BOM
        if (lineno == 1 && (unsigned char)start[0] == 0xEF &&
                           (unsigned char)start[1] == 0xBB &&
                           (unsigned char)start[2] == 0xBF) {
            start += 3;
        }
#endif
        start = lskip(rstrip(start));

        if (*start == ';' || *start == '#') {
            /* Per Python configparser, allow both ; and # comments at the
               start of a line */
        }
#if INI_ALLOW_MULTILINE
        else if (*prev_name && *start && start > line) {
            /* Non-blank line with leading whitespace, treat as continuation
               of previous name's value (as per Python configparser). */
            if (!handler(user, section, prev_name, start, &nSectionNo) && !error)
                error = lineno;
        }
#endif
        else if (*start == '[') {
            /* A "[section]" line */
            end = find_chars_or_comment(start + 1, "]");
            if (*end == ']') {
                *end = '\0';
                strncpy0(section, start + 1, sizeof(section));
                *prev_name = '\0';
                nSectionNo++;
            }
            else if (!error) {
                /* No ']' found on section line */
                error = lineno;
            }
        }
        else if (*start) {
            /* Not a comment, must be a name[=:]value pair */
            end = find_chars_or_comment(start, "=:");
            if (*end == '=' || *end == ':') {
                *end = '\0';
                name = rstrip(start);
                value = lskip(end + 1);
#if INI_ALLOW_INLINE_COMMENTS
                end = find_chars_or_comment(value, NULL);
                if (*end)
                    *end = '\0';
#endif
                rstrip(value);

                /* Valid name[=:]value pair found, call handler */
                strncpy0(prev_name, name, sizeof(prev_name));
                if (!handler(user, section, name, value, &nSectionNo) && !error) {
                    error = lineno;
                    xprintfMsg("\r\nApplying INI format, error at process section: %s (line: %d, param: %s)\n", section, lineno, name);
                }
            }
            else if (!error) {
                /* No '=' or ':' found on name[=:]value line */
                error = lineno;
                if (*name)
                	xprintfMsg("\r\nParsing INI format, error in line: %d, param: %s\n", lineno, name);
                else
                    xprintfMsg("\r\nParsing INI format, error in line: %d, param: --\n", lineno);

            }
        }

#if INI_STOP_ON_FIRST_ERROR
        if (error)
            break;
#endif
    }


#if !INI_USE_STACK
    vPortFree(line);
#endif

    return error;
}
//
///* See documentation in header file. */
//int ini_parse_file(FILE* file, ini_handler handler, void* user)
//{
//    return ini_parse_stream((ini_reader)fgets, file, handler, user);
//}
//
///* See documentation in header file. */
//int ini_parse(const char* filename, ini_handler handler, void* user)
//{
//    FILE* file;
//    int error;
//
//    file = fopen(filename, "r");
//    if (!file)
//        return -1;
//    error = ini_parse_file(file, handler, user);
//    fclose(file);
//    return error;
//}
