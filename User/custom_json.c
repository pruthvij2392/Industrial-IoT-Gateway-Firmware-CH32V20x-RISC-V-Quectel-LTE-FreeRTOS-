#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include "comman.h"

/**
 * ==============================================================================
 * LIGHTWEIGHT ZERO-ALLOCATION JSON PARSER
 * ==============================================================================
 * Standard JSON parsers (like cJSON) use dynamic heap allocations (malloc/free),
 * which can cause memory fragmentation and potential heap exhaustion on
 * resource-constrained microcontrollers (20KB SRAM).
 * 
 * This module provides simple, safe, in-place string scanning routines to
 * extract string and integer values directly from incoming JSON buffers.
 */

static const char* Find_JSON_Value(const char* json, const char* key);

/**
 * @brief Searches for a specified key within a JSON buffer and locates the start of its value.
 * 
 * Step-by-step logic:
 * 1. Uses strstr() to locate the key substring (e.g. "\"dn\"").
 * 2. Advances the pointer past the length of the key.
 * 3. Skips any whitespace (' '), colons (':'), or opening quotation marks ('\"').
 * 4. Returns a pointer directly to the beginning of the value characters.
 * 
 * @param json Pointer to the null-terminated JSON input string.
 * @param key Null-terminated key string to locate (including quotes, e.g. "\"bpt\"").
 * @return const char* Pointer to the first character of the value, or NULL if key is not found.
 */
static const char* Find_JSON_Value(const char* json, const char* key) {
    const char *p = strstr(json, key);
    if (p) {
        p += strlen(key);
        /* Skip separator characters and opening quote */
        while (*p && (*p == ' ' || *p == ':' || *p == '\"')) p++;
        return p;
    }
    return NULL;
}

/**
 * @brief Extracts a string value associated with a JSON key into a destination buffer.
 * 
 * Copies characters until a closing quotation mark ('\"'), end-of-string ('\0'),
 * or maximum buffer length (`max_len - 1`) is reached, then null-terminates the output.
 * 
 * @param json Pointer to input JSON string.
 * @param key Key name to locate (e.g. "\"dn\"" or "\"bip\"").
 * @param dest Output buffer where extracted string will be stored.
 * @param max_len Maximum capacity of dest buffer (including space for '\0').
 * @return true if the key was found and copied; false otherwise.
 */
bool Get_JSON_String(const char* json, const char* key, char* dest, size_t max_len) {
    const char *p = Find_JSON_Value(json, key);
    if (p) {
        size_t i = 0;
        /* Copy value characters until closing quote or buffer limit */
        while (p[i] && p[i] != '\"' && i < (max_len - 1)) {
            dest[i] = p[i];
            i++;
        }
        dest[i] = '\0'; // Guarantee null-termination
        return true;
    }
    return false;
}

/**
 * @brief Extracts an integer value associated with a JSON key.
 * 
 * Locates the value using Find_JSON_Value() and converts the subsequent
 * digits into an integer using standard atoi().
 * 
 * @param json Pointer to input JSON string.
 * @param key Key name to locate (e.g. "\"bpt\"" or "\"spt\"").
 * @param dest Pointer to destination int32_t variable where converted integer will be stored.
 * @return true if key was found and parsed; false otherwise.
 */
bool Get_JSON_Int(const char* json, const char* key, int32_t* dest) {
    const char *p = Find_JSON_Value(json, key);
    if (p) {
        *dest = atoi(p); // Convert ASCII digits to integer
        return true;
    }
    return false;
}
