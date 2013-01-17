#include <stdio.h>
#include <stdlib.h>
#include <strings.h>
#include <values.h>
#include "strlcpy.h"
typedef double Prediction_t;
#define reset_pred()  {p = 0.0;}
#define add_pred(X) {p += (X);}
#define finalize_pred()  ((r) ? (r[1] = p , r[0] = -p) : -p)
#define defined_attr(X)  (attr[X])
#define int_attr(X)      (*((int *) attr[X]))
#define attr_contains_token(X,Y) (tokens[X][Y])
#define double_attr(X)   (*((double *) attr[X]))
#define WHITE_CHARS   " \t\n"
typedef struct hash_table_entry_s
{
    char *key;
    int id;
    struct hash_table_entry_s *next;
} HashTableEntry_t;
static char *keys[] =
{
};
#define num_keys  (0)
static HashTableEntry_t **hash_table = NULL;
#define hash_table_size  (1031)
static char *tokens[1];
static int text_attr[] = {};
#define num_text_attr  (0)
static char **text_patterns[] =
{
};
static char **words;
static int num_words;
static int cur_word;
static char *pattern;
static int pattern_len;
static void set_pattern(int n, char **m_w, char *pat)
{
    words = m_w;
    pattern = pat;
    cur_word = 0;
    num_words = n;
    pattern_len = strlen(pat);
}
#define more_tokens()  (cur_word <= num_words - pattern_len)
#define ADD_CHAR(S)  { \
  if (c >= buffer_size) { \
    buffer_size = 2 * buffer_size + 1; \
    buffer = (char *) realloc(buffer, buffer_size * sizeof(char)); \
  } \
  buffer[c++] = (S); \
}
static char *next_token(void)
{
    static char *buffer = NULL;
    static int buffer_size = 0;
    int i, c;
    char *s;
    c = 0;
    for (s = pattern; *s; s++)
        ADD_CHAR(*s);
    for (i = 0; i < pattern_len; i++)
    {
        if (pattern[i] == '1')
        {
            ADD_CHAR(' ');
            for (s = words[cur_word + i]; *s; s++)
                ADD_CHAR(*s);
        }
    }
    ADD_CHAR('\0');
    cur_word++;
    return buffer;
}
static int
hash(char *s)
{
    static int *coef = NULL;
    static int max_len = 0;
    int len = strlen(s);
    int i, h;
    if (len > max_len)
    {
        i = max_len;
        max_len = 2 * len;
        coef = (int *) realloc(coef, max_len * sizeof(int));
        for (; i < max_len; i++)
            coef[i] = random() % hash_table_size;
    }
    h = 0;
    for (i = 0; i < len; i++)
    {
        if (h >= MAXINT - (hash_table_size - 1) * 255)
            h = h % hash_table_size;
        h += coef[i] * s[i];
    }
    return (h % hash_table_size);
}
