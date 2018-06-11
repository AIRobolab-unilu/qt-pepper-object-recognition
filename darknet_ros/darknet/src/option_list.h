#ifndef OPTION_LIST_H
#define OPTION_LIST_H
#include "list.h"

typedef struct{
    char *key;
    char *val;
    int used;
} kvp;


int read_option(char *s, list *options);
void option_insert(list *l, char *key, char *val);
char *option_find(list *l, char *key);
<<<<<<< HEAD
<<<<<<< HEAD
=======
int option_find_int_quiet(list *l, char *key, int def);
>>>>>>> ba4c2b8d6b8dd56d46e2de94840a1b3c5c30f40a
=======
int option_find_int_quiet(list *l, char *key, int def);
>>>>>>> origin
float option_find_float(list *l, char *key, float def);
float option_find_float_quiet(list *l, char *key, float def);
void option_unused(list *l);

#endif
