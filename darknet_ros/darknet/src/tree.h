#ifndef TREE_H
#define TREE_H
#include "darknet.h"

<<<<<<< HEAD
<<<<<<< HEAD
=======
tree *read_tree(char *filename);
>>>>>>> ba4c2b8d6b8dd56d46e2de94840a1b3c5c30f40a
=======
tree *read_tree(char *filename);
>>>>>>> origin
int hierarchy_top_prediction(float *predictions, tree *hier, float thresh, int stride);
float get_hierarchy_probability(float *x, tree *hier, int c, int stride);

#endif
