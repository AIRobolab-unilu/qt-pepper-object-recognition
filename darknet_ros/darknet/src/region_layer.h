#ifndef REGION_LAYER_H
#define REGION_LAYER_H

#include "darknet.h"
#include "layer.h"
#include "network.h"

<<<<<<< HEAD
<<<<<<< HEAD
layer make_region_layer(int batch, int w, int h, int n, int classes, int coords);
=======
layer make_region_layer(int batch, int h, int w, int n, int classes, int coords);
>>>>>>> ba4c2b8d6b8dd56d46e2de94840a1b3c5c30f40a
=======
layer make_region_layer(int batch, int h, int w, int n, int classes, int coords);
>>>>>>> origin
void forward_region_layer(const layer l, network net);
void backward_region_layer(const layer l, network net);
void resize_region_layer(layer *l, int w, int h);

#ifdef GPU
void forward_region_layer_gpu(const layer l, network net);
void backward_region_layer_gpu(layer l, network net);
#endif

#endif
