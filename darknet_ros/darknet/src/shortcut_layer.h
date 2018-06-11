#ifndef SHORTCUT_LAYER_H
#define SHORTCUT_LAYER_H

#include "layer.h"
#include "network.h"

layer make_shortcut_layer(int batch, int index, int w, int h, int c, int w2, int h2, int c2);
void forward_shortcut_layer(const layer l, network net);
void backward_shortcut_layer(const layer l, network net);
<<<<<<< HEAD
<<<<<<< HEAD
void resize_shortcut_layer(layer *l, int w, int h);
=======
>>>>>>> ba4c2b8d6b8dd56d46e2de94840a1b3c5c30f40a
=======
>>>>>>> origin

#ifdef GPU
void forward_shortcut_layer_gpu(const layer l, network net);
void backward_shortcut_layer_gpu(const layer l, network net);
#endif

#endif
