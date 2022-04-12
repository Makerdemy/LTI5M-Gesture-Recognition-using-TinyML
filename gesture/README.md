# How to integrate Neuton into your firmware project 

## Include header file

Copy all files from this archive to your project and include header file:
``` C
#include "neuton.h"
```


## Set input values

Make a float array with input values in same order as you train a model.

``` C
// Accelerometer values
float inputs[] = { 
    acc_x,  // X axis
    acc_y,  // Y axis
    acc_z   // Z axis
};
```

Pass this array to `neuton_model_set_inputs` function. Function will return `0` when model is ready for prediction.

``` C
if (neuton_model_set_inputs(inputs) == 0)
{
    ...
}
```

If preprocessing is used you should call `neuton_model_set_inputs` on every acquired sample.
On each call values copied to window buffer and `0` indicates that window buffer is ready.


##	Make prediction

When window buffer is ready you should call `neuton_model_run_inference`.
``` C
if (neuton_model_set_inputs(inputs) == 0)
{
    uint16_t predictedClass;
    float* probabilities;
    
    if (neuton_model_run_inference(&predictedClass, &probabilities) == 0)
    {
        ...
    }
}
```

Here function accept two arguments:
* predictedClass - index of class with maximal probability (will be set for classification tasks);
* probabilities - array of probabilities of each class (dimension of array can be read using `neuton_model_outputs_count` function).

Function will return `0` on successfull prediction.
