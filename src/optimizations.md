# Optimizations
This file will include a list of potential speed optimizations which have
not yet been implemented.

## Unused sensors
We could detect whether sensors have no active neural network connections and disable them altogether. This should provide a significant speed improvement for high-computation sensors such as a camera.

## Combining collision objects
Currently, all links consist of single collision objects, connected through a revolute joint with zero limits (which appears to be the only way to fix them). We could instead create single links with multiple collision objects, offloading the physics engine. The problem here is the inertia matrix, which we cannot auto generate for nontrivial shapes. Within complex components however, we could estimate the inertia of fixed collision objects using Meshlab for those parts of
the component which are fixed.
