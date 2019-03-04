# Animation in Radium

The Radium Engine provides a set of classes dedicated to animation,
which can be found in `Core/Animation`.

## Animation Basics

An `Animation` is a set of key poses, each associated to a key time.
When looking for the pose at a specific animation time, 
the 2 closest surrounding key poses are linearly interpolated to build the querried pose.

Though the `Animation` class only provides keyframing for poses, which are sets of transformation matrices,
the `KeyFrame` class provides a more generic interface, usable for any type of animated data.

## Character Animation

The Radium Engine also provides the basic classes for character animation.
Starting from a `Handle`, which represents a deformation metaphor, such as `Cage` or `Skeleton`, 
one can associate influence weights to each vertex of a mesh within a `WeightMatrix` (rows representing vertices, columns handles).
These influence weights, or skinning weights, are then used by a skinning algorithm to deform 
the associated mesh vertices w.r.t. the `Handle` deformation defined by the animation pose.

## The Character Animation Plugins

The Radium Engine provides one plugin specific to skeleton-based character animation 
and another one specific to skeleton-based character skinning.
This first one manages user interactions with the animation skeleton, enabling posing the character
 and playing the animations.
The second one is responsible for deforming the object's vertices according to the desired skinning method.

## Importing skeleton-based character animation data into Radium

In order to import animation related data into Radium, the default loader would be the `AssimpLoader`,
 which deals with several standard animation formats (fbx, collada, ...).
In case one need to develop his own loader because using a file format not managed by the Assimp library,
the Radium Engine provides a set of classes in `Core/Asset` defining data wrappers filled by loaders (see the specific documentation -- which does not exist for now).

Regarding character animation data, there are a few things to do in order to ensure correct import by the 
animation and skinning plugins:
 * For now, the animation plugin imports all the `AnimationData` contained within a `FileData`,
   thus causing issues when loading files with several animated objects, except if the loader generates one `FileData` per animated object.
 * The generated `FileData` must contain one `GeometryData`, one `HandleData` and as much `AnimationData` as needed per animated object.
   Warning: objects composed of several mesh parts not being handled properly by the AssimpLoader for now, the plugins have not been tested for this feature.
 * The `GeometryData` and its associated `HandleData` must have matching names.
 * The animation skeleton, i.e. the bones and hierarchy, must be loaded into the `HandleData`, as well as the skinning weights.
   For skeletons, each bone will be represented by a `HandleComponentData`, storing the bone transform and its per-vertex skinning weights,
   while the bone hierarchy will be defined by the `EdgeData`.
   Beware the reference pose will be the one computed from the loaded skeleton.
 * Animation keyframes, i.e. couples of time and pose (in local space), if any, must be loaded into an `AnimationData`.
   The whole animation must be stored bone-wise in `HandleAnimations`, whose name must match the bone's.

