
SceneInformation class:

Contains the Internal data structure related to the whole scene, 
which consists of:
- a type of scene and
- a set of objects present in the scene (a vector of "Object")

Object class:

Contains the Internal data structure related to a single object:
- the object name 
    (Note that this is also related to the object class ID, in case it will 
     be needed)
- The object geometry parameters, which are defined as
  - a bounding box (cuboid) defined by its 8 vertices in 3D
  - the centroid wihch is computed given the 8 vertices of the bounding box.

ApiConvertKTHDB class:

Is the Api to convert the annotation format of KTH annotations.
Receives in input a file xml with the annotation.
Parses the file.
Gives in output the IDS, i.e. an object of class "SceneInformation" with all
the objects added.
The Api converts the object parameters stored in the KTH annotations 
into the object parameters chosen for the IDS.

TestScene class:

Contains all processing for an XML annotation file to use as test scene 
in a cross validation strategy. 
Loading the file, convert to IDS, Feature extraction, testing the 
learned probability models, evaluating probability / similarity to the 
learned scene model.

