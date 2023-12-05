# Final Project Writeup

## FP.1 Match 3D Objects

The function `matchingBoundingBoxes` in the file `camFusion_Student.cpp` is
implemented as described in the rubric.

## FP.2 Compute Lidar-based TTC

The function `computeTTCLidar` in the file `camFusion_Student.cpp` implements
the TTC computation based purely on lidar data as described in the exercises.
To prevent outlier points of leading to wrong results, median and standard
deviation of the `X` coordinate are computed and only points within a given
distance from the median are considered for the computation.

## FP.3 Associate Keypoint Correspondence with Bounding

The function `clusterKptMatchesWithROI` in the file `camFusion_Student.cpp`
computes the keypoint matches for a given box as described in the rubric. To
handle outliers median and standard deviation of the euclidian distance between
two points are computed and only matches with with a distance within a given
multiple of the standard deviation from the median are considered. The tests
shows that this factor needs to be fairly small for good results. Currently I
set it to `0.5`.

## FP.4 Performance Evaluation 1

The following is the output of the TTC system, where the keypoint/descriptor algorithm pair is FAST/BRIEF.

TTC Lidar : 12.972159 s, TTC Camera : 11.750239 s
TTC Lidar : 12.972159 s, TTC Camera : 11.694809 s
TTC Lidar : 13.246532 s, TTC Camera : 13.443180 s
> TTC Lidar : 29.057666 s, TTC Camera : 13.265749 s
TTC Lidar : 9.343759 s, TTC Camera : 12.598070 s
> TTC Lidar : 18.031756 s, TTC Camera : 12.584364 s
TTC Lidar : 14.987674 s, TTC Camera : 11.743797 s
TTC Lidar : 10.099996 s, TTC Camera : 11.360505 s
TTC Lidar : 10.967763 s, TTC Camera : 9.929307 s
TTC Lidar : 8.094218 s, TTC Camera : 11.406610 s
TTC Lidar : 8.813924 s, TTC Camera : 10.713727 s
TTC Lidar : 10.292551 s, TTC Camera : 10.484987 s
TTC Lidar : 8.309779 s, TTC Camera : 11.081470 s

The two lines marked with `>` show a significant difference of the TTC computed
purely with lidar to the TTC computed with the camera. Looking at the topview of
the lidar points of the preceding vehicle indicates that the computed TTC of the
lidar is most likely of for the following reasons:
  
  > The distance to the preceding vehicle strictly decreases.
  > The relative speed of the two vehicle doesn't seem to abruptly jump by more than a factor of two. 
  > There seem to be outliers visible at the very bottom of the bounding boxes that could be responsible for this result.

By allowing only points within a smaller distance to the median (1 * standard deviation), the results improve significantly:

TTC Lidar : 13.945691 s, TTC Camera : 11.750239 s
TTC Lidar : 13.945691 s, TTC Camera : 11.694809 s
TTC Lidar : 16.463854 s, TTC Camera : 13.443180 s
TTC Lidar : 12.459043 s, TTC Camera : 13.265749 s
TTC Lidar : 13.233407 s, TTC Camera : 12.598070 s
TTC Lidar : 13.282201 s, TTC Camera : 12.584364 s
TTC Lidar : 11.343067 s, TTC Camera : 11.743797 s
TTC Lidar : 9.863465 s, TTC Camera : 11.360505 s
TTC Lidar : 11.541973 s, TTC Camera : 9.929307 s
TTC Lidar : 8.419037 s, TTC Camera : 11.406610 s
TTC Lidar : 9.587677 s, TTC Camera : 10.713727 s
TTC Lidar : 9.620830 s, TTC Camera : 10.484987 s
TTC Lidar : 7.683126 s, TTC Camera : 11.081470 s

The reason for the outliers in the point cloud could be multiple. The source of the faulty lidar point could for example be dust/insect in the air.

# FP.6 Performance Evaluation 2