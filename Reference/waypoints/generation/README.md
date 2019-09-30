# waypoint generation
directory for creating waypoint graphs from real-images

# in progress
attempts to use contours to better represent the obstacles (the idea is that if they are smooth or mostly convex (L shapes obviously aren't convex but if they're perfectly smooth the idea applies), then one can simply place waypoints at each convex corner of the obstacles and that will result in nearly the same pathing but SIGNIFICANT computation decrease. some work was done but bugs exist and filling the contours that exist on boundaries seems troublesome.

waypoint pruning (as well as "line" drawing, where a "line" is used as the path between two waypoints). barely any work was done in this area. significant room for improvement by pruning both waypoints and lines that are not used. 

increasing the number of points generated as "safe" in transition regions should improve waypoint accuracy/precision
