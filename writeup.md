## Project: 3D Motion Planning

---

# Required Steps for a Passing Submission:
1. Load the 2.5D map in the colliders.csv file describing the environment.
2. Discretize the environment into a grid or graph representation.
3. Define the start and goal locations.
4. Perform a search using A* or other search algorithm.
5. Use a collinearity test or ray tracing method (like Bresenham) to remove unnecessary waypoints.
6. Return waypoints in local ECEF coordinates (format for `self.all_waypoints` is [N, E, altitude, heading], where the drone’s start location corresponds to [0, 0, 0, 0].
7. Write it up.
8. Congratulations!  Your Done!

## [Rubric](https://review.udacity.com/#!/rubrics/1534/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You are reading it! Below I describe how I addressed each rubric point and where in my code each point is handled.

### Explain the Starter Code

#### 1. Test that `motion_planning.py` is a modified version of `backyard_flyer_solution.py` for simple path planning. Verify that both scripts work. Then, compare them side by side and describe in words how each of the modifications implemented in `motion_planning.py` is functioning.

The biggest differences between the two scripts is that `motion_planning.py` has the state call PLANNING. In which state waypoints were planned, calculated and sent to the simulator.

### Implementing Your Path Planning Algorithm

#### 1. In the starter code, we assume that the home position is where the drone first initializes, but in reality you need to be able to start planning from anywhere. Modify your code to read the global home location from the first line of the `colliders.csv` file and set that position as global home (`self.set_home_position()`)

The following code snippets are added for read the first line of `colliders.csv`, and assign longitude and latitude for setting home position.
```
# TODO: read lat0, lon0 from colliders into floating point values
import csv
lat0, lon0 = 0, 0
with open("colliders.csv") as f:
    row = next(csv.reader(f))
    lat0 = float(row[0].split( )[1])
lon0 = float(row[1].split( )[1])

# TODO: set home position to (lon0, lat0, 0)
self.set_home_position(lon0, lat0, 0)
```

#### 2. In the starter code, we assume the drone takes off from map center, but you will need to be able to takeoff from anywhere. Retrieve your current position in geodetic coordinates from `self._latitude`, `self._longitude` and `self._altitude`. Then use the utility function `global_to_local()` to convert to local position (using `self.global_home` as well, which you just set)

- local posiiton can be retrieve by
```
local_position = global_to_local(self.global_position, self.global_home)
```
It is actually equal to `self.local_position`.
According to [drone.py](https://github.com/udacity/udacidrone/blob/master/udacidrone/drone.py), the self.local_position is read-only property, need not set explicitly.

- global position can be retrieve by 
```
global_position = (self._longitude, self._latitude, self._altitude)
```

It is actually equal to `self.global_position`, it can also be retrieve by
```
global_position = local_to_global(self.local_postion, self.global_home)
```


#### 3. In the starter code, the `start` point for planning is hardcoded as map center. Change this to be your current local position.

The center of the map is locate at (316, 445), I have added 2 command line argument for assigning global stating poing
```
  parser.add_argument('--start_lon', type=int, default=316, help='given start longitude')
  parser.add_argument('--start_lat', type=int, default=445, help='given start latitude')
```

#### 4. In the starter code, the goal position is hardcoded as some location 10 m north and 10 m east of map center. Modify this to be set as some arbitrary position on the grid given any geodetic coordinates (latitude, longitude)

I have added another two command line arguments to specify the goal position
```
  parser.add_argument('--goal_lon', type=int, default=326, help='given end longitude')
  parser.add_argument('--goal_lat', type=int, default=455, help='given end latitude')
```
 
#### 5. Write your search algorithm. Minimum requirement here is to add diagonal motions to the A* implementation provided, and assign them a cost of sqrt(2). However, you are encouraged to get creative and try other methods from the lessons and beyond!

For sqrt(2) cost function, I have declare as such
```
  import math
  sqrt2 = math.sqrt(2)
```

and modifiy the Action class as 
```
 WEST = (0, -1, 1)
 EAST = (0, 1, 1)
 NORTH = (-1, 0, 1)
 SOUTH = (1, 0, 1)
 NORTHWEST = (-1, -1, sqrt2)
 SOUTHWEST = (1, -1, sqrt2)
 NORTHEAST = (-1, 1, sqrt2)
 SOUTHEAST = (1, 1, sqrt2)
```

Another part of A* algorithm need to be modify is the `valid_actions` function
```
  if x - 1 < 0 or grid[x - 1, y] == 1:
      valid_actions.remove(Action.NORTH)
  if x + 1 > n or grid[x + 1, y] == 1:
      valid_actions.remove(Action.SOUTH)
  if y - 1 < 0 or grid[x, y - 1] == 1:
      valid_actions.remove(Action.WEST)
  if y + 1 > m or grid[x, y + 1] == 1:
      valid_actions.remove(Action.EAST)
  if (x - 1 < 0 and y - 1 < 0) or grid[x - 1, y - 1] == 1:
      valid_actions.remove(Action.NORTHWEST)
  if (x - 1 < 0 and y + 1 > m) or grid[x - 1, y + 1] == 1:
      valid_actions.remove(Action.NORTHEAST)
  if (x + 1 > n and y - 1 < 0) or grid[x + 1, y - 1] == 1:
      valid_actions.remove(Action.SOUTHWEST)
  if (x + 1 > n and y + 1 > m) or grid[x + 1, y + 1] == 1:
      valid_actions.remove(Action.SOUTHEAST)
```
 
#### 6. Cull waypoints from the path you determine using search.

I have calculated the area of the triangle defineded by three consecutive path points, if it is smaller than a threshold, then we say these three points are collinear, and we can remove the middle points.

```
  path, _ = a_star(grid, heuristic, grid_start, grid_goal)
  print("path: {0}".format(path))

  def prune_path(paths):
      def is_collinear(p0, p1, p2):
          area = p0[0] * (p1[1] - p2[1]) + p1[0] * (p2[1] - p0[1]) + p2[0] * (p0[1] - p1[1])
          if area < 0.000001:
              print("remove middle pt:{0}".format(p1))
              return True
          else:
              return False

      prunes = paths.copy()
      for i in range(len(paths) - 2):
          if is_collinear(paths[i], paths[i+1], paths[i+2]):
             prunes.remove(paths[i+1]) 

      return prunes

  prunes = prune_path(path)
  print("prunes: {0}".format(prunes))
```

### Execute the flight
#### 1. This is simply a check on whether it all worked. Send the waypoints and the autopilot should fly you from start to goal!

if (--start_lon, --start_lat, --goal_lon, --goal_lat) are not set, we will execute the original plan, fly from (316, 445) to (326, 455).
```
  Local Start and Goal:  (316, 445) (326, 455)
  Found a path.
  path: [(316, 445), (317, 446), (318, 447), (319, 448), (320, 449), (321, 450), (322, 451), (323, 452), (324, 453), (325, 454), (326, 455)]
  remove middle pt:(317, 446)
  remove middle pt:(318, 447)
  remove middle pt:(319, 448)
  remove middle pt:(320, 449)
  remove middle pt:(321, 450)
  remove middle pt:(322, 451)
  remove middle pt:(323, 452)
  remove middle pt:(324, 453)
  remove middle pt:(325, 454)
  prunes: [(316, 445), (326, 455)]
  waypoints: [[0, 0, 5, 0], [10, 10, 5, 0]]
```

else, set either of the customized start or goal point, said we want to fly from (320, 450) to (333, 488)
```
  Local Start and Goal:  (320, 450) (333, 488)
  Found a path.
  path: [(320, 450), (320, 451), (320, 452), (320, 453), (320, 454), (320, 455), (320, 456), (320, 457), (320, 458), (320, 459), (320, 460), (321, 461), (321, 462), (322, 463), (322, 464), (323, 465), (323, 466), (323, 467), (324, 468), (324, 469), (325, 470), (325, 471), (326, 472), (326, 473), (327, 474), (327, 475), (328, 476), (328, 477), (328, 478), (329, 479), (329, 480), (330, 481), (330, 482), (331, 483), (331, 484), (332, 485), (332, 486), (332, 487), (333, 488)]
  remove middle pt:(320, 451)
  remove middle pt:(320, 452)
  remove middle pt:(320, 453)
  remove middle pt:(320, 454)
  remove middle pt:(320, 455)
  remove middle pt:(320, 456)
  remove middle pt:(320, 457)
  remove middle pt:(320, 458)
  remove middle pt:(320, 459)
  remove middle pt:(320, 460)
  remove middle pt:(321, 462)
  remove middle pt:(322, 464)
  remove middle pt:(323, 466)
  remove middle pt:(323, 467)
  remove middle pt:(324, 469)
  remove middle pt:(325, 471)
  remove middle pt:(326, 473)
  remove middle pt:(327, 475)
  remove middle pt:(328, 477)
  remove middle pt:(328, 478)
  remove middle pt:(329, 480)
  remove middle pt:(330, 482)
  remove middle pt:(331, 484)
  remove middle pt:(332, 486)
  remove middle pt:(332, 487)
  prunes: [(320, 450), (321, 461), (322, 463), (323, 465), (324, 468), (325, 470), (326, 472), (327, 474), (328, 476), (329, 479), (330, 481), (331, 483), (332, 485), (333, 488)]
  waypoints: [[4, 5, 5, 0], [5, 16, 5, 0], [6, 18, 5, 0], [7, 20, 5, 0], [8, 23, 5, 0], [9, 25, 5, 0], [10, 27, 5, 0], [11, 29, 5, 0], [12, 31, 5, 0], [13, 34, 5, 0], [14, 36, 5, 0], [15, 38, 5, 0], [16, 40, 5, 0], [17, 43, 5, 0]]
```

![](https://i.imgur.com/gMIVCWj.png)
