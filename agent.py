"""
agent template for the nav challenge

1. treat all the '?' as free paths with a higher weight (so A star will prefer known paths) until shown otherwise
2. give all the known space '.' a weight of 1 so a star can prefer it
3. by default give the robot 2 cells of room between the wall and the robot to account for the noise when collecting wall data and any potential drift that may occur
4  also include a path where you give the robot 1 cell of space this only accounts for the walls and make it more expensive than the 2 cell padded one, when I was testing and this was the default behavior it crashed alot so I made this a back up
5  in the absolute worst case dont give the robot any padding between wall and bot just incase there are no other routes left and make this one extremely expensive so it is not prefered
6  check your planned path -> if it is valid continue on that path -> if the path leads you to collision call A* again to calculate your new path
7  if there is sufficent space start speeding up but never go so fast that you cant stop within view distance (similar to 3 second rule for driving) and constantly adjust this speed every step
8  watch the program run out of time for cases 3, 6, 9, 11, 12


"""
import heapq
import math
import itertools
class Agent:
  def __init__(self, cfg:dict):
    """cfg keys: width_m, height_m, resolution, robot_radius, v_max, a_max, dt, sense_cells, goal_tol, goal (x, y)."""
    self.cfg = cfg
    self.known_map: dict[tuple[int,int], str] = {}
    res = cfg['resolution']
    self.grid_width = int(cfg['width_m']/res) #width of the array is in number of tiles
    self.grid_length = int(cfg['height_m']/res) #length of the array in number of tiles 
    self.goal_cell = (int(cfg['goal'][0] / res), int(cfg['goal'][1] / res))
    self.safety_radius = int(cfg['robot_radius'] / res) + 2 #+1 represents the sensor noise where walls can be 1 off between scans and a +1 more for good measure incase something I didnt immediately think of comes up
    self.current_path = []
    self.last_vx = 0.0
    self.last_vy = 0.0
    self.known_safe_cells = set() #will hold tuples of 2 ints representing what cells that were already checked 
    self.known_tight_cells = set()

  def step(self, pose:tuple[float, float], scan:tuple[int, int, list[str]]) -> tuple[float, float]:
    """
    called once per tick.

    pose: (x, y) metres from SLAM, ~2 cm gaussian noise.
    scan: (cx0, cy0, rows) -- a (2*sense_cells+1)^2 window of '#'/'.' around the robot. rows[j][i] is cell (cx0+i, cy0+j).
          everything in the window is observed, nothing outside it is.
          the window origin comes from the noisy pose, so walls can land one cell off between scans. 
    returns: (vx, vy) world-frame velocity command in m/s. sim clamps speed and acceleration.
    """
    self.merge_map(scan)  # fold this tick's observed cells into the persistent known_map

    res = self.cfg['resolution']
    current_cell = (int(pose[0] / res), int(pose[1] / res))  # convert pose to a cell index on my grid

    dist_to_goal_m = math.hypot(pose[0] - self.cfg['goal'][0], pose[1] - self.cfg['goal'][1])  # straight-line distance to goal, in metres
    if dist_to_goal_m <= self.cfg['goal_tol']:
        return (0.0, 0.0)  # already within tolerance of the goal, stop moving
    if self.current_path and current_cell in self.current_path:
        self.current_path = self.current_path[self.current_path.index(current_cell):]  #remove everything before where we currently are unless its an empty list
    else:
        self.current_path = []
    safe_cells = self.is_safe(self.current_path)  # how many leading cells of our CURRENT stored path are still known-clear and straight
    safe_dist_m = safe_cells * res  # convert that cell-count into meters
    stopping_dist_needed = int((self.last_vx**2 + self.last_vy**2) / (2 * self.cfg['a_max']) / res)
    # replan only when our current path is either too short, or doesn't start where we actually are, 5 was determined experimentally
    if (not self.path_valid(min(stopping_dist_needed, safe_cells))):
        self.current_path = self.getRoute(current_cell, self.goal_cell)  # run A* fresh from where we are now
        safe_cells = self.is_safe(self.current_path)  # recompute safety count against the brand-new path
        safe_dist_m = safe_cells * res
    if not self.current_path or len(self.current_path) < 2:
        return (0.0, 0.0)  # no route exists (or we're already at the only cell in it) — stay put rather than crash

    a_max = self.cfg['a_max']
    v_max = self.cfg['v_max']
    target_speed = min(v_max, math.sqrt(2 * a_max * safe_dist_m))  # fastest speed we could be going and still stop within the known-safe run, capped at v_max verived from v_f ^2 = v_i^2 - 2*a_max*dx

    next_cell = self.current_path[1]  # current_path[0] is roughly "here", so the next waypoint to steer toward is index 1
    dx = (next_cell[0] - current_cell[0]) * res  # x-distance to next waypoint, in metres
    dy = (next_cell[1] - current_cell[1]) * res  # y-distance to next waypoint, in metres
    step_dist = math.hypot(dx, dy)  # straight-line distance to that waypoint

    if step_dist == 0:
        return (0.0, 0.0)  # already sitting on the next waypoint, nothing to steer toward this tick

    vx = (dx / step_dist) * target_speed  # unit x-direction scaled to our target speed
    vy = (dy / step_dist) * target_speed  # unit y-direction scaled to our target speed

    self.last_vx, self.last_vy = vx, vy  # remember what we commanded, in case a future check needs "our own last velocity"
    return (vx, vy)

  #checks if the current path is valid -> do we have to recalculate 
  def path_valid(self, check_dist: int):
    if len(self.current_path) == 0:
       return False
    for i in range(0, check_dist, 1): #only checks if the cells are actually safe to go on if we are close to them bc the terrain can change and checking is expensive
        cell = self.current_path[i]
        if self.known_map.get(cell) == "#" or not self.cell_safety_check(cell, self.safety_radius - 2): #if one of the elements in the path becomes a wall or is in range of one
           return False
    for cell in self.current_path: #checks the entire path for newly discovered walls
       if(cell == "#"):
          return False
    return True

  
  def debug(self) -> dict:
    """
    optional, for `harness.py --viz` only.
    keys: blocked (cells), free (cells), path ([(x, y), ...]).
    """
    return {}
  #Returns how many leading cells of the path are known-free ('.') and continuing in a straight line.
  def is_safe(self, path: list[tuple[int,int]]) -> int:
    if len(path) < 2:
      return len(path)
    direction = self.add_tuples(path[1], tuple(-x for x in path[0]))
    counter = 0

    for i in range(0, len(path) - 1, 1):
        cell = path[i]
        if self.known_map.get(cell) != ".":
          return counter
        step_direction = self.add_tuples(path[i+1], tuple(-x for x in path[i]))
        if step_direction != direction:
            return counter
        counter += 1
    return counter

  #takes a scan and merges it to what is already known about the map (the map is given as []"...#.", "##..." etc])
  def merge_map(self, scan: tuple[int, int, list[str]]):
    x, y, rows = scan
    for j, row in enumerate(rows):
      for i, char in enumerate(row):
        cell = (x + i, y + j)
        old = self.known_map.get(cell)
        if char == "#" and old != "#":
            self.invalidate_safety_cache_near(cell, self.known_safe_cells)  #if a wall just appeared what was previously deemed safe may not be anymore
            self.invalidate_safety_cache_near(cell, self.known_tight_cells)
        self.known_map[cell] = char
    return

  #Remove cached safety verdicts for cells within safety_radius of a newly-discovered wall cell.
  def invalidate_safety_cache_near(self, wall_cell: tuple[int,int], cache: set[tuple[int,int]]):
    wx, wy = wall_cell
    r = self.safety_radius
    for dx in range(-r, r + 1, 1):
        for dy in range(-r, r + 1, 1):
            if dx**2 + dy**2 <= r**2: #if within a circular radius of r
                cache.discard((wx + dx, wy + dy)) 

  #estimate cost to get to the goal from a point using euclidian distance
  def heuristics(self, start, end) -> float:
    return math.sqrt((start[0] - end[0])**2 + (start[1] - end[1])**2)

  @staticmethod #make this a static method so I dont clutter the parameters with "self"
  def add_tuples(a: tuple[int,int], b: tuple[int,int]):
    return (a[0] + b[0], a[1] + b[1]) # add coords like you would with cartesian coordinates

  def in_bounds(self, node: tuple[int,int]):
    x, y = node
    if (x < self.grid_width and x >= 0) and (y < self.grid_length and y >= 0):
      return True;
    return False;

  #for a cell to be safe there should be no walls within a certain radius of it so it takes the position data of the cell and checks whether the robot will hit anything if it actually went to that cell 
  def cell_safety_check(self, cell: tuple[int,int], radius: int) -> bool:
    if radius <= self.safety_radius and cell in self.known_safe_cells:
       return True
    if radius <= self.safety_radius -1 and cell in self.known_tight_cells:
       return True
    x, y = cell
    r = radius  #the radius we must keep around the robot to ensure it does not crash (accounts for sensor noise)
    for dx in range(-r, r + 1, 1):
        for dy in range(-r, r + 1, 1):
            if dx**2 + dy**2 > r**2: #if its not a circular radius we dont have to check it
                continue
            check_cell = (x + dx, y + dy)
            if self.known_map.get(check_cell) == "#":
                return False
    if radius == self.safety_radius:
       self.known_safe_cells.add(cell)
    elif radius == self.safety_radius - 1:
       self.known_tight_cells.add(cell)
    return True

  def safety_tier(self, cell: tuple[int,int]) -> int:
    if self.cell_safety_check(cell, self.safety_radius):
        return 0
    if self.cell_safety_check(cell, self.safety_radius - 1):
        return 1
    if self.cell_safety_check(cell, self.safety_radius - 2):
        return 2
    return 3
  
  #gets all the neighbors of a node that are in bounds and the robot is able to be on without crashing into wall (checked via cell_safety_check)
  def get_neighbors(self, currentNode: dict) -> list[dict]:
    tight_penalty = 30 #I ran into an error where if I made the wall offset too big sometimes it would not find a route within 30ms and if I mad eit too small it would crash, this makes it so we penalize tight corners isntead of removing them entirely
    valid_neighbors = []
    directions = [(1,0), (0,1), (-1,0), (0,-1)]
    directionsDiag = [(1,1), (-1,1), (-1,-1), (1,-1)]
    for move in directions:
        neighbor = self.add_tuples(currentNode['position'], move)
        if not self.in_bounds(neighbor):
            continue
        scout = self.known_map.get(neighbor, "?")
        if scout == "#":
            continue
        base_cost = 1 if scout == "." else 2
        if not self.cell_safety_check(neighbor, self.safety_radius):
            if self.cell_safety_check(neighbor, self.safety_radius - 1):
                if self.safety_tier(currentNode['position']) >= 2:
                   continue
                cost = currentNode['cost'] + base_cost + tight_penalty
            elif self.cell_safety_check(neighbor, self.safety_radius - 2):
                if self.safety_tier(currentNode['position']) >= 2:
                   continue
                cost = currentNode['cost'] + base_cost + tight_penalty * 20  #absolute worst case if get neighbor cannot find anything has a very steep price because it does not account for noise at all
            else:
                continue
        else:
            cost = currentNode['cost'] + base_cost
        valid_neighbors.append(self.create_node(neighbor, cost, self.heuristics(neighbor, self.goal_cell), currentNode))
    for move in directionsDiag:
        neighbor = self.add_tuples(currentNode['position'], move)
        if not self.in_bounds(neighbor):
            continue
        scout = self.known_map.get(neighbor, "?")
        if scout == "#" or not self.cell_safety_check(neighbor, self.safety_radius):
            continue
        cost = currentNode['cost'] + (math.sqrt(2) if scout == "." else 2 * math.sqrt(2))
        x, y = move
        if self.known_map.get(self.add_tuples(currentNode['position'], (x, 0))) == "#" and self.known_map.get(self.add_tuples(currentNode['position'], (0, y))) == "#":
            continue
        valid_neighbors.append(self.create_node(neighbor, cost, self.heuristics(neighbor, self.goal_cell), currentNode))
    return valid_neighbors
  #creates a node where cost is the "cost" to get here and heur is the estimated cost to get to the goal parent is what node it came from (default none) this is to reconstruct the path later
  def create_node(self, position: tuple[int,int], cost: float, heur: float, parent: dict = None) -> dict:
    return {
      'position': position,
      'cost': cost,
      'heuristics': heur,
      'f': cost + heur, #f is the cost to get here + estimated cost to get to goal
      'parent': parent
    }

  def construct_route(self, node: dict) -> list[tuple[int,int]]:
    current_node = node
    path = []
    while(current_node is not None): #rebuilds path from a node to the root node (the one robot is currently standing on)
      path.append(current_node['position'])
      current_node = current_node['parent']
    path.reverse()
    return path

  #runs A* with the following weights: known clear path 1, unknown path 2, wall invalid path cannot use
  def getRoute(self, start: tuple[int,int], goal: tuple[int,int]) -> list[tuple[int,int]]:
    counter = itertools.count() #incase 2 nodes have the same 'f' use counter as the tie breaker (does not represent anything) so python dosnt try to compare 2 dicts and throw error
    start_node = self.create_node(start, 0, self.heuristics(start, goal))
    open_queue = []
    closed_nodes = set()
    best_cost = {start:  0.0} #tracks the best known cost to get to a node to prevent pushing worse nodes onto the heap
    heapq.heappush(open_queue, (start_node['f'], next(counter), start_node))
    while(len(open_queue) != 0):
      f, c, current_node = heapq.heappop(open_queue) #unpack the tuple to get the dict representing the current node
      closed_nodes.add(current_node['position']) #once a node has been popped the shortest path to it is guranteed to be found so close the node (meaning fix its value in place and not waste compute resources processing it)
      if current_node['position'] == goal:
        return self.construct_route(current_node)
      for neighbor in self.get_neighbors(current_node):
        bestRoute = False
        if neighbor['cost'] < best_cost.get(neighbor['position'], float('inf')):
           best_cost[neighbor['position']] = neighbor['cost']
           bestRoute = True
        if neighbor['position'] in closed_nodes or not bestRoute:
          continue
        heapq.heappush(open_queue, (neighbor['f'], next(counter), neighbor))
    return [] #no path found