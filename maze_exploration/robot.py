# coding: utf8
import json
import random
from sys import stderr

import numpy as np


class Robot(object):
    class Cell(object):
        """A cell of the robot's internal maze memory."""

        def __init__(self):
            # Stores the direction in which the path lies that has led to a junction.
            self.previous = ''  # type : str
            # Indicates if the cell is unvisited, visited or double visited.
            self.value = 0  # type: int

    def __init__(self, maze_dim):
        """
        Set up attributes that the agent will use to learn and navigate the
        maze. Some initial attributes are
        provided based on common information, including the size of the maze
        the robot is placed in.
        """

        # Initialize coordinate values
        self.orig_x, self.orig_y = 0, 0
        self.x, self.y = 0, 0
        self.last_x, self.last_y = 0, 0

        # Maximum allowed movement units per turn
        self.max_movement = 3

        # These values will be updated with every call of next_move() and will
        # be returned to the caller.
        self.rotation = 0
        self.movement = 0

        # Members to store the external inputs in
        self.maze_dim = maze_dim
        self.sensors = []

        # Flag that indicates the first step of exploration
        self.is_beginning = True
        # Flag that indicates if the robot is in process of reversing,
        # e. g. rotating by 180 degrees to face the opposite direction.
        self.is_reversing = False

        # Current heading of the robot expressed as global direction
        self.heading = 'up'

        # The robot's current mode of operation.
        # This decides what the robot does when next_move() is called.
        self.mode = "explore"

        # Text file in which the travelled path will be logged.
        self.log_filename = 'path.json'
        # This clears an existing log file.
        open(self.log_filename, 'w').close()

        # Corresponding new headings after rotating
        self.dict_rotation = {'up': ['left', 'right'],
                              'right': ['up', 'down'],
                              'down': ['right', 'left'],
                              'left': ['down', 'up']}

        # Vectors for different directions
        self.direction_to_vec = {'up': [0, 1],
                                 'right': [1, 0],
                                 'down': [0, -1],
                                 'left': [-1, 0]}

        # Opposite directions
        self.opposite = {'up': 'down',
                         'right': 'left',
                         'down': 'up',
                         'left': 'right'}

        # Dictionary for backtracking, connects robot headings with rotations needed to face a global direction
        self.direction_to_rotation = {
        heading: {directions[0]: -90, directions[1]: 90}
        for heading, directions in self.dict_rotation.items()}

        # Rotation matrices
        self.rot_matrices = {'left': np.array([(0, 1), (-1, 0)]),
                             'forward': np.array([(1, 0), (0, 1)]),
                             'right': np.array([(0, -1), (1, 0)])}

        # Numbers assigned to open walls in cells.
        # See comment at maze_map for further description.
        self.wall_values = {'up': 1,
                            'right': 2,
                            'down': 4,
                            'left': 8}

        # Internal maze cell map and binary dictionary for the robot.
        # Each number represents a four-bit number that has a bit value of 0 if an edge is closed (walled) and
        # 1 if an edge is open (no wall); the 1s register corresponds with the upwards-facing side, the 2s register
        # the right side, the 4s register the bottom side, and the 8s register the left side. For example,
        # the number 10 means that a square is open on the left and right,
        # with walls on top and bottom (0*1 + 1*2 + 0*4 + 1*8 = 10).
        # The index origin (0, 0) is at the bottom left. The first index is the offset right from the origin,
        # the second index is the offset up from the origin.
        self.maze_map = [[0 for _ in range(maze_dim)] for _ in range(maze_dim)]

        # Internal path map for the robot to keep track of the already visited parts of the maze.
        self.path_map = [[self.Cell() for _ in range(maze_dim)] for _ in
                         range(maze_dim)]

        # Policy grid which will be created after fully exploring the maze and
        # performing a search algorithm.
        self.policy_grid = [['' for _ in range(self.maze_dim)] for _ in
                            range(self.maze_dim)]

        # Possible cell values
        # These are used to mark and log the robot's path.
        self.UNVISITED = 0
        self.VISITED = 1
        self.DOUBLE_VISITED = 2
        self.SHORTEST = 3

    def next_move(self, sensors):
        """
        Determines the next move the robot should make,
        based on the input from the sensors after its previous move. Sensor
        inputs are a list of three distances from the robot's left, front, and
        right-facing sensors, in that order.

        Outputs are a tuple of two values. The first value indicates
        robot rotation (if any), as a number: 0 for no rotation, +90 for a
        90-degree rotation clockwise, and -90 for a 90-degree rotation
        counterclockwise. Other values will result in no rotation. The second
        value indicates robot movement, and the robot will attempt to move the
        number of indicated squares: a positive number indicates forwards
        movement, while a negative number indicates backwards movement. The
        robot may move a maximum of three units per turn. Any excess movement
        is ignored.

        If the robot wants to end a run (e.g. during the first training run in
        the maze) then returning the tuple ('Reset', 'Reset') will indicate to
        the tester to end the run and return the robot to the start.
        """

        self.rotation = 0
        self.movement = 0
        self.sensors = sensors

        if self.mode == "explore":
            # Explore and map the complete maze iteratively on
            # every call of next_move().
            self.explore()
            # Log the exploring path to the log file
            self.log_location()
            # Perform rotations and movements determined by the exploring function
            self.rotate()
            self.move(self.movement)

        if self.mode == "search":
            # At this point, the robot has fully explored the maze
            # and completely rebuilt an internal representation of
            # the maze map in memory.
            # A searching algorithm can now be used with the internal
            # map to find the shortest path.
            # This is completed in just one call.
            self.find_shortest_path()
            self.switch_to_race()

        elif self.mode == "race":
            # Race to the goal room on the shortest path through the maze.
            # The robot still moves iteratively on every call of next_move().
            self.race_to_goal()
            # Perform rotations and movements determined by the racing function
            self.rotate()
            self.move(self.movement)
            # This marks the racing path and logs it to the logfile
            self.mark_path(self.SHORTEST)
            self.log_location()

        return self.rotation, self.movement

    def finished_exploration(self):
        """Returns true when the robot is back at the origin."""
        return [self.x, self.y] == [self.orig_x, self.orig_y]

    def reverse(self):
        """Start reversing the robot to perform a 180 degree rotation."""
        self.is_reversing = True
        self.rotation = 90

    def check_open_directions(self):
        """Check which directions are not blocked return them."""
        open_directions = []
        if self.sensors[0] > 0:
            open_directions.append('left')
        if self.sensors[1] > 0:
            open_directions.append('forward')
        if self.sensors[2] > 0:
            open_directions.append('right')
        return open_directions

    def get_paths(self, open_directions, value):
        """Returns the directions where there are paths with the specified value.
            Only checks in the provided directions."""
        # Direction vector pointing in the direction of the robot's heading.
        movement_vec = np.array(self.direction_to_vec[self.heading])

        paths = []
        for direction in open_directions:
            # Rotate robot's direction vector to given direction
            dir_vec = np.dot(movement_vec, self.rot_matrices[direction])
            # Get the next location in that direction
            next_loc = (self.x + dir_vec[0], self.y + dir_vec[1])
            # Check if the path at that location is not the robot's last location
            # and if it has the specified value.
            if (not (next_loc[0], next_loc[1]) == (
            self.last_x, self.last_y) and
                    self.path_is(value, next_loc[0], next_loc[1])):
                paths.append(direction)

        return paths

    def mark_path(self, new_value=None):
        """Mark a traveled path by increasing its value in the path map."""
        if new_value is None:
            self.path_map[self.x][self.y].value += 1
        else:
            self.path_map[self.x][self.y].value = new_value

    def path_is(self, value, x=None, y=None):
        """
        Returns true if the path at the given position has the specified value.
            If no position parameters are given, checks at the robot's current position."""
        if x is None:
            x = self.x
        if y is None:
            y = self.y

        return self.path_map[x][y].value == value

    def follow_path(self, direction):
        """Follow path in the given direction."""
        if direction == "left":
            self.rotation = -90
            self.movement = 1
        elif direction == "forward":
            self.rotation = 0
            self.movement = 1
        elif direction == "right":
            self.rotation = 90
            self.movement = 1
        else:
            print(
                "Can't follow path, chosen direction " + direction + "is invalid.",
                file=stderr)
            self.movement = 0
            self.movement = 0

    def continue_backtracking(self):
        """Continue backtracking through a junction."""
        self.movement = 1
        # Get direction in which the previous cell lies, to which we wish to backtrack to.
        direction = self.path_map[self.x][self.y].previous
        # Translate that direction into a possibly needed rotation of the robot,
        # considering the current heading.
        # This sets the rotation to -90, 0 or 90 to face the given direction.
        self.rotation = self.direction_to_rotation[self.heading].get(direction,
                                                                     0)

    def rotate(self):
        """Rotate by a given angle."""
        if type(self.rotation) is str:
            # A 'reset' was set
            return

        # Update robot heading to reflect the current rotation
        if self.rotation == -90:
            self.heading = self.dict_rotation[self.heading][0]
        elif self.rotation == 90:
            self.heading = self.dict_rotation[self.heading][1]
        elif self.rotation == 0:
            pass

    def movement_allowed(self):
        """Check if the path in the desired direction is blocked."""
        if self.rotation == -90:
            return self.sensors[0] > 0
        elif self.rotation == 90:
            return self.sensors[2] > 0
        elif self.rotation == 0:
            return self.sensors[1] > 0
        else:
            return False

    def move(self, distance):
        """Move distance in direction of current heading."""
        if type(distance) is str:
            # A 'reset' was set.
            return

        self.last_x, self.last_y = self.x, self.y
        while distance > 0:
            if self.movement_allowed():
                if self.heading == "up":
                    self.y += 1
                elif self.heading == "down":
                    self.y -= 1
                elif self.heading == "right":
                    self.x += 1
                elif self.heading == "left":
                    self.x -= 1

                distance -= 1

            else:
                print("Movement blocked.", file=stderr)
                distance = 0

    def end_exploration(self):
        """Stop the robot's exploration mode and reset the run."""
        print("Robot has reached the origin again. Finishing exploration.")
        # Reset some localization-specific values
        self.heading = "up"
        self.x, self.y = self.orig_x, self.orig_y

        self.mode = "search"

        # Set the reset signals
        self.movement = "Reset"
        self.rotation = "Reset"

    def log_location(self):
        """Stores current coordinates in a log file."""
        with open(self.log_filename, 'a') as file_object:
            # Data format: [Robot-X, Robot-Y, Current Cell Value, Robot-Heading]
            out_data = [self.x, self.y, self.path_map[self.x][self.y].value,
                        self.heading]
            json.dump(out_data, file_object)
            file_object.write('\n')

    def update_map(self, open_directions):
        """Update the robot's internal map using the unblocked (open)
            directions detected by the current sensor readings."""
        # Get the unit vector which points in the direction of the robot's heading
        movement_vec = np.array(self.direction_to_vec[self.heading])

        # First, translate the detected openings into global directions
        for direction in open_directions:
            global_dir = None
            if direction == 'left':
                global_dir = self.dict_rotation[self.heading][0]
            elif direction == 'right':
                global_dir = self.dict_rotation[self.heading][1]
            elif direction == 'forward':
                global_dir = self.heading

            # Get the corresponding wall value for an wall opening in the given direction
            wall_value = self.wall_values[global_dir]
            # Update the current map cell with the new wall value
            self.maze_map[self.x][self.y] |= wall_value
            # Rotate robot's direction vector to given direction
            dir_vec = np.dot(movement_vec, self.rot_matrices[direction])
            # Get the wall opening value for the next cell
            wall_value = self.wall_values[self.opposite[global_dir]]
            # Update the next map cell with the opening that can be seen from this cell.
            # If this step is omitted, the robot never maps entries to deadends.
            self.maze_map[self.x + dir_vec[0]][
                self.y + dir_vec[1]] |= wall_value

    def explore(self):
        """Explore a maze using Trémaux' algorithm."""

        if self.is_beginning:
            # This prevents the robot from immediately cancelling exploration
            self.is_beginning = False
        elif self.finished_exploration():
            # When back at the start, end the exploration
            self.end_exploration()
            self.mark_path()
            return

        # When in reversing mode, just finish the rotation and move forward
        if self.is_reversing:
            self.rotation = 90
            self.movement = 1
            self.is_reversing = False
            return

        # Translate sensor readings into unblocked directions
        open_directions = self.check_open_directions()
        # Update the internal mapping of the maze
        self.update_map(open_directions)

        # --------------------------------------
        # Trémaux' algorithm
        # --------------------------------------

        if len(open_directions) == 0:
            # Robot is at a deadend
            # Start backtracking
            self.reverse()
            self.mark_path(self.DOUBLE_VISITED)

        elif len(open_directions) == 1:
            # Robot is on a path to the next junction
            self.follow_path(open_directions.pop())
            self.mark_path()

        elif len(open_directions) > 1:
            # Robot is at a junction
            if self.path_is(self.UNVISITED):
                # Robot is at a new junction
                # Store the direction to the path which has led to this junction, used for backtracking.
                self.path_map[self.x][self.y].previous = self.opposite[
                    self.heading]
                # Get the adjacent paths that are still unvisited.
                unvisited_paths = self.get_paths(open_directions, self.UNVISITED)
                if len(unvisited_paths) > 0:
                    # There are still unvisited paths branching from this junction, follow a random one.
                    self.follow_path(random.choice(unvisited_paths))
                    # Mark this junction for the first time
                    self.mark_path()
                else:
                    # This junction has no unvisited paths left,
                    # treat it like a dead end.
                    self.reverse()
                    self.mark_path(self.DOUBLE_VISITED)

            elif self.path_is(self.VISITED):
                # Robot is at an old junction
                if self.path_is(self.DOUBLE_VISITED, self.last_x, self.last_y):
                    # Robot is backtracking
                    unvisited_paths = self.get_paths(open_directions, self.UNVISITED)
                    if len(unvisited_paths) > 0:
                        # There is still at least one unvisited path branching from this junction
                        # Follow a random one of them.
                        self.follow_path(random.choice(unvisited_paths))
                    else:
                        # There are no unvisited paths branching from this junction.
                        # Continue backtracking.
                        self.continue_backtracking()
                        # Mark this junction for the second time
                        self.mark_path()
                else:
                    # Robot has stepped into an old junction while not backtracking,
                    # so treat it like a deadend.
                    self.reverse()
            else:
                print("The junction at position " + str((self.x, self.y)) +
                      " has no valid value.", file=stderr)

    def find_shortest_path(self):
        """Find the shortest path to the goal using breadth-first search and
            create an action policy from it."""
        init = [self.orig_x, self.orig_y]

        # The center cells which make up the goal room.
        goal_room = [[self.maze_dim / 2, self.maze_dim / 2],
                     [self.maze_dim / 2 - 1, self.maze_dim / 2],
                     [self.maze_dim / 2, self.maze_dim / 2 - 1],
                     [self.maze_dim / 2 - 1, self.maze_dim / 2 - 1]]

        # This could be used to change the movement costs.
        cost = 1

        # Connects movement delta vectors and
        # their corresponding directional actions.
        delta_to_action = {(0, 1): 'up',
                           (1, 0): 'right',
                           (0, -1): 'down',
                           (-1, 0): 'left'}

        # This grid holds the action delta at every position of the maze.
        delta_grid = [[(0, 0) for _ in range(self.maze_dim)] for _ in
                      range(self.maze_dim)]

        # Initialize some values and lists for the search algorithm
        g = 0
        open_cells = [[g, init[0], init[1]]]
        visited = [init]
        end = []

        # Search through the maze with Dijkstra.
        while True:

            if not open_cells:
                break

            open_cells.sort()
            # Get the cell from the open list with the lowest cost-value (G-Value).
            g, x, y = open_cells.pop(0)

            if [x, y] in goal_room:
                # Stop when entering the goal room.
                end = [x, y]
                break

            # Check the current position in the maze map for wall openings.
            # For every wall opening, the corresponding directional delta vector is added
            # to the deltas list. This essentially creates a list of deltas to cells connected to
            # the current cell in the map.
            deltas = []
            for direction, value in self.wall_values.items():
                if self.maze_map[x][y] & value != 0:
                    deltas.append(self.direction_to_vec[direction])

            # Now, loop through all the connected cells
            for dx, dy in deltas:
                # Use delta to calculate the coords of the next cell (nx, ny)
                nx, ny = x + dx, y + dy
                if [nx, ny] not in visited:
                    # The next cell is not yet visited
                    open_cells.append([g + cost, nx, ny])
                    visited.append([nx, ny])
                    # Save the action delta vector needed to get to this next cell (nx, ny)
                    delta_grid[nx][ny] = (dx, dy)

        # Create policy path by travelling from end to start
        x, y = end
        self.policy_grid[x][y] = '*'
        while [x, y] != init:
            # Apply the previously saved action deltas backwards.
            nx = x - delta_grid[x][y][0]
            ny = y - delta_grid[x][y][1]
            # Save the action string to the policy grid.
            self.policy_grid[nx][ny] = delta_to_action[delta_grid[x][y]]
            # Continue with the next position
            x, y = nx, ny

            # This piece of code displays the shortest path by printing out the policy grid.
            # print("Path:")
            # for y in list(reversed(range(self.maze_dim))):
            #     print("[", end="")
            #     for x in range(self.maze_dim):
            #         print("{:>7}".format(self.policy_grid[x][y]), end="")
            #     print("]")

    def switch_to_race(self):
        """Switches to racing mode and performs one-time actions for the switch."""
        # This is needed to mark the beginning of the race path.
        self.mark_path(self.SHORTEST)
        self.log_location()
        self.mode = "race"

    def race_to_goal(self):
        """Travel the shortest path to the goal room."""

        # First, collect up to three actions in a line if they are the same
        actions = []
        x, y = self.x, self.y
        abort = False
        while len(actions) < self.max_movement and not abort:
            current_action = self.policy_grid[x][y]

            if not current_action:
                abort = True
            else:
                actions.append(current_action)
                dx, dy = self.direction_to_vec[current_action]
                nx, ny = x + dx, y + dy
                # Check if the next cell (nx, ny) has the same action.
                if (0 <= nx < self.maze_dim and
                        0 <= ny < self.maze_dim and
                        self.policy_grid[nx][ny] == current_action):
                    x = nx
                    y = ny
                else:
                    abort = True

        # Secondly, set rotation and movement according to the collected actions
        self.rotation = self.direction_to_rotation[self.heading].get(
            actions[0], 0)
        self.movement = len(actions)
