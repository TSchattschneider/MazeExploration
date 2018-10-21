import json
import sys
import turtle

from maze import Maze


def draw_path(filepath, pen, origin, sq_size):
    """"Reads a path from a file and draws it on the maze."""
    first = True
    with open(filepath, 'r') as file_object:
        for line in file_object:
            x, y, visited, heading = json.loads(line)

            if visited == 0:
                color = 'gray'
            elif visited == 1:
                color = 'green yellow'
            elif visited == 2:
                color = 'gray'
            elif visited == 3:
                color = 'red'
            else:
                color = 'black'

            if first:
                pen.hideturtle()
                pen.pensize(int(sq_size / 2))
                pen.pencolor(color)
                pen.setheading(90)
                pen.goto(origin + sq_size / 2, origin + sq_size / 2)
                pen.showturtle()
                first = False
            else:
                draw_line(x, y, color, heading, pen, origin, sq_size)


def draw_line(x, y, color, heading, pen, origin, sq_size):
    """Draws a continuous line on the path."""
    center_x = origin + sq_size * x + sq_size / 2
    center_y = origin + sq_size * y + sq_size / 2
    heading_dict = {"up": 90, "right": 0, "down": 270, "left": 180}
    pen.setheading(heading_dict[heading])

    pen.pendown()
    pen.goto(center_x, center_y)
    pen.penup()

    pen.pencolor(color)


def draw_maze(maze, pen, origin, sq_size):
    """Draws the maze lines om screen."""
    # iterate through squares one by one to decide where to draw walls
    for x in range(maze.dim):
        for y in range(maze.dim):
            if not maze.is_permissible([x, y], 'up'):
                pen.goto(origin + sq_size * x, origin + sq_size * (y + 1))
                pen.setheading(0)
                pen.pendown()
                pen.forward(sq_size)
                pen.penup()

            if not maze.is_permissible([x, y], 'right'):
                pen.goto(origin + sq_size * (x + 1), origin + sq_size * y)
                pen.setheading(90)
                pen.pendown()
                pen.forward(sq_size)
                pen.penup()

            # only check bottom wall if on lowest row
            if y == 0 and not maze.is_permissible([x, y], 'down'):
                pen.goto(origin + sq_size * x, origin)
                pen.setheading(0)
                pen.pendown()
                pen.forward(sq_size)
                pen.penup()

            # only check left wall if on leftmost column
            if x == 0 and not maze.is_permissible([x, y], 'left'):
                pen.goto(origin, origin + sq_size * y)
                pen.setheading(90)
                pen.pendown()
                pen.forward(sq_size)
                pen.penup()


if __name__ == '__main__':
    '''
    This script uses Python's turtle library to draw a picture of the maze
    given as an argument when running the script.
    '''

    # Create a maze based on input argument on command line.
    maze = Maze(str(sys.argv[1]))

    # Initialize the window and drawing turtle.
    window = turtle.Screen()
    pen = turtle.Turtle()
    pen.speed(0)
    pen.penup()

    # maze centered on (0,0), squares are 20 units in length.
    sq_size = 20
    origin = maze.dim * sq_size / -2

    window.tracer(0)
    draw_maze(maze, pen, origin, sq_size)
    window.update()
    window.tracer(1)

    if len(sys.argv) == 3:
        draw_path(str(sys.argv[2]), pen, origin, sq_size)

    pen.hideturtle()
    window.exitonclick()
