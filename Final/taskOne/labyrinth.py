class Point:
    def __init__(self, x, y):
        """

        :param x: x coord
        :param y: y coord
        """
        self.x = x
        self.y = y


class State:
    def __init__(self, point, angle):
        """

        :param point: current position coordinates
        :param angle: angle in degrees
        """
        self.pos = point
        self.angle = angle

class Cell:
    def __init__(self, point, sides=None):
        """

        :param point: position af a cell
        :param sides: forward, back, left, right, True if a wall exists
        """
        self.point = point

        self.forward = sides[0]
        self.back = sides[1]
        self.left = sides[2]
        self.right = sides[3]

        self.sides = [
            self.forward,
            self.back,
            self.left,
            self.right
        ]


class Labyrinth:

    def __init__(self, cells=None):
        """

        :param cells: list of cells
        """
        self.cells = cells
    def add_cell(self, cell):
        self.cells.append(cell)

    def next_move_right_hand(self, forward, right):
        """

        :param current_state: x,y,angle
        :param forward: True if a wall exists
        :param right: True if a wall exists

        :return next_state: next position to fly
        """

        if not forward:
            if right:
                return "move forward"
            else:
                return "rotate right"
        else:
            return "rotate left"
        
