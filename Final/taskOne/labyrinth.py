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
    def __init__(self, sides=None):
        """
        :param sides: forward, back, left, right, True if a wall exists
        """

        self.forward = sides[0]
        self.right = sides[1]
        self.back = sides[2]
        self.left = sides[3]

        self.sides = sides


class Labyrinth:

    def __init__(self, cells=None):
        """

        :param cells: list of cells
        """
        self.cells = cells
    def add_cell(self, cell):
        self.cells.append(cell)


def print_maze_map(maze_map):
    # maze_size_x = len(maze_map)
    # maze_size_y = len(maze_map[0])
    # print(f"maze_size_x: {maze_size_x} \nmaze_size_y: {maze_size_x}\n\n")

    for i, row in enumerate(maze_map, 0):
        for j, cell in enumerate(row, 0):
            messages = []
            if not cell.back:
                messages.append(f"{j + i*4 + 1}-{j + (i-1)*4 + 1}")
            if not cell.left:
                messages.append(f"{j + i*4 + 1}-{j - 1 + i*4 + 1}")
            if not cell.right:
                messages.append(f"{j + i*4 + 1}-{j + 1 + i*4 + 1}")
            if not cell.forward:
                messages.append(f"{j + i*4 + 1}-{j + (i+1)*4 + 1}")
            for k, message in enumerate(messages, 0):
                if k != len(messages) - 1:
                    print(message, end="; ")
                else:
                    print(message, end="")

            print("")


if __name__ == "__main__":
    maze_map = [
        [Cell([True, False, True, True]), Cell([False, False, True, False]), Cell([True, False, True, False]), Cell([False, True, True, False])],
        [Cell([False, False, True, True]), Cell([True, True, False, False]), Cell([True, False, True, True]), Cell([True, True, False, False])],
        [Cell([True, False, False, True]), Cell([False, True, True, False]), Cell([True, False, True, True]), Cell([False, True, True, False])],
        [Cell([True, False, True, True]), Cell([True, False, False, False]), Cell([True, False, True, False]), Cell([True, True, False, False])]
    ]
    print_maze_map(maze_map)

