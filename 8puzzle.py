from copy import deepcopy
import numpy as np
import heapq
from array import array


def cal_hamming(state):
    """Caluclate the cost ie, number of tiles that deviates from thier position"""
    h = sum(1 for i, el in enumerate(state) if (el and el == i+1))
    return 9-h


def cal_manhattan(state):
    """Caluclate the cost ie, manhattan distance"""
    h = sum(abs((val - 1) % 3 - i % 3) + abs((val - 1) // 3 - i // 3) for i, val in enumerate(state) if val)
    return h


def perform_move(lt):
    """Makes the all possible moves and yields the state after the move"""
    i = lt.index(0)
    if i in [3, 4, 5, 6, 7, 8]:  # down slide
        new_board = deepcopy(lt)
        new_board[i], new_board[i-3] = new_board[i-3], new_board[i]
        yield new_board
    if i in [1, 2, 4, 5, 7, 8]:  # right side
        new_board = deepcopy(lt)
        new_board[i], new_board[i-1] = new_board[i-1], new_board[i]
        yield new_board
    if i in [0, 1, 3, 4, 6, 7]:  # left slide
        new_board = deepcopy(lt)
        new_board[i], new_board[i+1] = new_board[i+1], new_board[i]
        yield new_board
    if i in [0, 1, 2, 3, 4, 5]:  # Up slide
        new_board = deepcopy(lt)
        new_board[i], new_board[i+3] = new_board[i+3], new_board[i]
        yield new_board


class Node:
    """A node class for A* Pathfinding"""

    def __init__(self, state, steps=0):

        self.state = state
        self.steps = steps
        # self.cost = cal_hamming(state) + self.steps
        self.cost = cal_manhattan(state) + self.steps

    def __lt__(self, other):
        return self.cost < other.cost


def main():
    import time

    "------> Test States <-----------"
    # l = [4, 1, 3, 7, 2, 6, 0, 5, 8]  # 6 Steps
    l = [7, 2, 4, 5, 0, 6, 8, 3, 1]  # 20 Steps
    # l = [6, 4, 7, 8, 5, 0, 3, 2, 1]  # difficult  depth first 49  //  31 (19.376s)
    l = [8, 6, 7, 2, 5, 4, 3, 0, 1]  # difficult  depth first 193 // A* Manhattan 31 (25.178 s)
    # l = [1, 2, 3, 4, 5, 6, 8, 7, 0]  # No Solution

    closed_nodes = set()
    open_nodes = []
    goal = [i for i in range(1, 9)]
    goal.append(0)
    current_node = Node(l)
    start_time = time.time()
    CN = ''.join(map(str, current_node.state))
    GOAL = ''.join(map(str, goal))


    while not CN == GOAL:
        if CN not in closed_nodes:
            closed_nodes.add(CN)
            for i in perform_move(current_node.state):
                if ''.join(map(str, i)) not in closed_nodes:
                    heapq.heappush(open_nodes, Node(i, (current_node.steps+1)))
            current_node = heapq.heappop(open_nodes)
        else:
            current_node = heapq.heappop(open_nodes)
        CN = ''.join(map(str, current_node.state))


    print(f'Iterations: {current_node.steps}')
    print('--------------------------------')
    print(f'Solved list : {current_node.state}')
    print("--- %s seconds ---" % (time.time() - start_time))


def old2main():
    import time
    start_time = time.time()

    "------> Test States <-----------"
    l = [4, 1, 3, 7, 2, 6, 0, 5, 8]  # 6 Steps
    l = [7, 2, 4, 5, 0, 6, 8, 3, 1]  # 20 Steps
    # l = [6, 4, 7, 8, 5, 0, 3, 2, 1]  # difficult
    # l = [8, 6, 7, 2, 5, 4, 3, 0, 1]  # difficult
    # l = [1, 2, 3, 4, 5, 6, 8, 7, 0]  # No Solution

    closed_nodes = []
    open_nodes = []

    start_node = Node(l, 0)
    end_node = Node(goal)
    while not start_node.state == end_node.state:
        if start_node.state not in closed_nodes:
            closed_nodes.append(start_node.state)
            open_nodes.extend([Node(i, (start_node.steps+1)) for i in perform_move(start_node.state)])
            ht = [cal_h(n) for n in open_nodes]
            start_node = open_nodes[ht.index(min(ht))]
        else:
            open_nodes.remove(start_node)
            if len(open_nodes) == 0:
                print('No Solution')
                break
            ht = [cal_h(n) for n in open_nodes]
            start_node = open_nodes[ht.index(min(ht))]
    print(f'Iterations: {start_node.steps}')
    print('--------------------------------')
    print(f'Solved list : {start_node.state}')
    print("--- %s seconds ---" % (time.time() - start_time))


def oldmain():
    goal = [i for i in range(1, 9)]
    goal.append(0)
    evaluated = []
    rest = []
    # eightPuzzle([4, 1, 3, 7, 2, 6, 0, 5, 8], repeated)

    l = [4, 1, 3, 7, 2, 6, 0, 5, 8]  # 6 Steps
    l = [7, 2, 4, 5, 0, 6, 8, 3, 1]
    # l = [6, 4, 7, 8, 5, 0, 3, 2, 1]  # difficult
    # l = [8, 6, 7, 2, 5, 4, 3, 0, 1]  # difficult
    # l = [1, 2, 3, 4, 5, 6, 8, 7, 0]  # No Solution
    import time
    start_time = time.time()
    while not l == goal:
        if l not in evaluated:
            evaluated.append(l)
            rest.extend([i for i in perform_move(l)])
            ht = [cal_manhattan(h) for h in rest]
            l = rest[ht.index(min(ht))]
        else:
            rest.remove(l)
            if len(rest) == 0:
                print('No Solution')
                break
            ht = [cal_manhattan(h) for h in rest]
            l = rest[ht.index(min(ht))]
    print(f'Iterations: {len(evaluated)}')
    print('--------------------------------')
    print(f'Solved list : {l}')
    print("--- %s seconds ---" % (time.time() - start_time))


if __name__ == '__main__':
    main()
