import pygame
from const import *
from maze import SearchSpace

def DFS(g: SearchSpace, sc: pygame.Surface):
    print('Implement DFS algorithm')

    open_set = [g.start.id]
    closed_set = []
    father = [-1]*g.get_length()

    raise NotImplementedError('not implemented')

def BFS(g: SearchSpace, sc: pygame.Surface):
    print('Implement BFS algorithm')

    open_set = [g.start.id]
    closet_set = [g.start.id]
    father = {g.start.id : -1}

    while len(open_set) > 0:
        node_id = open_set.pop(0)
        print(node_id)

        for neighbor in g.get_neighbors(g.grid_cells[node_id]):
            if not (neighbor.id in closet_set):
                open_set.append(neighbor.id)
                closet_set.append(neighbor.id)
                father[neighbor.id] = node_id
                if neighbor == g.goal:
                    break
                g.grid_cells[neighbor.id].set_color(BLUE, sc)
    
    if not (g.goal.id in father):
        print('No path found')
        return
    path = [g.goal.id]
    current_father = father[g.goal.id]
    while current_father != -1:
        path.append(current_father)
        current_father = father[current_father]
    
    for i in range(len(path)):
        if i >= len(path) - 1:
            break
        node_id = path[i]
        next_id = path[i+1]
        start_point = g.grid_cells[node_id].rect.center
        end_point = g.grid_cells[next_id].rect.center
        pygame.draw.line(surface = sc, color = WHITE, start_pos=start_point, end_pos=end_point, width=4)
        pygame.time.delay(5)
        pygame.display.update()
        
