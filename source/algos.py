import pygame
from const import *
from maze import SearchSpace

def DFS(g: SearchSpace, sc: pygame.Surface):
    print('Implement DFS algorithm')

    open_set = [g.start.id]
    closed_set = []
    father = [-1]*g.get_length()

    while len(open_set) > 0:
        node_id = open_set.pop(0)
        closed_set.append(node_id)
        if node_id == g.goal.id:
            break
        if node_id != g.start.id:
            g.grid_cells[node_id].set_color(YELLOW, sc)
        for neighbor in g.get_neighbors(g.grid_cells[node_id]):
            if not (neighbor.id in closed_set):
                open_set.insert(0, neighbor.id)
                father[neighbor.id] = node_id
                g.grid_cells[neighbor.id].set_color(RED, sc)
        if node_id != g.start.id:
            g.grid_cells[node_id].set_color(BLUE, sc)
    
    g.start.set_color(ORANGE, sc)
    g.goal.set_color(PURPLE, sc)

    if father[g.goal.id] == -1:
        print('No path found')
        return
    path = [g.goal.id]
    current_father = father[g.goal.id]
    while current_father != -1:
        path.append(current_father)
        current_father = father[current_father]
    
    g.stroke_path(path, sc)


def BFS(g: SearchSpace, sc: pygame.Surface):
    print('Implement BFS algorithm')

    visited = [g.start.id]
    queue = [g.start.id]
    father = [-1]*g.get_length()

    while len(queue) > 0:
        node_id = queue.pop(0)
        g.grid_cells[node_id].set_color(YELLOW, sc)
        for neighbor in g.get_neighbors(g.grid_cells[node_id]):
            if not (neighbor.id in visited):
                visited.append(neighbor.id)
                queue.append(neighbor.id)
                g.grid_cells[neighbor.id].set_color(BLUE, sc)
                father[neighbor.id] = node_id
                if neighbor == g.goal:
                    break
        g.grid_cells[node_id].set_color(BLUE, sc)
    
    g.start.set_color(ORANGE, sc)
    g.goal.set_color(PURPLE, sc)
    
    if father[g.goal.id] == -1:
        print('No path found')
        return
    
    path = [g.goal.id]
    current_father = father[g.goal.id]
    while current_father != -1:
        path.append(current_father)
        current_father = father[current_father]
    
    g.stroke_path(path, sc)

def Dijkstra(g: SearchSpace, sc: pygame.Surface):
    print('Implement Dijkstra algorithm')

    visited = [g.start.id]
    queue = [g.start.id]
    father = [-1]*g.get_length()
    dist = [g.get_length()]*g.get_length()
    dist[g.start.id] = 0

    while len(queue) > 0:
        node_id = queue.pop(0)
        g.grid_cells[node_id].set_color(YELLOW, sc)
        for neighbor in g.get_neighbors(g.grid_cells[node_id]):
            if not (neighbor.id in visited) and \
                dist[neighbor.id] > dist[node_id] + get_distance(node_id, neighbor.id):
                dist[neighbor.id] = dist[node_id] + get_distance(node_id, neighbor.id)
                visited.append(neighbor.id)
                queue.append(neighbor.id)
                queue.sort(key=lambda i:dist[i])
                g.grid_cells[neighbor.id].set_color(BLUE, sc)
                father[neighbor.id] = node_id
                if neighbor == g.goal:
                    break
        g.grid_cells[node_id].set_color(BLUE, sc)
    
    g.start.set_color(ORANGE, sc)
    g.goal.set_color(PURPLE, sc)

    if father[g.goal.id] == -1:
        print('No path found')
        return
    path = [g.goal.id]
    current_father = father[g.goal.id]
    while current_father != -1:
        path.append(current_father)
        current_father = father[current_father]
    
    g.stroke_path(path, sc)

def get_distance(node_a_id : int, node_b_id : int) -> int:
    diff_x = abs(node_a_id/COLS - node_b_id/COLS)
    diff_y = abs(node_a_id%COLS - node_b_id%COLS)
    if diff_x > diff_y:
        return 14*diff_y + 10*(diff_x-diff_y)
    return 14*diff_x + 10*(diff_y-diff_x)