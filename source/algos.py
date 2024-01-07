import pygame
from const import *
from maze import SearchSpace

def DFS(g: SearchSpace, sc: pygame.Surface):
    if (g.start.id == g.goal.id):
        print('You\'re already there')
        return

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
                if neighbor.id != g.goal.id:
                    g.grid_cells[neighbor.id].set_color(RED, sc)
        if node_id != g.start.id:
            g.grid_cells[node_id].set_color(BLUE, sc)
    
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
    if (g.start.id == g.goal.id):
        print('You\'re already there')
        return

    print('Implement BFS algorithm')

    visited = [g.start.id]
    queue = [g.start.id]
    father = [-1]*g.get_length()

    while len(queue) > 0:
        node_id = queue.pop(0)
        if node_id != g.start.id:
            g.grid_cells[node_id].set_color(YELLOW, sc)
        goal_found = False
        for neighbor in g.get_neighbors(g.grid_cells[node_id]):
            if not (neighbor.id in visited):
                visited.append(neighbor.id)
                queue.append(neighbor.id)
                father[neighbor.id] = node_id
                if neighbor == g.goal:
                    goal_found = True
                    break
                g.grid_cells[neighbor.id].set_color(BLUE, sc)
        if node_id != g.start.id:
            g.grid_cells[node_id].set_color(BLUE, sc)
        if goal_found:
            break
    
    if father[g.goal.id] == -1:
        print('No path found')
        return
    
    path = [g.goal.id]
    current_father = father[g.goal.id]
    while current_father != -1:
        path.append(current_father)
        current_father = father[current_father]
    
    length = g.stroke_path(path, sc)

    print("Done! Total segments: %d. Total length: %d" % (len(path)-1, length))

def Dijkstra(g: SearchSpace, sc: pygame.Surface):
    if (g.start.id == g.goal.id):
        print('You\'re already there')
        return
    
    print('Implement Dijkstra algorithm')

    visited = [g.start.id]
    queue = [g.start.id]
    father = [-1]*g.get_length()
    dist = [g.get_length()*14]*g.get_length()
    dist[g.start.id] = 0

    while len(queue) > 0:
        node_id = queue.pop(0)
        if node_id != g.start.id:
            g.grid_cells[node_id].set_color(YELLOW, sc)
        goal_found = False
        for neighbor in g.get_neighbors(g.grid_cells[node_id]):
            if not (neighbor.id in visited) and \
                dist[neighbor.id] > dist[node_id] + g.get_distance(node_id, neighbor.id):
                dist[neighbor.id] = dist[node_id] + g.get_distance(node_id, neighbor.id)
                visited.append(neighbor.id)
                queue.append(neighbor.id)
                queue.sort(key=lambda i:dist[i])
                father[neighbor.id] = node_id
                if neighbor == g.goal:
                    goal_found = True
                    break
                g.grid_cells[neighbor.id].set_color(BLUE, sc)
        if node_id != g.start.id:
            g.grid_cells[node_id].set_color(BLUE, sc)
        if goal_found:
            break
    
    if father[g.goal.id] == -1:
        print('No path found')
        return
    path = [g.goal.id]
    current_father = father[g.goal.id]
    while current_father != -1:
        path.append(current_father)
        current_father = father[current_father]
    
    length = g.stroke_path(path, sc)

    print("Done! Total segments: %d. Calculated length: %d. Actual length: %d"
          % (len(path)-1, dist[g.goal.id], length))

def AStar(g: SearchSpace, sc: pygame.Surface):
    if (g.start.id == g.goal.id):
        print('You\'re already there')
        return
    
    print('Implement AStar algorithm')

    open_set = [g.start.id]
    closed_set = []
    father = [-1]*g.get_length()
    # G Cost = Chiều dài đường đi từ g.start đến một node
    # H Cost = Khoảng cách Euclid từ một node đến g.goal
    gcost = [0]*g.get_length()
    hcost = [0]*g.get_length()

    def get_fcost(idx : int) -> int:
        return gcost[idx] + hcost[idx]

    def get_lowest_fcost_id() -> int:
        lowest_fcost_id = open_set[0]
        for id in open_set:
            if get_fcost(id) < get_fcost(lowest_fcost_id):
                lowest_fcost_id = id
        return lowest_fcost_id

    while True:
        current_id = get_lowest_fcost_id()
        open_set.remove(current_id)
        closed_set.append(current_id)
        if current_id == g.goal.id:
            break
        if current_id != g.start.id:
            g.grid_cells[current_id].set_color(YELLOW, sc)
        for neighbor in g.get_neighbors(g.grid_cells[current_id]):
            if neighbor.id in closed_set:
                continue
            new_gcost = gcost[current_id] + g.get_distance(current_id, neighbor.id)
            if new_gcost < gcost[neighbor.id] or not(neighbor.id in open_set):
                gcost[neighbor.id] = new_gcost
                hcost[neighbor.id] = g.get_distance(neighbor.id, g.goal.id)
                father[neighbor.id] = current_id
                if not (neighbor.id in open_set):
                    open_set.append(neighbor.id)
                    if neighbor.id != g.goal.id:
                        g.grid_cells[neighbor.id].set_color(RED, sc)
        if current_id != g.start.id:
            g.grid_cells[current_id].set_color(BLUE, sc)

    if father[g.goal.id] == -1:
        print("No path found")
        return
    
    path = [g.goal.id]
    current_father = father[g.goal.id]
    while current_father != -1:
        path.append(current_father)
        current_father = father[current_father]
    
    length = g.stroke_path(path, sc)

    print("Done! Total segments: %d. Calculated length: %d. Actual length: %d"
          % (len(path)-1, gcost[g.goal.id], length))