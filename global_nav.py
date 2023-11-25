# Code inspired from https://builtin.com/software-engineering-perspectives/dijkstras-algorithm
import heapq

class Node:
    def __init__(self):
        self.dist=float('inf') # Distance from start
        self.explored = False # Is the node explored
        self.parent = None # What was the previous node (on the shortest path yet)
        
def dijkstra(map, start):
    nodes = {}
    for iter in map:
        nodes[iter] = Node()
    nodes[start].dist = 0
    
    queue = [(0,start)]
    while queue: # Continues to run as long as queue is not empty
        d,node=heapq.heappop(queue) #
        if nodes[node].explored:
            continue
        nodes[node].explored = True
        for neighbor in graph[node]:
            