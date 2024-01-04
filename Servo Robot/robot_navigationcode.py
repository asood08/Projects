#Code for navigation

def make_graph(num_subloops, tables_list):
    graph = {}
    graph[(0,0)] = [(1,0)]
    for i in range(1, num_subloops + 1):
        tables_num = tables_list[i]
        # First Node in Subloop
        graph[(i,0)] = [(i-1, tables_list[i-1]), (i, 1), ((i+1)%(num_subloops+1), 0)]
        for j in range(1, tables_num):
            graph[(i, j)] = [(i, j+1), (i, j-1)]
        # Last Node in Subloop
        graph[(i, tables_num)] = [((i+1)%(num_subloops+1), 0), (i, tables_num-1)]


    for x, y in graph.items():
        print(x, ":",y)
        
    return graph

def get_route(graph, start=(0,0), to=(2,0)):
    visited = []
    queue = [(start, [start])]
    while queue:
        m = queue.pop(0)
        for neighbour in graph[m[0]]:
            if neighbour == to:
                return m[1] + [to]
            if neighbour not in visited:
                visited.append(neighbour)
                queue.append((neighbour, m[1] + [neighbour]))
        
    print("No Path")
    
num_subloops = int(input("Number Subloops: "))
tables_list = [0,]
for i in range(num_subloops):
     table_input = int(input(f"Number of junctions in subloop {i+1}: "))
     tables_list.append(table_input)
graph = make_graph(num_subloops, tables_list)

while True: 
    start = input("Start Location: ")
    start = (int(start.split()[0]), int(start.split()[1]))
    end = input("End Location: ")
    end = (int(end.split()[0]), int(end.split()[1]))
    print(get_route(graph, start, end))    


"""
Sample Input/ Output
Number Subloops: 5
Number of junctions in subloop 1: 5
Number of junctions in subloop 2: 5
Number of junctions in subloop 3: 5
Number of junctions in subloop 4: 5
Number of junctions in subloop 5: 5
(0, 0) : [(1, 0)]
(1, 0) : [(0, 0), (1, 1), (2, 0)]
(1, 1) : [(1, 2), (1, 0)]
(1, 2) : [(1, 3), (1, 1)]
(1, 3) : [(1, 4), (1, 2)]
(1, 4) : [(1, 5), (1, 3)]
(1, 5) : [(2, 0), (1, 4)]
(2, 0) : [(1, 5), (2, 1), (3, 0)]
(2, 1) : [(2, 2), (2, 0)]
(2, 2) : [(2, 3), (2, 1)]
(2, 3) : [(2, 4), (2, 2)]
(2, 4) : [(2, 5), (2, 3)]
(2, 5) : [(3, 0), (2, 4)]
(3, 0) : [(2, 5), (3, 1), (4, 0)]
(3, 1) : [(3, 2), (3, 0)]
(3, 2) : [(3, 3), (3, 1)]
(3, 3) : [(3, 4), (3, 2)]
(3, 4) : [(3, 5), (3, 3)]
(3, 5) : [(4, 0), (3, 4)]
(4, 0) : [(3, 5), (4, 1), (5, 0)]
(4, 1) : [(4, 2), (4, 0)]
(4, 2) : [(4, 3), (4, 1)]
(4, 3) : [(4, 4), (4, 2)]
(4, 4) : [(4, 5), (4, 3)]
(4, 5) : [(5, 0), (4, 4)]
(5, 0) : [(4, 5), (5, 1), (0, 0)]
(5, 1) : [(5, 2), (5, 0)]
(5, 2) : [(5, 3), (5, 1)]
(5, 3) : [(5, 4), (5, 2)]
(5, 4) : [(5, 5), (5, 3)]
(5, 5) : [(0, 0), (5, 4)]
Start Location: 0 0
End Location: 5 3
[(0, 0), (1, 0), (2, 0), (3, 0), (4, 0), (5, 0), (5, 1), (5, 2), (5, 3)]
Start Location: 4 3
End Location: 2 2
[(4, 3), (4, 4), (4, 5), (5, 0), (0, 0), (1, 0), (2, 0), (2, 1), (2, 2)]
"""








