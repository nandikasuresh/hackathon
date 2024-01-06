import json

f_read = open('level1a.json')

data = json.load(f_read)

quantities = []
rest_distances = []
neighbourhoods = []
for i in data["neighbourhoods"]:
	neighbourhoods.append(i)
        
for j in data["restaurants"]["r0"]["neighbourhood_distance"]:
    rest_distances.append(j)
rest_distances.insert(0, 0)

graph = []
index = 1
for i in neighbourhoods:
    data["neighbourhoods"][i]["distances"].insert(0, rest_distances[index])
    index += 1
    graph.append(data["neighbourhoods"][i]["distances"])
    quantities.append(data["neighbourhoods"][i]["order_quantity"])
    
graph.insert(0, rest_distances)
#print(graph)

max = 600


def nearest_neighbor(graph):
    num_neigh = len(graph)
    unvisited_neigh = [i for i in range(1, num_neigh)]
    current_neigh = 0
    tours = []
    tour = [current_neigh]

    
    visited_path, unvisited_check = visited(unvisited_neigh, current_neigh, tour)
    tours.append(visited_path)

    while unvisited_check:
        current_neigh = 0
        tour = [current_neigh]
        visited_path, unvisited_check = visited(unvisited_check, current_neigh, tour)
        tours.append(visited_path)

    return tours

def visited(unvisited_neigh, current_neigh, tour):
    cost = 0
    while unvisited_neigh:
        nearest_neigh = min(unvisited_neigh, key=lambda neigh: graph[current_neigh][neigh])
        ind = unvisited_neigh.index(nearest_neigh)
        cost += quantities[ind]
        if cost > max:
            cost -= quantities[ind]
            tour.append(tour[0])
            return tour, unvisited_neigh
        tour.append(nearest_neigh)
        quantities.pop(ind)
        unvisited_neigh.remove(nearest_neigh)
        current_neigh = nearest_neigh
    tour.append(tour[0])
    return tour, unvisited_neigh
result_tour = nearest_neighbor(graph)

for i in result_tour:
    i[0] = i[len(i) - 1] = "r0"
    for j in range(1, len(i)-1):
        i[j] = 'n' + str(i[j]-1)

result = {}
paths = []
for k in range(len(result_tour)):
    paths.append("path" + str(k+1))

for k in range(len(result_tour)):
    result[paths[k]] = result_tour[k]

result = dict(v0 = (result))


with open('level1a_output.json', 'w') as f_write:
    json.dump(result, f_write)

f_read.close()
