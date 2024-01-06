import json

f_read = open('level0.json')

data = json.load(f_read)

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
    
graph.insert(0, rest_distances)


def nearest_neighbor(graph):
    num_neigh = len(graph)
    unvisited_neigh = set(range(1, num_neigh))
    current_neigh = 0
    tour = [current_neigh]

    while unvisited_neigh:
        nearest_neigh = min(unvisited_neigh, key=lambda neigh: graph[current_neigh][neigh])
        tour.append(nearest_neigh)
        unvisited_neigh.remove(nearest_neigh)
        current_neigh = nearest_neigh

    tour.append(tour[0])

    return tour


result_tour = nearest_neighbor(graph)
print("Nearest Neighbor Tour:", result_tour)

result_tour[0] = result_tour[len(result_tour) - 1] = "r0"
for i in range(1, len(result_tour)-1):
     result_tour[i] = 'n' + str(result_tour[i]-1)

result = dict(v0 = (dict(path = result_tour)))

with open('level0_output.json', 'w') as f_write:
    json.dump(result, f_write)

f_read.close()

