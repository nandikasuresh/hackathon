import numpy as np

def calculate_total_distance(tour, distance_matrix):
    total_distance = 0
    for i in range(len(tour) - 1):
        total_distance += distance_matrix[tour[i]][tour[i + 1]]
    total_distance += distance_matrix[tour[-1]][tour[0]] 
    return total_distance


def generate_initial_population(population_size, num_cities):
    population = [list(np.random.permutation(num_cities)) for _ in range(population_size)]
    return population

def crossover(parent1, parent2):
    start = np.random.randint(len(parent1))
    end = np.random.randint(start, len(parent1))

    child = [-1] * len(parent1)
    mapping = {}

    for i in range(start, end + 1):
        child[i] = parent1[i]
        mapping[parent1[i]] = parent2[i]

    for i in range(len(parent1)):
        if i < start or i > end:
            city = parent2[i]
            while city in child:
                city = mapping[city]
            child[i] = city

    return child

def mutate(tour):
    idx1, idx2 = np.random.choice(len(tour), 2, replace=False)
    tour[idx1], tour[idx2] = tour[idx2], tour[idx1]

def tournament_selection(population, distances, tournament_size):
    selected_indices = np.random.choice(len(population), tournament_size, replace=False)
    selected_individuals = [population[i] for i in selected_indices]
    selected_individuals.sort(key=lambda x: calculate_total_distance(x, distances))
    return selected_individuals[0]

def genetic_algorithm(distance_matrix, population_size=100, generations=100, tournament_size=5, mutation_rate=0.1):
    num_cities = len(distance_matrix)
    population = generate_initial_population(population_size, num_cities)

    for generation in range(generations):
        population.sort(key=lambda x: calculate_total_distance(x, distance_matrix))
        new_population = [population[0]] 

        while len(new_population) < population_size:
            parent1 = tournament_selection(population, distance_matrix, tournament_size)
            parent2 = tournament_selection(population, distance_matrix, tournament_size)
            child = crossover(parent1, parent2)

            if np.random.rand() < mutation_rate:
                mutate(child)

            new_population.append(child)

        population = new_population

    best_tour = population[0]
    best_distance = calculate_total_distance(best_tour, distance_matrix)
    return best_tour, best_distance

distance_matrix_example = [
    [0, 10, 15, 20],
    [10, 0, 35, 25],
    [15, 35, 0, 30],
    [20, 25, 30, 0]
]

best_tour, best_distance = genetic_algorithm(distance_matrix_example)
print("Best Tour:", best_tour)
print("Best Distance:", best_distance)


'''
import numpy as np

#nearest_neighbour
def nearest_neighbor(graph):
    num_cities = len(graph)
    unvisited_cities = set(range(1, num_cities))
    current_city = 0  # Start from the first city
    tour = [current_city]

    while unvisited_cities:
        nearest_city = min(unvisited_cities, key=lambda city: graph[current_city][city])
        tour.append(nearest_city)
        unvisited_cities.remove(nearest_city)
        current_city = nearest_city

    # Return to the starting city to complete the tour
    tour.append(tour[0])

    return tour

# Example usage:
graph_example = [
    [0, 10, 15, 20],
    [10, 0, 35, 25],
    [15, 35, 0, 30],
    [20, 25, 30, 0]
]

result_tour = nearest_neighbor(graph_example)
print("Nearest Neighbor Tour:", result_tour)
'''


'''
#branch_and_bound
def tsp_dynamic_programming(graph):
    n = len(graph)
    all_points_set = set(range(n))
    memo = {}

    def tsp_dp_helper(current, subset):
        if not subset:
            return graph[current][0] 

        subset_tuple = tuple(subset)
        if (current, subset_tuple) in memo:
            return memo[(current, subset_tuple)]

        min_cost = float('inf')
        for next_point in subset:
            if next_point != current:
                new_subset = tuple(point for point in subset if point != next_point)
                cost = graph[current][next_point] + tsp_dp_helper(next_point, new_subset)
                min_cost = min(min_cost, cost)

        memo[(current, subset_tuple)] = min_cost
        return min_cost

    optimal_route = None
    min_cost = float('inf')
    for start_point in range(1, n):
        subset = tuple(point for point in range(1, n) if point != start_point)
        cost = graph[0][start_point] + tsp_dp_helper(start_point, subset)
        if cost < min_cost:
            min_cost = cost
            optimal_route = (0, start_point) + tuple(subset)

    return optimal_route, min_cost

# Example usage:
graph_example = [
    [0, 10, 15, 20],
    [10, 0, 35, 25],
    [15, 35, 0, 30],
    [20, 25, 30, 0]
]

optimal_route, min_cost = tsp_dynamic_programming(l)
print("Optimal Route:", optimal_route)
print("Minimum Cost:", min_cost)


'''