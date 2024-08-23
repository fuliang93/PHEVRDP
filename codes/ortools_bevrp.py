#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu May  2 11:45:01 2024

@author: fuliang
"""

"""Vehicles Routing Problem (VRP) with Time Windows."""

from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

def print_solution(data, manager, routing, solution):
    """Prints solution on console."""
    print(f"Objective: {solution.ObjectiveValue()}")
    time_dimension = routing.GetDimensionOrDie("Time")
    total_time = 0
    paths = {} # The paths of all vehicles
    for vehicle_id in range(data["num_vehicles"]):
        index = routing.Start(vehicle_id)
        # path = [manager.IndexToNode(index)] # The path of vehicle_id
        path = []
        plan_output = f"Route for vehicle {vehicle_id}:\n"
        while not routing.IsEnd(index):
            time_var = time_dimension.CumulVar(index)
            plan_output += (
                f"{manager.IndexToNode(index)}"
                f" Time({solution.Min(time_var)},{solution.Max(time_var)})"
                " -> "
            )
            path.append(manager.IndexToNode(index))  # path append
            index = solution.Value(routing.NextVar(index))
            
        time_var = time_dimension.CumulVar(index)
        plan_output += (
            f"{manager.IndexToNode(index)}"
            f" Time({solution.Min(time_var)},{solution.Max(time_var)})\n"
        )
        plan_output += f"Time of the route: {solution.Min(time_var)}min\n"
        print(plan_output)
        total_time += solution.Min(time_var)
        
        path.append(0)
        paths[vehicle_id] = path
    
    routes = []
    for i in paths.keys():
        for j in range(len(paths[i])-1):
            routes.append((paths[i][j],paths[i][j+1]))
        
    # print(routes)
    return routes, paths


# =============================================================================
# Solve the vrptw
# =============================================================================
def vrpSol(data, localSearchTime):
    
    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(
        len(data["distance_matrix"]), data["num_vehicles"], data["depot"]
    )

    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)

    # Create and register a transit callback.
    def distance_callback(from_index, to_index):
        """Returns the distance between the two nodes."""
        # Convert from routing variable Index to distance matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data["distance_matrix"][from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    # Define cost of each arc.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Distance constraint.
    dimension_name = "Distance"
    routing.AddDimension(
        transit_callback_index,
        0,  # no slack
        max(max(ele) for ele in data["distance_matrix"])*len(data["distance_matrix"])*len(data["distance_matrix"][0]),  # vehicle maximum travel distance
        True,  # start cumul to zero
        dimension_name,
    )
    # distance_dimension = routing.GetDimensionOrDie(dimension_name)
    # distance_dimension.SetGlobalSpanCostCoefficient(100)
        
    
    # Add Time Windows constraint.
    def time_callback(from_index, to_index):
        """Returns the travel time between the two nodes."""
        # Convert from routing variable Index to time matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data['time_matrix'][from_node][to_node] + data['service_time'][from_node]

    time_callback_index = routing.RegisterTransitCallback(time_callback)
            
    # Add Time Windows constraint.
    time = "Time"
    routing.AddDimension(
        time_callback_index,
        data['maxWaiting'],  # allow waiting time
        data['time_max'],  # maximum time per vehicle
        False,  # Don't force start cumul to zero.
        time,
    )
    time_dimension = routing.GetDimensionOrDie(time)
    # Add time window constraints for each location except depot.
    for location_idx, time_window in enumerate(data["time_windows"]):
        # print(location_idx, time_window)
        if location_idx == data["depot"]:
            continue
        index = manager.NodeToIndex(location_idx)
        # time_dimension.CumulVar(index).SetRange(time_window[0], time_window[1])
        time_dimension.SetCumulVarSoftLowerBound(index, time_window[0], 10**5)
        time_dimension.SetCumulVarSoftUpperBound(index, time_window[1], 10**8)
        
    # Add time window constraints for each vehicle start node.
    depot_idx = data["depot"]
    for vehicle_id in range(data["num_vehicles"]):
        index = routing.Start(vehicle_id)
        time_dimension.CumulVar(index).SetRange(
            data["time_windows"][depot_idx][0], data["time_windows"][depot_idx][1]
        )

    # Instantiate route start and end times to produce feasible times.
    for i in range(data["num_vehicles"]):
        routing.AddVariableMinimizedByFinalizer(
            time_dimension.CumulVar(routing.Start(i))
        )
        routing.AddVariableMinimizedByFinalizer(time_dimension.CumulVar(routing.End(i)))
    
    
    # Add Capacity constraint.
    def demand_callback(from_index):
        """Returns the demand of the node."""
        # Convert from routing variable Index to demands NodeIndex.
        from_node = manager.IndexToNode(from_index)
        return data["demands"][from_node]
     
    demand_callback_index = routing.RegisterUnaryTransitCallback(demand_callback)
    routing.AddDimensionWithVehicleCapacity(
        demand_callback_index,
        0,  # null capacity slack
        data["vehicle_capacities"],  # vehicle maximum capacities
        True,  # start cumul to zero
        "Capacity",
        ) 

    

    # Setting first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PARALLEL_CHEAPEST_INSERTION
    )    # PARALLEL_CHEAPEST_INSERTION ; SAVINGS ; AUTOMATIC
    
    search_parameters.local_search_metaheuristic = (
        #routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
        routing_enums_pb2.LocalSearchMetaheuristic.TABU_SEARCH
    ) # AUTOMATIC ; GUIDED_LOCAL_SEARCH; SIMULATED_ANNEALING; TABU_SEARCH; GENERIC_TABU_SEARCH
    search_parameters.time_limit.FromSeconds(localSearchTime)
    
    # Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)
    
    # print(solution)
    
    # Print solution on console.
    if solution:
        routes, paths = print_solution(data, manager, routing, solution)
    else:
        routes, paths = [], []
        
    return routes, paths

