# 6.0002 Spring 2022
# Graph Optimization
# Finding shortest paths to drive from home to work on a road network


from graph import DirectedRoad, Node, RoadMap

def create_graph(map_filename):
    """
    Parses the map file and constructs a road map (graph).
    image.png
    Parameters:
        map_filename : str
            Name of the map file.

    Assumes:
        Each entry in the map file consists of the following format, separated by spaces:
            source_node destination_node travel_time road_type traffic_multiplier

        Note: hill road types always are uphill in the source to destination direction and
              downhill in the destination to the source direction. Downhill travel takes
              half as long as uphill travel. The travel_time represents the time to travel
              from source to destination (uphill).

    Returns:
        RoadMap
            A directed road map representing the given map.
    """
    road_map = RoadMap()
    with open(map_filename, "r") as open_file: # opens the file as a string, reads it
        file_contents = open_file.readlines()

        for line in file_contents:
            plain_line = line.strip() # removes new line characters 
            line_list = plain_line.split(" ") # splits each line at spaces

            # storing parameters in variables
            src_node, dest_node, travel_time, road_type, traffic_multiplier = line_list[0], line_list[1], line_list[2], line_list[3], line_list[4]

            # create instance of Node object for each node, casts travel_time and traffic_multiplier as floats
            src_node, dest_node = Node(src_node), Node(dest_node)
            travel_time, traffic_multiplier = float(travel_time), float(traffic_multiplier)

            # time going down a hill takes half the travel_time
            if road_type == "uphill":
                directed_road = DirectedRoad(src_node, dest_node, travel_time, "hill", traffic_multiplier)
                directed_road_reverse = DirectedRoad(dest_node, src_node, travel_time / 2, "hill", traffic_multiplier)
            
            # for every other road type, create an instance of DirectedRoad
            else:
                directed_road = DirectedRoad(src_node, dest_node, travel_time, road_type, traffic_multiplier)
                directed_road_reverse = DirectedRoad(dest_node, src_node, travel_time, road_type, traffic_multiplier)

            # check if both nodes are in road_map, add them if not
            if not road_map.contains_node(src_node):
                road_map.insert_node(src_node)
            if not road_map.contains_node(dest_node):
                road_map.insert_node(dest_node)

            # add roads and reverse roads to road_map
            road_map.insert_road(directed_road)
            road_map.insert_road(directed_road_reverse)

    return road_map


            
def find_shortest_path(roadmap, start, end, restricted_roads=None, has_traffic=False):
    """
    Finds the shortest path between start and end nodes on the road map,
    without using any restricted roads, following traffic conditions.
    Uses Dijkstra's algorithm

    Parameters:
        roadmap: RoadMap
            The graph on which to carry out the search.
        start: Node
            Node at which to start.
        end: Node
            Node at which to end.
        restricted_roads: list of str or None
            Road Types not allowed on path. If None, all are roads allowed
        has_traffic: bool
            Flag to indicate whether to get shortest path during traffic or not.

    Returns:
        A two element tuple of the form (best_path, best_time).
            The first item is a list of Node, the shortest path from start to end.
            The second item is a float, the length (time traveled) of the best path.
        If there exists no path that satisfies constraints, then return None.
    """
    # if the start and end nodes aren't in the roadmap, there's no point in calculating distance
    if not roadmap.contains_node(start) or not roadmap.contains_node(end):
        return None
    if start == end:
        return ([start], 0) # takes no time to get from start node to start node

    unvisited = roadmap.get_all_nodes() # set of nodes
    previous_node = {}
    time_to = {node: float("inf") for node in roadmap.get_all_nodes()} # dictionary with key being node and value being distance
    time_to[start] = float(0) 

    while len(unvisited) != 0: # while there are nodes to visit
        current = None
        for node in unvisited:
            if current == None: # if there is no current node yet, the first one is the first node in unvisited
                current = node
            elif time_to[node] < time_to[current]: # if the time to the unvisited node is less than the time to the current node,
                current = node # choose the unvisited node
            if node == end and time_to[node] < time_to[current]:
                current = end
                break
            

        if time_to[current] == float("inf"): # if the shortest distance is infinity, there is no way to reach end
            break
        if current == end: # end has been reached
            break


        if restricted_roads == None:
            road_list: list[DirectedRoad] = roadmap.get_reachable_roads_from_node(current, []) # list of roads branching from (initially start) node
        else:
            road_list: list[DirectedRoad] = roadmap.get_reachable_roads_from_node(current, restricted_roads) # list of roads branching from (initially start) node
        for road in road_list:
            accessible_node = road.get_destination_node() # nodes that are within one road of current node
            if time_to[current] + road.get_travel_time(has_traffic) < time_to[accessible_node]: # if the newly calculated time is less than the travel time to the accessible node, it's optimal
                time_to[accessible_node] = time_to[current] + road.get_travel_time(has_traffic) # for each accessible node, the travel time is the time taken to get to previous node plus the road's travel time to the accessible node
                previous_node[accessible_node] = current # the node's "parent" is the current node

        unvisited.remove(current)

    if end not in previous_node: 
        return None

    else:
        best_path = [end] # list containing the end node
        backtrack_node = end
        while backtrack_node != start:
            backtrack_node = previous_node[backtrack_node] # accessing the previous node for the best path iteratively
            best_path.append(backtrack_node)

        best_path.reverse()

        return (best_path, time_to[end])

        

def find_shortest_path_no_traffic(filename, start, end):
    """
    Finds the shortest path from start to end during conditions of no traffic.

    Parameters:
        filename: str
            Name of the map file that contains the graph
        start: Node
            Node object at which to start.
        end: Node
            Node object at which to end.

    Returns:
        list of Node
            The shortest path from start to end in normal traffic.
        If there exists no path, then return None.
    """
    roadmap = create_graph(filename)
    shortest_path = find_shortest_path(roadmap, start, end, None, False) 

    if shortest_path != None: # if there actually is a shortest path, 
        return shortest_path[0] # return the first entry of tuple (the path)
    else:
        return None 



def find_shortest_path_restricted(filename, start, end):
    """
    Finds the shortest path from start to end when local roads and hill roads cannot be used.

    Parameters:
        filename: str
            Name of the map file that contains the graph
        start: Node
            Node object at which to start.
        end: Node
            Node object at which to end.

    Returns:
        list of Node
            The shortest path from start to end given the aforementioned conditions.
        If there exists no path that satisfies constraints, then return None.
    """
    restricted_roads = ["local", "hill"]
    roadmap = create_graph(filename)
    shortest_path = find_shortest_path(roadmap, start, end, restricted_roads, False)

    if shortest_path != None:
        return shortest_path[0]
    else:
        return None



def find_shortest_path_in_traffic_no_toll(filename, start, end):
    """
    Finds the shortest path from start to end when toll roads cannot be used and in traffic,
    i.e. when all roads' travel times are multiplied by their traffic multipliers.

    Parameters:
        filename: str
            Name of the map file that contains the graph
        start: Node
            Node object at which to start.
        end: Node
            Node object at which to end.

    Returns:
        list of Node
            The shortest path from start to end given the aforementioned conditions.
        If there exists no path that satisfies the constraints, then return None.
    """
    restricted_roads = ["toll"]
    roadmap = create_graph(filename)
    shortest_path = find_shortest_path(roadmap, start, end, restricted_roads, True)

    if shortest_path != None:
        return shortest_path[0]
    else:
        return None



if __name__ == '__main__':

    pass

    small_map = create_graph('./maps/small_map.txt')

    road_map = create_graph("maps/test_create_graph.txt")
    print(road_map)

    start = Node('Denver')
    end = Node('Southlands')
    restricted_roads = []
    print(find_shortest_path(road_map, start, end, restricted_roads))
