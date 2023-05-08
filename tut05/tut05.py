from tabulate import tabulate  # pip install tabulate
import queue  # Importing the queue module
import threading  # Importing the threading module
import time  # Importing the time module
from os.path import exists  # Importing the os module
import sys  # Importing the sys module


class Router:  # Defining a Router class
    INF = 1e18  # Defining a large constant as infinity

    def __init__(self, router_id):  # Constructor to initialize a Router object
        self.id = router_id  # Assigning the router ID
        self.iteration_count = 0  # Initializing the iteration count to 0
        # Creating an empty dictionary to store the neighbours and edge costs -> dict[Router, int]
        self.neighbours = {}
        # Creating an empty list to store the routing table entries -> list[list[Router, Router, int]]
        self.routing_table = []
        # Creating a queue object to store the incoming adjacency matrix
        self.queue = queue.Queue()
        self.adjacency_matrix = []

    # Method to add a neighbour to the router
    def add_neighbours(self, neighbour_node, edge_cost):
        if self == neighbour_node:  # Check if neighbour node is the same as the router itself
            print("Self loop is not allowed.")  # Print a warning message
            exit(0)  # Exit the program
        # Add the neighbour and edge cost to the neighbours dictionary
        self.neighbours[neighbour_node] = edge_cost

    # Method to initialize the routing table of the router
    def initialise_routing_table(self, routers):
        for router in routers:  # Loop over all routers in the network
            # Check if the router is a neighbour of the current router
            if self.neighbours.get(router):
                self.routing_table.append(  # If yes, add a routing table entry for this neighbour
                    [router, router, self.neighbours.get(router)])  # [Destination, Via, Cost]
            elif router == self:  # If the router is the current router itself
                # Add a routing table entry for this router itself
                self.routing_table.append([router, router, 0])
            else:
                # Add a routing table entry for all other routers in the network
                self.routing_table.append([router, None, self.INF])

    # Method to initialize the adjacency matrix of the router
    def initialise_adjacency_matrix(self, routers):
        for u in routers:
            row = []
            if u == self:
                for v in routers:
                    if self.neighbours.get(v):
                        row.append(self.neighbours.get(v))
                    else:
                        row.append(0)
            else:
                for v in routers:
                    row.append(0)
            self.adjacency_matrix.append(row)

    # Method to print the routing table of a router
    def print_routing_table(self, lock):
        # Acquire the lock before executing the code inside the method
        lock.acquire()

        # Define a string of dashes for use in the output formatting
        DASHES = "-" * 41
        # Define a string to represent the router and iteration count for use in the output formatting
        head_string = f"From Router: {self.id} | Iteration Count: {self.iteration_count}"
        # Print a line of dashes to separate the output
        print(DASHES)
        # Print the header string for this router and iteration count, with proper spacing and alignment
        print(f"| {head_string:<37} |")
        # Print another line of dashes to separate the output
        print(DASHES)
        # Print the table headers for the routing table output
        print("| To Router  | Cost       | Via Router  |")
        print(DASHES)

        # Iterate through each element of the routing table
        for element in self.routing_table:
            # Extract the destination router ID from the element
            to_router = element[0].id

            # Print the routing table entry, with proper spacing and alignment
            print(f"| {to_router:<10} |"
                  + f" {element[2] if element[2] != self.INF else 'INF' :<10} |"
                  + f" {(element[1].id if element[1] else '--'):<11} |")

        # Print another line of dashes to separate the output
        print(DASHES, end="\n\n")
        # Release the lock
        lock.release()

    def print_adjacency_matrix(self, lock, routers):
        lock.acquire()

        print("Adjacency Matrix of " + self.id +
              " (Iteration Count: " + str(self.iteration_count) + ")")
        table = []
        row = []

        for router in routers:
            row.append(router.id)

        table.append(row)

        for index, each_row in enumerate(self.adjacency_matrix):
            row = [routers[index].id]
            row = row + self.adjacency_matrix[index]
            table.append(row)

        print(tabulate(table, headers='firstrow', tablefmt='grid'), end="\n\n")

        lock.release()

    def forward_adjacency_matrix(self):
        for neighbour in self.neighbours.keys():     # loop through each neighbour of the node
            # add the current node to the neighbour's queue
            neighbour.queue.put(self)

    # This function updates the adjacency matrix of a node by comparing its matrix with its neighbours.
    # If there is a change in the adjacency matrix, it returns True.
    def update_adjacency_matrix(self):
        # set a flag to track if there is a change in the adjacency matrix
        is_there_a_change = False

        while True:                                   # loop indefinitely
            try:
                # get a neighbour from the node's queue
                neighbour = self.queue.get(block=False)

                # get the adjacency matrix of the neighbour
                adjacency_matrix_of_neighbour = neighbour.adjacency_matrix

                # loop through the rows of the neighbour's matrix
                for i in range(len(adjacency_matrix_of_neighbour)):
                    # loop through the columns of the neighbour's matrix
                    for j in range(len(adjacency_matrix_of_neighbour)):
                        # compare the values in the matrices
                        if self.adjacency_matrix[i][j] < adjacency_matrix_of_neighbour[i][j]:
                            # if there is a change, set the flag to True
                            is_there_a_change = True
                            # update the adjacency matrix of the node
                            self.adjacency_matrix[i][j] = adjacency_matrix_of_neighbour[i][j]
            except:
                # break out of the loop if there are no more neighbours in the queue
                break

        # return the flag indicating if there was a change in the adjacency matrix
        return is_there_a_change

    # This function updates the routing table of a node based on shortest distances and parents.
    def update_routing_table(self, routers, shortest_distances, parents):
        # loop through the routers
        for index, router in enumerate(routers):
            if router == self:
                continue                  # if the router is the current node, skip it

            # create an empty list to store the nodes to go through to get to the destination node
            via = []
            current_vertex = index         # set the current vertex to the index of the router
            NO_PARENT = -1                 # set a constant for when there is no parent

            while current_vertex != NO_PARENT:             # loop until there is no parent
                # add the current vertex to the via list
                via.append(current_vertex)
                # set the current vertex to its parent
                current_vertex = parents[current_vertex]

            # update the routing table for the current router with the next hop and distance to the destination node
            self.routing_table[index] = (
                [router, routers[via[-2]], shortest_distances[index]])

    # Apply Dijkstra Algorithm once we get whole graph
    def apply_dijkstra(self, routers):

        start_vertex = -1
        NO_PARENT = -1

        for i in range(len(routers)):
            if routers[i] == self:
                start_vertex = i

            n_vertices = len(self.adjacency_matrix[0])

        # shortest_distances[i] will hold the
        # shortest distance from start_vertex to i
        shortest_distances = [sys.maxsize] * n_vertices

        # added[i] will true if vertex i is
        # included in shortest path tree
        # or shortest distance from start_vertex to
        # i is finalized
        added = [False] * n_vertices

        # Initialize all distances as
        # INFINITE and added[] as false
        for vertex_index in range(n_vertices):
            shortest_distances[vertex_index] = sys.maxsize
            added[vertex_index] = False

        # Distance of source vertex from
        # itself is always 0
        shortest_distances[start_vertex] = 0

        # Parent array to store shortest
        # path tree
        parents = [-1] * n_vertices

        # The starting vertex does not
        # have a parent
        parents[start_vertex] = NO_PARENT

        # Find shortest path for all
        # vertices
        for i in range(1, n_vertices):
            # Pick the minimum distance vertex
            # from the set of vertices not yet
            # processed. nearest_vertex is
            # always equal to start_vertex in
            # first iteration.
            nearest_vertex = -1
            shortest_distance = sys.maxsize
            for vertex_index in range(n_vertices):
                if not added[vertex_index] and shortest_distances[vertex_index] < shortest_distance:
                    nearest_vertex = vertex_index
                    shortest_distance = shortest_distances[vertex_index]

            # Mark the picked vertex as
            # processed
            added[nearest_vertex] = True

            # Update dist value of the
            # adjacent vertices of the
            # picked vertex.
            for vertex_index in range(n_vertices):
                edge_distance = self.adjacency_matrix[nearest_vertex][vertex_index]

                if edge_distance > 0 and shortest_distance + edge_distance < shortest_distances[vertex_index]:
                    parents[vertex_index] = nearest_vertex
                    shortest_distances[vertex_index] = shortest_distance + \
                        edge_distance

        self.update_routing_table(routers, shortest_distances, parents)


def parse_input():
    number_of_routers = 0               # Initialize the number of routers to zero
    routers = []                        # Create an empty list for routers
    # Create an empty dictionary for mapping router IDs to their index in the routers list
    routers_map = {}

  #  filename = "topology.txt"           # Set the filename to "topology.txt"

   # if not exists(filename):
   #     print("`topology.txt` does not exist in the current directory.")
    #    exit(0)

    with open("/content/topology.txt") as line:        # Open the file for reading
        # Read the first line and convert the string to an integer to get the number of routers
        number_of_routers = int(line.readline().strip())

        for index, router_id in enumerate(line.readline().strip().split(" ")):
            # Read the second line, split the string by space and iterate through the resulting list
            # Create a Router object for each router ID and add it to the routers list
            routers.append(Router(router_id))
            # Add the router ID and its index in the routers list to the routers_map dictionary
            routers_map[router_id] = index

        # If the length of the routers_map dictionary is not equal to the number_of_routers
        if len(routers_map) != number_of_routers:
            print("Duplicate nodes found.")         # Print an error message
            exit(0)                                 # And exit the program

        for edge in line.readlines():              # Iterate through the remaining lines of the file
            # Remove any leading or trailing white space from the line
            edge = edge.strip()
            if edge == "EOF":                       # If the line is "EOF"
                break                               # Stop iterating

            # Split the line by space and get a list of the resulting strings
            edge = edge.split(" ")
            if len(edge) != 3:                      # If the length of the list is not 3
                # Print an error message
                print(
                    "The edge description line must contain only three space separated words.")
                exit(0)                             # And exit the program

            # Get the index of the first node from the routers_map dictionary
            first_node = routers_map[edge[0]]
            # Get the index of the second node from the routers_map dictionary
            second_node = routers_map[edge[1]]
            # Convert the third string to an integer to get the edge cost
            edge_cost = int(edge[2])

            # Add the second node as a neighbour of the first node with the given edge cost
            routers[first_node].add_neighbours(routers[second_node], edge_cost)
            # Add the first node as a neighbour of the second node with the given edge cost
            routers[second_node].add_neighbours(routers[first_node], edge_cost)

    # Return the number of routers and the routers list as a tuple
    return number_of_routers, routers


# Method to initialize the routing table for each router object
def initialise_routers(routers):
    for router in routers:
        router.initialise_routing_table(routers)


# Method to initialize the adjacency matrix for each router object
def initialise_adjacency_matrices(routers):
    for router in routers:
        router.initialise_adjacency_matrix(routers)


# Method to print the routing table for all routers
# It takes a lock as an argument to prevent multiple threads from writing to the console simultaneously
def print_all_routing_tables(routers, lock):
    print("#" * 50, end="\n\n")
    for router in routers:
        router.print_routing_table(lock)
    print("#" * 50, end="\n\n")


# Method to simulate the process of routing information between routers
def simulate(router, lock, routers):
    # Set the waiting time between each iteration to 2 seconds
    waiting_time = 2
    # Set the flag to True indicating that a change is initially present in the routing table
    is_there_a_change = True
    # Set the flag to True indicating that the simulation can continue
    can_continue = True

    # Continue the simulation while the flag can_continue is True
    while can_continue:
        # Increment the iteration count of the current router
        router.iteration_count += 1

        # If there is a change in the adjacency matrix, forward the adjacency matrix
        if is_there_a_change:
            router.forward_adjacency_matrix()

        # Update the adjacency matrix and check if there is a change
        is_there_a_change = router.update_adjacency_matrix()

        # If there is a change in the adjacency matrix, print the adjacency matrix
        if is_there_a_change:
            router.print_adjacency_matrix(lock, routers)

        # Wait for the specified time before the next iteration
        time.sleep(waiting_time)

        # Set the flag can_continue to False
        can_continue = False

        # Check if any router has any incoming adjacency matrix in its queue, if so, set the can_continue flag to True
        for each_router in routers:
            can_continue |= not each_router.queue.empty()

    router.apply_dijkstra(routers)


def main():
    # Create a threading lock to control access to shared resources
    start_time = datetime.now()
    lock = threading.Lock()

    # Call parse_input function to get the number of routers and their details from topology.txt file
    number_of_routers, routers = parse_input()

    # Call the initialise_routers function to set the initial routing table for each router
    initialise_routers(routers)

    # Call the initialise_adjacency_matrices function to set the initial adjacency matrix for each router
    initialise_adjacency_matrices(routers)

    # Call the print_all_routing_tables function to print the initial routing table for all routers
    print_all_routing_tables(routers, lock)

    # Create a list to hold all the threads
    threads = []

    # Loop over all routers
    for router in routers:
        # Create a new thread for each router
        # The target function for each thread is simulate function
        # The router object, threading lock and routers list are passed as arguments to the simulate function
        thread = threading.Thread(
            target=simulate, args=(router, lock, routers))
        thread.start()
        threads.append(thread)

    # Loop over all threads
    for thread in threads:
        # Wait for each thread to finish executing
        thread.join()

    # Call the print_all_routing_tables function to print the final routing table for all routers
    print_all_routing_tables(routers, lock)

    print("Duration of Program Execution: {}".format(datetime.now()- start_time))


if __name__ == "__main__":
   
    main()
 