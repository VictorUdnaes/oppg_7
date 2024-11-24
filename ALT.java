import java.io.*;
import java.util.*;
import java.util.concurrent.*;

public class ALT {
    private final Graph graph;
    private final PoiType[] pointsOfInterest;
    private final Long[][] poiToDistanceArr;
    private final Long[][] poiFromDistanceArr;

    private final List<Map<Integer, Integer>> poiMappingTable = new ArrayList<>();
    public static final int[] LANDMARK_IDS = {253181, 4897239, 4439984, 2620420};
    private static final String NODE_PATH = "noder_norden.txt";
    private static final String VERTICES_PATH = "kanter_norden.txt";
    private static final String POI_PATH = "interessepkt_norden.txt";
    private static final String POI_FROM_DISTANCES = "poiFromDistances.txt";
    private static final String POI_TO_DISTANCES = "poiToDistances.txt";

    public ALT() {
        graph = new Graph(7956886, 17815613);
        pointsOfInterest = new PoiType[graph.size()];
        readValuesFromFiles();
        poiToDistanceArr = new Long[4][graph.size()];
        poiFromDistanceArr = new Long[4][graph.size()];
        for (Long[] row : poiToDistanceArr) {
            Arrays.fill(row, Long.MAX_VALUE);
        }
    }

    public List<Map<Integer, Integer>> getPoiMappingTable() {
        return poiMappingTable;
    }

    public static void main(String[] args) {
        new ALT();
    }

    private void readValuesFromFiles() {
        System.out.println("Loading nodes and vertices from file...");
        readNodesFromFile(graph);
        readVerticesFromFile(graph);
        System.out.println("Loading points of interest from file...");
        readPointsOfInterestFromFile();
        System.out.println("Successfully loaded nodes and vertices from file");
    }

    public void setupPOIDistances() {
        findFurthestNodes(graph.getNode(graph.size() / 2));
        File file = new File(POI_TO_DISTANCES);
        if (file.exists()) {
            System.out.println("POI distances already computed");
            readPOIDistancesFromFile();
        } else {
            System.out.println("Computing POI distances...");
            computeDistancesFromLandmarks();
        }
        System.out.println("Successfully loaded POI distances from file");
    }

    public class AltPathResult {
        public final long totalDistance;
        public final List<Node> path;

        public AltPathResult(long totalDistance, List<Node> path) {
            this.totalDistance = totalDistance;
            this.path = path;
        }
    }

    public AltPathResult runAlt(Node startNode, Node endNode) {
        if (startNode == null || endNode == null) {
            return null;
        }

        // Reset all nodes
        for (Node node : graph.nodes) {
            node.distanceFromStart = Long.MAX_VALUE;  // Actual distance from start
            node.distanceToGoal = Long.MAX_VALUE;  // Estimated cost to goal (heuristic)
            node.visited = false;
            node.previous = null;
        }

        // Initialize start node
        startNode.distanceFromStart = 0;
        startNode.distanceToGoal = calculateHeuristic(startNode, endNode);

        // Priority queue ordered by fScore = distanceFromStart + distanceToGoal
        PriorityQueue<Node> openSet = new PriorityQueue<>(
                Comparator.comparingLong(a -> a.distanceFromStart + a.distanceToGoal)
        );
        openSet.add(startNode);

        // Set to track nodes we've already processed
        Set<Integer> closedSet = new HashSet<>();

        while (!openSet.isEmpty()) {
            Node current = openSet.poll();

            if (current.id == endNode.id) {
                System.out.println("nodes visited ALT: " + closedSet.size());
                return reconstructPath(current, startNode);
            }

            closedSet.add(current.id);

            for (Vertex edge : graph.findNeighbours(current)) {
                if (edge == null) {
                    continue;
                }
                Node neighbor = graph.nodes[edge.toNode];

                if (neighbor == null || closedSet.contains(neighbor.id)) {
                    continue;
                }

                // Calculate tentative gScore
                long tentativeGScore = current.distanceFromStart + edge.driveTime;

                if (tentativeGScore > neighbor.distanceFromStart) {
                    continue;  // This is not a better path
                }

                // This path is the best until now. Record it!
                neighbor.previous = current;
                neighbor.distanceFromStart = tentativeGScore;
                neighbor.distanceToGoal = calculateHeuristic(neighbor, endNode);

                if (!openSet.contains(neighbor)) {
                    openSet.add(neighbor);
                } else {
                    // Update position in priority queue
                    openSet.remove(neighbor);
                    openSet.add(neighbor);
                }
            }
        }

        return null;  // No path found
    }

    private long calculateHeuristic(Node current, Node end) {
        int numLandmarks = poiMappingTable.size();

        if (numLandmarks == 0) {
            return 0;
        }

        long maxEstimate = 0;

        // Use triangle inequality with landmarks (POIs)
        for (int i = 0; i < numLandmarks; i++) {
            Node landmark = graph.getNode(getPoiArrMapping(i));
            if (landmark == null) continue;

            // Distances from landmark to nodes and vice versa
            long distLandmarkToEnd = poiToDistanceArr[i][end.id];
            long distLandmarkToCurrent = poiToDistanceArr[i][current.id];
            long distCurrentToLandmark = poiFromDistanceArr[i][current.id];
            long distEndToLandmark = poiFromDistanceArr[i][end.id];

            // Check validity of distances and apply triangle inequality in both directions
            if (distLandmarkToEnd != Long.MAX_VALUE && distLandmarkToCurrent != Long.MAX_VALUE) {
                long estimate = Math.abs(distLandmarkToEnd - distLandmarkToCurrent);
                maxEstimate = Math.max(maxEstimate, estimate);
            }
            if (distCurrentToLandmark != Long.MAX_VALUE && distEndToLandmark != Long.MAX_VALUE) {
                long estimate = Math.abs(distCurrentToLandmark - distEndToLandmark);
                maxEstimate = Math.max(maxEstimate, estimate);
            }
        }

        return maxEstimate;
    }

    private AltPathResult reconstructPath(Node endNode, Node startNode) {
        List<Node> path = new ArrayList<>();
        Node current = endNode;

        while (current != startNode) {
            path.add(current);
            if (current.previous == null) {
                return null;  // Path is incomplete
            }
            current = current.previous;
        }
        path.add(startNode);
        Collections.reverse(path);
        return new AltPathResult(endNode.distanceFromStart, path);
    }

    // DIJKSTA ---------------------------------------------------------------------------------------------------------
    public Node[] runDijkstrasAlgorithm(Node startNode, Node endNode) {
        if (startNode == null || endNode == null) {
            return null;
        }

        // Initialize all nodes
        for (Node node : graph.nodes) {
            node.distanceFromStart = Long.MAX_VALUE;
            node.visited = false;
            node.previous = null;
        }

        startNode.distanceFromStart = 0;

        PriorityQueue<Node> queue = new PriorityQueue<>(
                Comparator.comparingLong(node -> node.distanceFromStart)
        );
        queue.add(startNode);

        Set<Integer> closedSet = new HashSet<>();

        while (!queue.isEmpty()) {
            Node currentNode = queue.poll();

            if (currentNode.id == endNode.id) {
                break;
            }

            currentNode.visited = true;
            closedSet.add(currentNode.id);

            for (Vertex vertex : graph.findNeighbours(currentNode)) {
                Node neighbour = vertex != null ? graph.nodes[vertex.toNode] : null;

                if (neighbour == null || neighbour.visited) {
                    continue;
                }

                long newDistance = currentNode.distanceFromStart + vertex.driveTime; // Prevent integer overflow
                if (newDistance < neighbour.distanceFromStart) {
                    neighbour.distanceFromStart = newDistance;
                    neighbour.previous = currentNode;
                    queue.add(neighbour); // Add updated neighbor to the queue
                }
            }
        }

        List<Node> path = new ArrayList<>();
        Node currentNode = endNode;
        long totalPathLength = 0;

        // Build the path and calculate the total path length
        while (currentNode != null && currentNode != startNode) {
            path.add(currentNode);
            currentNode = currentNode.previous;
        }

        path.add(startNode); // Add start node to the path
        Collections.reverse(path);

        // Display the results
        System.out.println("nodes visited Dijkstra: " + closedSet.size());
        return path.toArray(new Node[0]);
    }

    // ---------------------------------------------------------------------------------------------------------------------
    /*
    Separate Dijsktra's algorithm for preprocessing, only used to find distances between nodes and POIs and uses
    lists instead of the actual graph. This is to avoid race conditions when running the algorithm in parallel.
     */
    public long[] computeDistancesFromLandmark(Node landmark) {
        if (landmark == null) {
            return null;
        }

        int numNodes = graph.size();
        long[] distances = new long[numNodes];
        boolean[] visited = new boolean[numNodes];

        Arrays.fill(distances, Long.MAX_VALUE);
        Arrays.fill(visited, false);

        distances[landmark.id] = 0;

        PriorityQueue<Node> queue = new PriorityQueue<>(Comparator.comparingLong(node -> distances[node.id]));
        queue.add(landmark);

        while (!queue.isEmpty()) {
            Node currentNode = queue.poll();
            if (visited[currentNode.id]) {
                continue;
            }
            visited[currentNode.id] = true;

            for (Vertex vertex : graph.findNeighbours(currentNode)) {
                if (vertex == null) {
                    continue;
                }
                Node neighbour = graph.getNode(vertex.toNode);

                long newDistance = distances[currentNode.id] + vertex.driveTime;
                if (newDistance < distances[neighbour.id]) {
                    distances[neighbour.id] = newDistance;
                    queue.add(neighbour);
                }
            }
        }

        return distances;
    }

    // FURTHEST NODES ------------------------------------------------------------------------------------------------------
    public void findFurthestNodes(Node startNode) {
        /*
        if (startNode == null) {
            throw new IllegalArgumentException("Start node cannot be null");
        }
        var furthestNodes = new Node[graph.size()];
        var visitedNodes = new Node[graph.size()]; // Track visited nodes for efficient resetting

        for (int i = 0; i < 4; i++) {
            System.out.println("Finding furthest node " + (i + 1) + "...");
            // Reset only visited nodes
            for (Node node : visitedNodes) {
                if (node == null) {
                    continue;
                }
                node.distanceFromStart = Long.MAX_VALUE;
                node.visited = false;
                node.previous = null;
            }
            visitedNodes = new Node[graph.size()];

            // Initialize priority queue with all previously found furthest nodes
            PriorityQueue<Node> priorityQueue = new PriorityQueue<>(
                    Comparator.comparingLong(node -> node.distanceFromStart)
            );
            if (i == 0) {
                startNode.distanceFromStart = 0;
                priorityQueue.add(startNode);
                visitedNodes[startNode.id] = startNode;
            } else {
                for (Node node : furthestNodes) {
                    if (node == null) {
                        continue;
                    }
                    node.distanceFromStart = 0;
                    priorityQueue.add(node);
                    visitedNodes[node.id] = node;
                }
            }

            Node furthestNode = null;
            long maxDistance = 0;

            while (!priorityQueue.isEmpty()) {
                Node currentNode = priorityQueue.poll();
                if (currentNode.id % 100000 == 0) {
                    System.out.println("Current node: " + currentNode.id);
                }
                currentNode.visited = true;
                // Update the furthest node if we found a longer path
                if (currentNode.distanceFromStart > maxDistance
                        && currentNode.distanceFromStart != Integer.MAX_VALUE
                        && pointsOfInterest[currentNode.id] == null) {
                    maxDistance = currentNode.distanceFromStart;
                    furthestNode = currentNode;
                }
                try {
                    for (Vertex vertex : graph.findNeighbours(currentNode)) {
                        Node neighbour = vertex != null ? graph.nodes[vertex.toNode] : null;
                        if (neighbour == null || neighbour.visited) continue;

                        long newDistance = currentNode.distanceFromStart + vertex.driveTime;

                        if (newDistance < neighbour.distanceFromStart) {
                            neighbour.distanceFromStart = newDistance;
                            neighbour.previous = currentNode;
                            priorityQueue.add(neighbour);
                            if (visitedNodes[neighbour.id] == null) {
                                visitedNodes[neighbour.id] = neighbour;
                            }
                        }
                    }
                }catch (NullPointerException e){
                    System.out.println(e.getMessage());
                }
            }
            if (furthestNode != null) {
                furthestNodes[furthestNode.id] = furthestNode;
                System.out.println("Furthest node " + (i + 1) + " found: " + furthestNode.id);
                poiMappingTable.add(Map.of(i, furthestNode.id)); // Maps actual node id to poi id (0-3) for future reference
            }
        }
         */
        for (int i = 0; i < 4; i++) {
            Node furthestNode = graph.getNode(LANDMARK_IDS[i]);
            poiMappingTable.add(Map.of(i, furthestNode.id));
        }
    }

    // -----------------------------------------------------------------------------------------------------------------
    public void computeDistancesFromLandmarks() {
        for (int i = 0; i < 2; i++) {
            int numNodes = graph.size();
            int numPois = 4;
            long[][] distanceResults = new long[numPois][numNodes];  // 2D array to store distances

            // Create a thread pool to execute tasks in parallel
            ExecutorService executor = Executors.newFixedThreadPool(Runtime.getRuntime().availableProcessors());

            // Create a list to keep track of all submitted tasks
            List<Future<Void>> futures = new ArrayList<>();

            // Submit tasks for each POI
            for (int poi = 0; poi < numPois; poi++) {
                System.out.println("Submitting task for POI: " + poi);
                Callable<Void> task = getVoidCallable(poi, distanceResults);

                // Submit the task and add the future to the list
                futures.add(executor.submit(task));
            }

            // Wait for all tasks to complete
            for (Future<Void> future : futures) {
                try {
                    future.get();  // This blocks until the task is complete, ensuring all tasks finish
                } catch (InterruptedException | ExecutionException e) {
                    e.printStackTrace();
                }
            }

            // Shut down the executor service
            executor.shutdown();

            // Write the results to the file after all computations are done
            try (BufferedWriter writer = new BufferedWriter(new FileWriter(i == 0 ? POI_FROM_DISTANCES : POI_TO_DISTANCES))) {
                for (int nodeId = 0; nodeId < numNodes; nodeId++) {
                    StringBuilder line = new StringBuilder();
                    for (int poi = 0; poi < numPois; poi++) {
                        line.append(distanceResults[poi][nodeId]).append(" ");
                    }
                    writer.write(line.toString().trim());
                    writer.newLine();
                }
            } catch (Exception e) {
                e.printStackTrace();  // Handle exceptions appropriately
            }
            graph.reverse();
        }
    }

    private Callable<Void> getVoidCallable(int poi, long[][] distanceResults) {
        return () -> {
            Node poiNode = graph.getNode(getPoiArrMapping(poi));
            System.out.println("Computing distances from POI: " + poiNode.id);

            // Run Dijkstra's algorithm to find the shortest paths from the POI to all nodes
            long[] distances = computeDistancesFromLandmark(poiNode);
            // Store the distances in the 2D array
            distanceResults[poi] = distances;

            return null;  // Since Callable must return something, we return null
        };
    }

    // IO --------------------------------------------------------------------------------------------------------------
    private static void readNodesFromFile(Graph graph) {
        try (Scanner scanner = new Scanner(new File(NODE_PATH))) {
            scanner.nextLine(); // Skip first line
            while (scanner.hasNextLine()) {
                String[] parts = scanner.nextLine().split("\\s+");
                int id = Integer.parseInt(parts[0]);
                double latitude = Double.parseDouble(parts[1]);
                double longitude = Double.parseDouble(parts[2]);
                graph.addNode(id, new Node(id, null, latitude, longitude, Long.MAX_VALUE, Long.MAX_VALUE, false));
            }
        } catch (FileNotFoundException e) {
            System.out.println("File not found");
        }
    }

    /*
    TODO:
      de orignale tekstfilene har antall noder som første entry som fucker opp hele parsingen.
      gidder ikke deale med det nå men implementer noe som skipper første line. gjelder alle filene.
    */
    private void readPointsOfInterestFromFile() {
        try (BufferedReader reader = new BufferedReader(new FileReader(POI_PATH))) {
            String line;
            reader.readLine(); // Skip first line
            while ((line = reader.readLine()) != null) {
                String[] parts = line.trim().split("\\s+");
                int id = Integer.parseInt(parts[0]);
                var poiType = new PoiType(Integer.parseInt(parts[1]), parts[2]);
                pointsOfInterest[id] = poiType;
            }
        } catch (IOException e) {
            //System.out.println("File not found");
        }
    }

    record PoiType(int code, String name) {
    }

    private static void readVerticesFromFile(Graph graph) {
        try (BufferedReader reader = new BufferedReader(
                new FileReader(VERTICES_PATH),
                8192)) // Increased buffer size
        {
            String line;
            reader.readLine(); // Skip first line
            while ((line = reader.readLine()) != null) {
                String[] parts = line.trim().split("\\s+");
                int key = Integer.parseInt(parts[0]);
                int toNode = Integer.parseInt(parts[1]);
                int driveTime = Integer.parseInt(parts[2]);
                int distance = Integer.parseInt(parts[3]);
                int speedLimit = Integer.parseInt(parts[4]);
                graph.addVertex(key, new Vertex(toNode, driveTime, distance, speedLimit));
            }
        } catch (IOException e) {
            throw new RuntimeException("Failed to read vertices", e);
        }
    }

    private void readPOIDistancesFromFile() {
        for (int i = 0; i < 2; i++) {
            try (BufferedReader reader = new BufferedReader(new FileReader(i == 0 ? POI_FROM_DISTANCES : POI_TO_DISTANCES))) {
                String line;
                int j = 0;
                while ((line = reader.readLine()) != null) {
                    String[] parts = line.trim().split("\\s+");
                    for (int k = 0; k < 4; k++) {
                        long distance = Long.parseLong(parts[k]);
                        if (i == 0) {
                            poiFromDistanceArr[k][j] = distance;
                        } else {
                            poiToDistanceArr[k][j] = distance;
                        }
                    }
                    j++;
                }
            } catch (IOException e) {
                System.out.println(e.getMessage());
            }
        }
    }

// ---------------------------------------------------------------------------------------------------------------------

    public Graph getGraph() {
        return this.graph;
    }

    public int getPoiArrMapping(int id) {
        return poiMappingTable.get(id).values().iterator().next();
    }

}

class Graph {
    final Node[] nodes;
    final Vertex[][] vertices;

    public Graph(int nodeAmount, int vertexAmount) {
        // Use initial capacity to avoid resizing
        nodes = new Node[nodeAmount];
        vertices = new Vertex[vertexAmount][10]; // Assume max 10 neighbors per node
    }

    public Node getNode(int id) {
        return nodes[id];
    }

    public Node getNodeByCoordinates(double latitude, double longitude) {
        for (Node node : nodes) {
            if (node.latitude == latitude && node.longitude == longitude) {
                return node;
            }
        }
        return null;
    }

    public void addNode(int id, Node node) {
        nodes[id] = node;
    }

    public Vertex[] findNeighbours(Node node) {
        return vertices[node.id];
    }

    // When adding vertices, make them unmodifiable right away
    public void addVertex(int fromNode, Vertex vertex) {
        for (int i = 0; i < vertices[fromNode].length; i++) {
            if (vertices[fromNode][i] == null) {
                vertices[fromNode][i] = vertex;
                break;
            }
        }
    }

    public Node[] getNodes() {
        return nodes;
    }

    public int size() {
        return nodes.length;
    }

    public void reverse() {
        // Create a temporary structure to hold the reversed edges
        Vertex[][] reversedVertices = new Vertex[vertices.length][10]; // Assume max 10 neighbors per node

        // Iterate through all nodes and reverse the edges
        for (int fromNode = 0; fromNode < vertices.length; fromNode++) {
            Vertex[] edges = vertices[fromNode];
            if (edges != null) {
                for (Vertex edge : edges) {
                    if (edge != null) {
                        int toNode = edge.toNode;

                        // Add the reversed edge to the temporary structure
                        for (int i = 0; i < reversedVertices[toNode].length; i++) {
                            if (reversedVertices[toNode][i] == null) {
                                reversedVertices[toNode][i] = new Vertex(
                                        fromNode, // Reverse direction
                                        edge.driveTime,
                                        edge.distance,
                                        edge.speedLimit
                                );
                                break;
                            }
                        }
                    }
                }
            }
        }

        // Replace the original vertices with the reversed ones
        for (int i = 0; i < vertices.length; i++) {
            vertices[i] = reversedVertices[i];
        }
    }
}

class Node {
    final int id;
    Node previous;
    final double latitude;
    final double longitude;
    long distanceFromStart;
    long distanceToGoal;
    boolean visited;


    public Node(int id, Node previous, double latitude, double longitude, long distanceFromStart, long distanceToGoal, boolean visited) {
        this.id = id;
        this.previous = previous;
        this.latitude = latitude;
        this.longitude = longitude;
        this.distanceFromStart = distanceFromStart;
        this.distanceToGoal = distanceToGoal;
        this.visited = visited;
    }
}

class Vertex {
    public final int toNode;
    public final int driveTime;
    public final int distance;
    public final int speedLimit;

    public Vertex(int toNode, int driveTime, int distance, int speedLimit) {
        this.toNode = toNode;
        this.driveTime = driveTime;
        this.distance = distance;
        this.speedLimit = speedLimit;
    }
}
