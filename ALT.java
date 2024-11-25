import java.io.*;
import java.util.*;

public class ALT {
    private final Graph graph;
    private final PoiType[] pointsOfInterest;
    private final long[][] poiToDistanceArr;
    private final long[][] poiFromDistanceArr;
    private final List<Integer> landmarkIds = Arrays.asList(478452, 2531818, 4439984);

    private static final String NODE_PATH = "noder_norden.txt";
    private static final String VERTICES_PATH = "kanter_norden.txt";
    private static final String POI_PATH = "interessepkt_norden.txt";
    private static final String POI_FROM_DISTANCES = "poiFromDistances.txt";
    private static final String POI_TO_DISTANCES = "poiToDistances.txt";

    public ALT() {
        graph = new Graph(7956886);
        pointsOfInterest = new PoiType[graph.size()];
        readValuesFromFiles();
        poiToDistanceArr = new long[landmarkIds.size()][graph.size()];
        poiFromDistanceArr = new long[landmarkIds.size()][graph.size()];
    }

    public Graph getGraph() {
        return this.graph;
    }

    public void setupPOIDistances() {
        findLandmarks();
        File poiFromFile = new File(POI_FROM_DISTANCES);
        File poiToFile = new File(POI_TO_DISTANCES);
        if (poiFromFile.exists() && poiToFile.exists()) {
            System.out.println("POI distances already computed");
            readPOIDistancesFromFile();
        } else {
            System.out.println("Computing POI distances...");
            computeDistancesFromLandmarks();
            System.out.println("Successfully computed POI distances");
        }
    }

    public AltPathResult runAlt(Node startNode, Node endNode) {
        int nodesChecked = 0;
        if (startNode == null || endNode == null) {
            return null;
        }

        // Reset all nodes
        for (Node node : graph.getNodes()) {
            node.distanceFromStart = Long.MAX_VALUE;
            node.visited = false;
            node.previous = null;
        }

        startNode.distanceFromStart = 0;

        PriorityQueue<Node> openSet = new PriorityQueue<>(Comparator.comparingLong(n -> n.distanceFromStart + calculateHeuristic(n, endNode)));
        openSet.add(startNode);

        while (!openSet.isEmpty()) {
            Node current = openSet.poll();
            if (current.visited) {
                continue;
            }
            current.visited = true;
            nodesChecked++;

            if (current.id == endNode.id) {
                System.out.println("Amount of nodes checked: " + nodesChecked);
                return reconstructPath(current, startNode);
            }

            for (Vertex edge : graph.findNeighbours(current)) {
                Node neighbor = graph.getNode(edge.toNode);
                if (neighbor.visited) {
                    continue;
                }
                long tentativeDistance = current.distanceFromStart + edge.driveTime;
                if (tentativeDistance < neighbor.distanceFromStart) {
                    neighbor.distanceFromStart = tentativeDistance;
                    neighbor.previous = current;
                    openSet.add(neighbor);
                }
            }
        }
        return null; // No path found
    }

    public List<Node> runDijkstrasAlgorithm(Node startNode, Node endNode) {
        int nodesChecked = 0;
        if (startNode == null || endNode == null) {
            return null;
        }

        // Initialize all nodes
        for (Node node : graph.getNodes()) {
            node.distanceFromStart = Long.MAX_VALUE;
            node.visited = false;
            node.previous = null;
        }

        startNode.distanceFromStart = 0;

        PriorityQueue<Node> queue = new PriorityQueue<>(Comparator.comparingLong(n -> n.distanceFromStart));
        queue.add(startNode);

        while (!queue.isEmpty()) {
            Node current = queue.poll();
            if (current.visited) {
                continue;
            }
            current.visited = true;
            nodesChecked++;

            if (current.id == endNode.id) {
                break;
            }

            for (Vertex edge : graph.findNeighbours(current)) {
                Node neighbor = graph.getNode(edge.toNode);
                if (neighbor.visited) {
                    continue;
                }
                long tentativeDistance = current.distanceFromStart + edge.driveTime;
                if (tentativeDistance < neighbor.distanceFromStart) {
                    neighbor.distanceFromStart = tentativeDistance;
                    neighbor.previous = current;
                    queue.add(neighbor);
                }
            }
        }

        List<Node> path = new ArrayList<>();
        Node current = endNode;
        while (current != null && current != startNode) {
            path.add(current);
            current = current.previous;
        }
        if (current == null) {
            return null; // No path
        }
        path.add(startNode);
        Collections.reverse(path);
        System.out.println("Amount of nodes checked: " + nodesChecked);
        return path;
    }

    // Helper methods and classes

    private void readValuesFromFiles() {
        System.out.println("Loading nodes and vertices from file...");
        readNodesFromFile(graph);
        readVerticesFromFile(graph);
        System.out.println("Loading points of interest from file...");
        readPointsOfInterestFromFile();
        System.out.println("Successfully loaded nodes and vertices from file");
    }

    private void findLandmarks() {
        // Landmarks are predefined in landmarkIds
        // This method can be extended to dynamically select landmarks if needed
    }

    private void computeDistancesFromLandmarks() {
        int numLandmarks = landmarkIds.size();
        int numNodes = graph.size();

        // Compute distances from landmarks to all nodes (poiToDistanceArr)
        for (int i = 0; i < numLandmarks; i++) {
            int landmarkId = landmarkIds.get(i);
            Node landmarkNode = graph.getNode(landmarkId);
            System.out.println("Computing distances from landmark " + landmarkId + " to all nodes...");
            long[] distances = computeDistancesFromLandmark(landmarkNode);
            poiToDistanceArr[i] = distances;
        }

        // Reverse the graph to compute distances from all nodes to landmarks
        graph.reverse();

        // Compute distances from landmarks to all nodes in reversed graph (poiFromDistanceArr)
        for (int i = 0; i < numLandmarks; i++) {
            int landmarkId = landmarkIds.get(i);
            Node landmarkNode = graph.getNode(landmarkId);
            System.out.println("Computing distances from all nodes to landmark " + landmarkId + "...");
            long[] distances = computeDistancesFromLandmark(landmarkNode);
            poiFromDistanceArr[i] = distances;
        }

        // Reverse the graph back to its original state
        graph.reverse();

        // Write distances to files
        try (BufferedWriter writer = new BufferedWriter(new FileWriter(POI_TO_DISTANCES))) {
            for (int i = 0; i < numNodes; i++) {
                StringBuilder line = new StringBuilder();
                for (int j = 0; j < numLandmarks; j++) {
                    line.append(poiToDistanceArr[j][i]).append(" ");
                }
                writer.write(line.toString().trim());
                writer.newLine();
            }
        } catch (IOException e) {
            System.out.println("Error writing POI to distances: " + e.getMessage());
        }

        try (BufferedWriter writer = new BufferedWriter(new FileWriter(POI_FROM_DISTANCES))) {
            for (int i = 0; i < numNodes; i++) {
                StringBuilder line = new StringBuilder();
                for (int j = 0; j < numLandmarks; j++) {
                    line.append(poiFromDistanceArr[j][i]).append(" ");
                }
                writer.write(line.toString().trim());
                writer.newLine();
            }
        } catch (IOException e) {
            System.out.println("Error writing POI from distances: " + e.getMessage());
        }
    }

    private long[] computeDistancesFromLandmark(Node landmark) {
        int numNodes = graph.size();
        long[] distances = new long[numNodes];
        boolean[] visited = new boolean[numNodes];

        Arrays.fill(distances, Long.MAX_VALUE);
        distances[landmark.id] = 0;

        PriorityQueue<NodeDistance> queue = new PriorityQueue<>(Comparator.comparingLong(nd -> nd.distance));
        queue.add(new NodeDistance(landmark.id, 0));

        while (!queue.isEmpty()) {
            NodeDistance current = queue.poll();
            if (visited[current.nodeId]) {
                continue;
            }
            visited[current.nodeId] = true;
            List<Vertex> neighbors = graph.findNeighbours(graph.getNode(current.nodeId));
            for (Vertex edge : neighbors) {
                int neighborId = edge.toNode;
                if (visited[neighborId]) {
                    continue;
                }
                long newDist = distances[current.nodeId] + edge.driveTime;
                if (newDist < distances[neighborId]) {
                    distances[neighborId] = newDist;
                    queue.add(new NodeDistance(neighborId, newDist));
                }
            }
        }
        return distances;
    }

    private static class NodeDistance {
        int nodeId;
        long distance;

        public NodeDistance(int nodeId, long distance) {
            this.nodeId = nodeId;
            this.distance = distance;
        }
    }

    private void readPOIDistancesFromFile() {
        int numLandmarks = landmarkIds.size();
        int numNodes = graph.size();

        try (BufferedReader reader = new BufferedReader(new FileReader(POI_TO_DISTANCES))) {
            for (int i = 0; i < numNodes; i++) {
                String line = reader.readLine();
                String[] parts = line.trim().split("\\s+");
                for (int j = 0; j < numLandmarks; j++) {
                    poiToDistanceArr[j][i] = Long.parseLong(parts[j]);
                }
            }
        } catch (IOException e) {
            System.out.println("Error reading POI to distances: " + e.getMessage());
        }

        try (BufferedReader reader = new BufferedReader(new FileReader(POI_FROM_DISTANCES))) {
            for (int i = 0; i < numNodes; i++) {
                String line = reader.readLine();
                String[] parts = line.trim().split("\\s+");
                for (int j = 0; j < numLandmarks; j++) {
                    poiFromDistanceArr[j][i] = Long.parseLong(parts[j]);
                }
            }
        } catch (IOException e) {
            System.out.println("Error reading POI from distances: " + e.getMessage());
        }
    }

    private long calculateHeuristic(Node current, Node end) {
        long maxEstimate = Long.MIN_VALUE; // Initialize to the lowest possible value
        int numLandmarks = landmarkIds.size();

        for (int i = 0; i < numLandmarks; i++) {
            long distLandmarkToEnd = poiToDistanceArr[i][end.id];
            long distLandmarkToCurrent = poiToDistanceArr[i][current.id];
            long distCurrentToLandmark = poiFromDistanceArr[i][current.id];
            long distEndToLandmark = poiFromDistanceArr[i][end.id];

            // Check for valid distances
            if (distLandmarkToEnd != Long.MAX_VALUE && distLandmarkToCurrent != Long.MAX_VALUE) {
                long estimate = distLandmarkToEnd - distLandmarkToCurrent;
                maxEstimate = Math.max(maxEstimate, estimate);
            }

            if (distCurrentToLandmark != Long.MAX_VALUE && distEndToLandmark != Long.MAX_VALUE) {
                long estimate = distCurrentToLandmark - distEndToLandmark;
                maxEstimate = Math.max(maxEstimate, estimate);
            }
        }

        // If no valid estimates were found, return zero
        if (maxEstimate == Long.MIN_VALUE) {
            return 0;
        }

        return maxEstimate;
    }

    private AltPathResult reconstructPath(Node endNode, Node startNode) {
        List<Node> path = new ArrayList<>();
        Node current = endNode;
        while (current != null && current != startNode) {
            path.add(current);
            current = current.previous;
        }
        if (current == null) {
            return null; // No path
        }
        path.add(startNode);
        Collections.reverse(path);
        long totalDistance = endNode.distanceFromStart;
        return new AltPathResult(totalDistance, path);
    }

    public static class AltPathResult {
        public final long totalDistance;
        public final List<Node> path;

        public AltPathResult(long totalDistance, List<Node> path) {
            this.totalDistance = totalDistance;
            this.path = path;
        }
    }

    // Reading files methods

    private static void readNodesFromFile(Graph graph) {
        try (BufferedReader reader = new BufferedReader(new FileReader(NODE_PATH))) {
            String line;
            reader.readLine(); // Skip first line
            while ((line = reader.readLine()) != null) {
                String[] parts = line.trim().split("\\s+");
                int id = Integer.parseInt(parts[0]);
                double latitude = Double.parseDouble(parts[1]);
                double longitude = Double.parseDouble(parts[2]);
                graph.addNode(id, new Node(id, latitude, longitude));
            }
        } catch (IOException e) {
            System.out.println("Error reading nodes from file: " + e.getMessage());
        }
    }

    private static void readVerticesFromFile(Graph graph) {
        try (BufferedReader reader = new BufferedReader(new FileReader(VERTICES_PATH))) {
            String line;
            reader.readLine(); // Skip first line
            while ((line = reader.readLine()) != null) {
                String[] parts = line.trim().split("\\s+");
                int fromNode = Integer.parseInt(parts[0]);
                int toNode = Integer.parseInt(parts[1]);
                int driveTime = Integer.parseInt(parts[2]);
                int distance = Integer.parseInt(parts[3]);
                int speedLimit = Integer.parseInt(parts[4]);
                graph.addVertex(fromNode, new Vertex(toNode, driveTime, distance, speedLimit));
            }
        } catch (IOException e) {
            System.out.println("Error reading vertices from file: " + e.getMessage());
        }
    }

    private void readPointsOfInterestFromFile() {
        try (BufferedReader reader = new BufferedReader(new FileReader(POI_PATH))) {
            String line;
            reader.readLine(); // Skip first line
            while ((line = reader.readLine()) != null) {
                String[] parts = line.trim().split("\\s+");
                int id = Integer.parseInt(parts[0]);
                PoiType poiType = new PoiType(Integer.parseInt(parts[1]), parts[2]);
                pointsOfInterest[id] = poiType;
            }
        } catch (IOException e) {
            System.out.println("Error reading points of interest from file: " + e.getMessage());
        }
    }

    private static class PoiType {
        int code;
        String name;

        public PoiType(int code, String name) {
            this.code = code;
            this.name = name;
        }
    }
}

class Graph {
    private final Node[] nodes;
    private List<Vertex>[] adjacencyList;

    @SuppressWarnings("unchecked")
    public Graph(int nodeAmount) {
        nodes = new Node[nodeAmount];
        adjacencyList = new ArrayList[nodeAmount];
        for (int i = 0; i < nodeAmount; i++) {
            adjacencyList[i] = new ArrayList<>();
        }
    }

    public Node getNode(int id) {
        return nodes[id];
    }

    public Node[] getNodes() {
        return nodes;
    }

    public void addNode(int id, Node node) {
        nodes[id] = node;
    }

    public void addVertex(int fromNode, Vertex vertex) {
        adjacencyList[fromNode].add(vertex);
    }

    public List<Vertex> findNeighbours(Node node) {
        return adjacencyList[node.id];
    }

    public int size() {
        return nodes.length;
    }

    public void reverse() {
        List<Vertex>[] reversedAdjacencyList = new ArrayList[nodes.length];
        for (int i = 0; i < nodes.length; i++) {
            reversedAdjacencyList[i] = new ArrayList<>();
        }

        for (int fromNode = 0; fromNode < adjacencyList.length; fromNode++) {
            for (Vertex edge : adjacencyList[fromNode]) {
                int toNode = edge.toNode;
                reversedAdjacencyList[toNode].add(new Vertex(
                        fromNode,
                        edge.driveTime,
                        edge.distance,
                        edge.speedLimit
                ));
            }
        }

        adjacencyList = reversedAdjacencyList;
    }
}

class Node {
    final int id;
    final double latitude;
    final double longitude;
    Node previous;
    long distanceFromStart;
    boolean visited;

    public Node(int id, double latitude, double longitude) {
        this.id = id;
        this.latitude = latitude;
        this.longitude = longitude;
    }
}

class Vertex {
    final int toNode;
    final int driveTime;
    final int distance;
    final int speedLimit;

    public Vertex(int toNode, int driveTime, int distance, int speedLimit) {
        this.toNode = toNode;
        this.driveTime = driveTime;
        this.distance = distance;
        this.speedLimit = speedLimit;
    }
}