package pathFinder;

import java.util.HashMap;

/**
 * Implementation of a unidirectional weighted graph using the adjacency list representation. Alongside the
 * weight an additional object (storing information) can be associated with an edge. The distinction of the vertex
 * objects is hash based.
 */
public class Graph<T_Vertex, T_Info> {
    private HashMap<T_Vertex, HashMap<T_Vertex, EdgeData>> adjacencyList;

    /**
     * Constructor.
     */
    public Graph() {
        adjacencyList = new HashMap<>();
    }

    /**
     * Add the vertex to the graph.
     * @param vertex The vertex which is added.
     * @throws IllegalArgumentException if the vertex is already present in the graph.
     */
    public void addVertex(T_Vertex vertex) {
        if (adjacencyList.containsKey(vertex)) {
            throw new IllegalArgumentException("The vertex is already present in the graph.");
        }
        adjacencyList.put(vertex, new HashMap<>());
    }

    /**
     * Checks if the vertex exists in the graph.
     * @param vertex The vertex to be checked.
     * @return True if the vertex already exists in the graph.
     */
    public boolean vertexExists(T_Vertex vertex) {
        return adjacencyList.containsKey(vertex);
    }

    /**
     * Checks if the edge exists in the graph.
     * @param from Start vertex of edge.
     * @param to End vertex of edge.
     * @return True if the edge exists.
     */
    public boolean edgeExists(T_Vertex from, T_Vertex to) {
        if (!adjacencyList.containsKey(from) || !adjacencyList.containsKey(to)) {
            return false;
        }
        return adjacencyList.get(from).containsKey(to);
    }
    /**
     * Add a directed edge to the graph. The edge must not exist.
     * @param from Start vertex of edge.
     * @param to End vertex of edge.
     * @param weight Weight of the edge.
     * @param info Additional information associated with the edge.
     * @throws IllegalArgumentException if one of the vertices does not exist or the edge already exists in the graph.
     */
    public void addEdge(T_Vertex from, T_Vertex to, int weight, T_Info info) {
        if (!adjacencyList.containsKey(from) || !adjacencyList.containsKey(to)) {
            throw new IllegalArgumentException("One of the vertices does not exist in the graph.");
        }
        if (edgeExists(from, to)) {
            throw new IllegalArgumentException("The edge already exists in the graph.");
        }
        EdgeData edgeData = new EdgeData(weight, info);
        adjacencyList.get(from).put(to, edgeData);
    }

    /**
     * Method throws an exception if the edge does not exist.
     * @param from Start vertex of edge.
     * @param to End vertex of edge.
     * @throws IllegalArgumentException if one of the vertices or the edge does not exist in the graph.
     */
    private void checkEdgeExistence(T_Vertex from, T_Vertex to) {
        if (!adjacencyList.containsKey(from) || !adjacencyList.containsKey(to)) {
            throw new IllegalArgumentException("One of the vertices does not exist in the graph.");
        }
        if (!edgeExists(from, to)) {
            throw new IllegalArgumentException("The edge does not exists in the graph.");
        }
    }

    /**
     * Get the information associated with the edge.
     * @param from Start vertex of edge.
     * @param to End vertex of edge.
     * @return The information associated with the edge.
     * @throws IllegalArgumentException if one of the vertices or the edge does not exist in the graph.
     */
    public T_Info getEdgeInfo(T_Vertex from, T_Vertex to) {
        checkEdgeExistence(from, to);

        return adjacencyList.get(from).get(to).getInfo();
    }

    /**
     * Get the edge weight.
     * @param from Start vertex of edge.
     * @param to End vertex of edge.
     * @return The edge weight.
     * @throws IllegalArgumentException if one of the vertices or the edge does not exist in the graph.
     */
    public int getEdgeWeight(T_Vertex from, T_Vertex to) {
        checkEdgeExistence(from, to);

        return adjacencyList.get(from).get(to).getWeight();
    }

    /**
     * Class combining the edge weight and the object associated with the edge.
     */
    private class EdgeData {
        private int weight;
        private T_Info info;

        private EdgeData(int weight, T_Info info) {
            this.weight = weight;
            this.info = info;
        }

        private int getWeight() {
            return weight;
        }

        private T_Info getInfo() {
            return info;
        }
    }
}
