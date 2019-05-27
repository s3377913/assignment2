package pathFinder;

import java.util.HashMap;

/**
 * Implementation of a bidirectional weighted graph using the adjacency list representation. Alongside the
 * weight an additional object (storing information) can be associated with an edge. The distinction of the vertex
 * objects is hash based.
 */
public class BiGraph<T_Vertex, T_Info> {
    private HashMap<T_Vertex, HashMap<T_Vertex, EdgeData>> adjacencyList;

    /**
     * Constructor.
     */
    public BiGraph() {
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
     * Add a bidirectional edge to the graph. The edge must not exist.
     * @param vertA One endpoint of the edge.
     * @param vertB Other endpoint of the edge.
     * @param weight Weight of the edge.
     * @param info Additional information associated with the edge.
     * @throws IllegalArgumentException if one of the vertices does not exist or the edge already exists in the graph.
     */
    public void addBiEdge(T_Vertex vertA, T_Vertex vertB, int weight, T_Info info) {
        if (!adjacencyList.containsKey(vertA) || !adjacencyList.containsKey(vertB)) {
            throw new IllegalArgumentException("One of the vertices does not exist in the graph.");
        }
        if (adjacencyList.get(vertA).get(vertB) != null) {
            throw new IllegalArgumentException("The edge already exists in the graph.");
        }
        EdgeData edgeData = new EdgeData(weight, info);
        adjacencyList.get(vertA).put(vertB, edgeData);
        adjacencyList.get(vertB).put(vertA, edgeData);
    }

    /**
     * Method throws an exception if the edge does not exist.
     * @param vertA One endpoint of the edge.
     * @param vertB Other endpoint of the edge.
     * @throws IllegalArgumentException if one of the vertices or the edge does not exist in the graph.
     */
    private void checkEdgeExistence(T_Vertex vertA, T_Vertex vertB) {
        if (!adjacencyList.containsKey(vertA) || !adjacencyList.containsKey(vertB)) {
            throw new IllegalArgumentException("One of the vertices does not exist in the graph.");
        }
        if (adjacencyList.get(vertA).get(vertB) == null) {
            throw new IllegalArgumentException("The edge does not exists in the graph.");
        }
    }

    /**
     * Get the information associated with the edge.
     * @param vertA One endpoint of the edge.
     * @param vertB Other endpoint of the edge.
     * @return The information associated with the edge.
     * @throws IllegalArgumentException if one of the vertices or the edge does not exist in the graph.
     */
    public T_Info getEdgeInfo(T_Vertex vertA, T_Vertex vertB) {
        checkEdgeExistence(vertA, vertB);

        return adjacencyList.get(vertA).get(vertB).getInfo();
    }

    /**
     * Get the edge weight.
     * @param vertA One endpoint of the edge.
     * @param vertB Other endpoint of the edge.
     * @return The edge weight.
     * @throws IllegalArgumentException if one of the vertices or the edge does not exist in the graph.
     */
    public int getEdgeWeight(T_Vertex vertA, T_Vertex vertB) {
        checkEdgeExistence(vertA, vertB);

        return adjacencyList.get(vertA).get(vertB).getWeight();
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
