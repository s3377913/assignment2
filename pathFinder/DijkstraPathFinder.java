package pathFinder;

import map.Coordinate;
import map.PathMap;

import java.util.*;

public class DijkstraPathFinder implements PathFinder
{
    private PathMap map;

    public DijkstraPathFinder(PathMap map) {
        this.map = map;
    } // end of DijkstraPathFinder()

    @Override
    public List<Coordinate> findPath() {
        int originsLength = map.originCells.size();
        int destLength = map.destCells.size();
        // Collection of the shortest path between each origin and destination coordinate:
        // If waypoints are included in the map the waypoints are included in each ShortestPath too.
        ArrayList<ShortestPath> paths = new ArrayList<>();
        for (int j=0; j<originsLength; j++) { // Iteration over every origin
            // For each origin calculate the Dijkstra PathCoordinate matrix only once
            // In this matrix the information about the shortest path to every other coordinate is stored
            Coordinate originCoord = map.originCells.get(j);
            PathCoordinate[][] pathCells = findPathsSingleOrigin(originCoord);
            for (int i=0; i<destLength; i++) { // Iteration over every destination
                ShortestPath shortestPath;
                Coordinate destCoord = map.destCells.get(i);
                if (map.waypointCells.size() == 0) { // No Waypoints have to be included, solve Task A/B:
                    // (Task C is solved through the iterations over the origins and destinations above)
                    shortestPath = createShortestPathList(pathCells, originCoord, destCoord);
                } else { // Waypoints have to be included, solve Task D:
                    shortestPath = createShortestPathAlongWaypoints(originCoord, destCoord, map.waypointCells);
                }
                paths.add(shortestPath);
            }
        }
        ShortestPath path = Collections.min(paths, Comparator.comparing(ShortestPath::getWeight));
        return path.getCoordList();
    }

    /**
     * Create the shortest path from origin to destination including all waypoints. Finding a solution is implemented
     * as a dynamic program. First, we create a graph connecting each origin/waypoint/destination cell with each other.
     * The weight of each edge is the distance between the two vertex coordinates. The Graph class is used to create
     * the graph representation. After we created this fully connected bidirectional graph a modified traveling
     * salesman problem results. This traveling salesman problem is solved with the dynamic programming approach
     * suggested by Held and Karp.
     * @param origin The origin coordinate of the path.
     * @param destCoord The destination coordinate of the path.
     * @param waypointCells All waypoint cells which should be visited.
     * @return The shortest path from origin to destination including all waypoints.
     */
    private ShortestPath createShortestPathAlongWaypoints(Coordinate origin,
                                                              Coordinate destCoord, List<Coordinate> waypointCells) {
        List<Coordinate> allLandmarks = new ArrayList<>(waypointCells);
        allLandmarks.add(origin);
        allLandmarks.add(destCoord);
        Graph<Coordinate, ShortestPath> distanceGraph = new Graph<>();
        // Create the graph representation between each origin/waypoint/edge:
        for (Coordinate startCoord : allLandmarks) {
            PathCoordinate[][] pathCells = findPathsSingleOrigin(startCoord);
            // Adding of startCoord to distance Graph is done by innermost loop too (as startCoord and coordB iterate over same coords)
            for (Coordinate endCoord : allLandmarks) {
                if (!distanceGraph.vertexExists(endCoord)) {
                    // Add missing coordinate to graph as new vertex:
                    distanceGraph.addVertex(endCoord);
                }
                // Exclude direct origin <-> destination connections:
                boolean originToDest = startCoord.equals(origin) && endCoord.equals(destCoord);
                boolean destToOrigin = startCoord.equals(destCoord) && endCoord.equals(origin);
                if (originToDest || destToOrigin) {
                    // Don't add connections from origin to destination or destination to origin because at least one
                    // waypoint has to be visited. This means the solution is not a direct connection between origin
                    // and destination.
                    continue;
                } // End of origin <-> destination exclusion
                // Add the edge between startCoord and endCoord if it does not exist already and startCoord != endCoord:
                if (!startCoord.equals(endCoord) && !distanceGraph.edgeExists(startCoord, endCoord)) {
                    ShortestPath shortestPath = createShortestPathList(pathCells, startCoord, endCoord);
                    distanceGraph.addEdge(startCoord, endCoord, shortestPath.getWeight(), shortestPath);
                }
            }
        } // Creation of shortest distances graph complete

        return searchForShortestPathAlongWaypoints(origin, destCoord, new HashSet<>(waypointCells), distanceGraph);
    }

    /**
     * Dynamic programming approach for the search of the shortest path from a origin to a destination including the
     * waypoints. This method works recursively and the problem size is reduced by one vertex each recursion call.
     *
     * @param origin The current origin coordinate.
     * @param dest The final destination coordinate.
     * @param waypoints The original set is not changed. It is worked on a shallow copy of the set.
     * @param graph The graph containing the information about the shortest paths between the vertices.
     * @return The shortest path from origin to destination including all waypoints.
     */
    private ShortestPath searchForShortestPathAlongWaypoints(Coordinate origin, Coordinate dest,
                                                                 HashSet<Coordinate> waypoints,
                                                                 Graph<Coordinate, ShortestPath> graph) {
        // Stop condition
        if (waypoints.size() == 0) {
            return graph.getEdgeInfo(origin, dest);
        }
        // Recursive creation of candidates:
        List<ShortestPath> canidates = new ArrayList<>();
        for (Coordinate waypoint : waypoints) {
            HashSet<Coordinate> newWaypoints = new HashSet<>(waypoints);
            newWaypoints.remove(waypoint);
            ShortestPath fromWaypoint = searchForShortestPathAlongWaypoints(waypoint, dest, newWaypoints, graph);
            ShortestPath toWaypoint = graph.getEdgeInfo(origin, waypoint);
            ShortestPath newCandidate;
            if (fromWaypoint.isValidPath() && toWaypoint.isValidPath()) {
                // Both paths alone are valid, try to concatenate them:
                newCandidate = new ShortestPath(toWaypoint, fromWaypoint);
            } else {
                // Create invalid dummy:
                newCandidate = new ShortestPath();
            }
            if (newCandidate.isValidPath()) {
                canidates.add(newCandidate);
            }
        }
        if (canidates.size() == 0) {
            // No valid paths were found, create a shortest path dummy with a weight of Integer.MAX_VALUE
            return new ShortestPath();
        } else {
            // Comparison of Candidates:
            return Collections.min(canidates, Comparator.comparing(ShortestPath::getWeight));
        }
    }
    /**
     * Dijkstra algorithm for the shortest paths starting from a single origin.
     * @return The shortest path from origin to destination found.
     */
    private PathCoordinate[][] findPathsSingleOrigin(Coordinate originCoord) {
        HashSet<PathCoordinate> visitedCoords = new HashSet<>();
        int nPassableCoordinates = 0;
        // Create path coordinates and priority queue:
        PriorityQueue<PathCoordinate> pathQueue = new PriorityQueue<>();
        PathCoordinate[][] pathCells = new PathCoordinate[map.sizeR][map.sizeC];
        for (int i = 0; i < map.sizeR; i++) {
            for (int j = 0; j < map.sizeC; j++) {
                PathCoordinate coord = new PathCoordinate(map.cells[i][j]);
                pathCells[i][j] = coord;
                if (map.isPassable(i, j)) {
                    pathQueue.offer(coord);
                    nPassableCoordinates++;
                }
            }
        }
        // Update origin:
        PathCoordinate origin = pathCells[originCoord.getRow()][originCoord.getColumn()];
        pathCoordinateUpdate(pathQueue, origin, null, 0);

        // Update other Coordinates:
        for (int i = 0; i < nPassableCoordinates; i++) {
            PathCoordinate minCoord = pathQueue.poll();
            if (minCoord == null) break;
            visitedCoords.add(minCoord);
            // Update all adjacent coordinates:
            List<PathCoordinate> adjacentCoordinates = getAdjacentCoordinates(pathCells, minCoord);
            for (PathCoordinate adjacentCoord : adjacentCoordinates) {
                // Check if a new minimal distance for an adjacent coordinate can be found:
                if (!visitedCoords.contains(adjacentCoord)) {
                    int distCurr = minCoord.getDistance();
                    int distAdj = adjacentCoord.getDistance();
                    int terrainCost = adjacentCoord.getTerrainCost();
                    int newDist = distCurr + terrainCost;
                    if (newDist < distAdj) {
                        // New minimal distance to adjacent coordinate found:
                        pathCoordinateUpdate(pathQueue, adjacentCoord, minCoord, newDist);
                    }
                }
            }
        }
        return pathCells;
    }

    /**
     * Create a list of coordinates to visit for a shortest path from origin to destination.
     * @param pathCells The path cells of the map after running the Dijkstra algorithm.
     * @return The coordinates to visit for a shortest path from origin to destination.
     */
    private ShortestPath createShortestPathList(PathCoordinate[][] pathCells, Coordinate origin, Coordinate dest) {
        List<Coordinate> path = new ArrayList<>();
        PathCoordinate currLast = pathCells[dest.getRow()][dest.getColumn()];
        int pathWeight = currLast.getDistance();
        while (!currLast.getCoordinate().equals(origin)) {
            path.add(currLast.getCoordinate());
            Coordinate prev = currLast.getPrevious();
            if (prev == null) {
                // No Connection between origin and destination found, return invalid path
                return new ShortestPath();
            }
            currLast = pathCells[prev.getRow()][prev.getColumn()];
        }
        // Add the origin to the list:
        path.add(origin);
        Collections.reverse(path);
        return new ShortestPath(path, pathWeight);
    }

    /**
     * Get all path coordinates adjacent to this coordinate which are passable.
     * @param coord The path coordinate for which to get the ajacent ones
     * @return The adjacent passable path coordinates
     */
    private List<PathCoordinate> getAdjacentCoordinates(PathCoordinate[][] pathCells, PathCoordinate coord) {
        List<PathCoordinate> adjacentCoordinates = new ArrayList<>();
        // Check coordinates above and below
        for (int i = 0; i < 2; i++) {
            // Offsets are either +1 or -1:
            int rOffset = 2 * i - 1;
            int row = coord.getCoordinate().getRow() + rOffset;
            int col = coord.getCoordinate().getColumn();
            if (map.isIn(row, col) && map.isPassable(row, col)) {
                adjacentCoordinates.add(pathCells[row][col]);
            }
        }
        // Check coordinates left and right
        for (int i = 0; i < 2; i++) {
            // Offsets are either +1 or -1:
            int cOffset = 2*i - 1;
            int row = coord.getCoordinate().getRow();
            int col = coord.getCoordinate().getColumn() + cOffset;
            if (map.isIn(row, col) && map.isPassable(row, col)) {
                adjacentCoordinates.add(pathCells[row][col]);
            }
        }
        return adjacentCoordinates;
    }

    /**
     * Update the path coordinate and reinsert into the priority queue.
     * @param coord The path coordinate to be updated
     * @param previous The new previous coordinate of this path coordinate
     * @param distance The new total distance to the origin coordinate
     */
    private void pathCoordinateUpdate(PriorityQueue<PathCoordinate> pathQueue,
                                      PathCoordinate coord, PathCoordinate previous, int distance) {
        pathQueue.remove(coord);
        if (previous != null)
            coord.setPrevious(previous.getCoordinate());
        else
            coord.setPrevious(null);
        coord.setDistance(distance);
        pathQueue.offer(coord);
    }

    @Override
    public int coordinatesExplored() {
        // TODO: Implement (optional)

        // placeholder
        return 0;
    } // end of cellsExplored()
}
