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
        System.out.println("originsLength: " + originsLength);
        System.out.println("destLength: " + destLength);
        ArrayList<ShortestPath> paths = new ArrayList<>();
        for (int j=0; j<originsLength; j++) {
            Coordinate sourceCoord = map.originCells.get(j);
            PathCoordinate[][] pathCells = findPathsSingleSource(sourceCoord);
            for (int i=0; i<destLength; i++) {
                System.out.println("i: " + i + ", j: " + j);
                List<Coordinate> list;
                Coordinate destCoord = map.destCells.get(i);
                if (map.waypointCells.size() == 0) {
                    list = createShortestPathList(pathCells, sourceCoord, destCoord);
                } else {
                    list = createShortestPathAlongWaypoints(sourceCoord, destCoord, map.waypointCells);
                }
                ShortestPath shortestPath = new ShortestPath(list);
                System.out.println("shortestPath weight for " + j + ", " + i + ": " + shortestPath.getWeight());
                paths.add(shortestPath);
            }
        }
        ShortestPath path = Collections.min(paths, Comparator.comparing(ShortestPath::getWeight));
        return path.coordList;
    }

    // TODO: Method currently assumes that source, the waypoints, and the destination are different coordinates!
    /**
     * Create the shortest path from source to destination including all waypoints. Finding a solution is implemented
     * as a dynamic program. First, we create a graph connecting each source/waypoint/destination cell with each other.
     * The weight of each edge is the distance between the two vertex coordinates. The Graph class is used to create
     * the graph representation. After we created this fully connected bidirectional graph a modified traveling
     * salesman problem results. This traveling salesman problem is solved with the dynamic programming approach
     * suggested by Held and Karp.
     * @param sourceCoord The source coordinate of the path.
     * @param destCoord The destination coordinate of the path.
     * @param waypointCells All waypoint cells which should be visited.
     * @return The shortest path from source to destination including all waypoints.
     */
    private List<Coordinate> createShortestPathAlongWaypoints(Coordinate sourceCoord,
                                                              Coordinate destCoord, List<Coordinate> waypointCells) {
        List<Coordinate> allCoords = new ArrayList<>(waypointCells);
        allCoords.add(sourceCoord);
        allCoords.add(destCoord);
        Graph<Coordinate, ShortestPath> distanceGraph = new Graph<>();
        // Create the graph representation between each source/waypoint/edge:
        for (Coordinate startCoord : allCoords) {
            PathCoordinate[][] pathCells = findPathsSingleSource(startCoord);
            // Adding of startCoord to distance Graph is done by innermost loop too (as startCoord and coordB iterate over same coords)
            for (Coordinate endCoord : allCoords) {
                if (!distanceGraph.vertexExists(endCoord)) {
                    // Add missing coordinate to graph as new vertex:
                    distanceGraph.addVertex(endCoord);
                }
                // Exclude direct source <-> destination connections:
                boolean sourceToDest = startCoord.equals(sourceCoord) && endCoord.equals(destCoord);
                boolean destToSource = startCoord.equals(destCoord) && endCoord.equals(sourceCoord);
                if (sourceToDest || destToSource) {
                    // Don't add connections from source to destination or destination to source because at least one
                    // waypoint has to be visited. This means the solution is not a direct connection between source
                    // and destination.
                    continue;
                } // End of source <-> destination exclusion
                // Add the edge between startCoord and endCoord if it does not exist already and startCoord != endCoord:
                if (!startCoord.equals(endCoord) && !distanceGraph.edgeExists(startCoord, endCoord)) {
                    List<Coordinate> path = createShortestPathList(pathCells, startCoord, endCoord);
                    ShortestPath shortestPath = new ShortestPath(path);
                    distanceGraph.addEdge(startCoord, endCoord, shortestPath.getWeight(), shortestPath);
                }
            }
        } // Creation of shortest distances graph complete

        ShortestPath result = searchForShortestPathAlongWaypoints(sourceCoord, destCoord, new HashSet<>(waypointCells), distanceGraph);
        return result.coordList;
    }

    /**
     * Dynamic programming approach for the search of the shortest path from a source to a destination including the
     * waypoints. This method works recursively and the problem size is reduced by one vertex each recursion call.
     *
     * @param source The current source coordinate.
     * @param dest The final destination coordinate.
     * @param waypoints The original set is not changed. It is worked on a shallow copy of the set.
     * @param graph The graph containing the information about the shortest paths between the vertices.
     * @return The shortest path from source to destination including all waypoints.
     */
    private ShortestPath searchForShortestPathAlongWaypoints(Coordinate source, Coordinate dest,
                                                                 HashSet<Coordinate> waypoints,
                                                                 Graph<Coordinate, ShortestPath> graph) {
        // Stop condition
        if (waypoints.size() == 0) {
            return graph.getEdgeInfo(source, dest);
        }
        // Recursive creation of candidates:
        List<ShortestPath> canidates = new ArrayList<>();
        for (Coordinate waypoint : waypoints) {
            HashSet<Coordinate> newWaypoints = new HashSet<>(waypoints);
            newWaypoints.remove(waypoint);
            ShortestPath fromWaypoint = searchForShortestPathAlongWaypoints(waypoint, dest, newWaypoints, graph);
            ShortestPath toWaypoint = graph.getEdgeInfo(source, waypoint);
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
     * Dijkstra algorithm for the shortest paths starting from a single source. 
     * @return The shortest path from source to destination found.
     */
    private PathCoordinate[][] findPathsSingleSource(Coordinate sourceCoord) {
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
        // Update source:
        PathCoordinate source = pathCells[sourceCoord.getRow()][sourceCoord.getColumn()];
        pathCoordinateUpdate(pathQueue, source, null, 0);

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
     * Create a list of coordinates to visit for a shortest path from source to destination.
     * @param pathCells The path cells of the map after running the Dijkstra algorithm.
     * @return The coordinates to visit for a shortest path from source to destination.
     */
    private List<Coordinate> createShortestPathList(PathCoordinate[][] pathCells, Coordinate source, Coordinate dest) {
        List<Coordinate> path = new ArrayList<>();
        PathCoordinate currLast = pathCells[dest.getRow()][dest.getColumn()];
        while (!currLast.getCoordinate().equals(source)) {
            path.add(currLast.getCoordinate());
            Coordinate prev = currLast.getPrevious();
            if (prev == null) {
                // No Connection between source and destination found
                return new ArrayList<>();
            }
            currLast = pathCells[prev.getRow()][prev.getColumn()];
        }
        // Add the source to the list:
        path.add(source);
        Collections.reverse(path);
        return path;
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
     * @param distance The new total distance to the source coordinate
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

class ShortestPath {
    List<Coordinate> coordList;
    int pathWeight;
    boolean validPath;

    /**
     * Constructor of the shortest path. The shortest path will not be empty and the weight will be set to
     * Integer.MAX_VALUE if the coordList is empty.
     * @param coordList The list of coordinates which the ShortestPath instance represents.
     */
    public ShortestPath(List<Coordinate> coordList) {
        this.coordList = coordList;
        if (coordList.isEmpty()) {
            this.pathWeight = Integer.MAX_VALUE;
            this.validPath = false;
        } else {
            setWeight();
            validPath = true;
        }
    }

    /**
     * A dummy path which is not valid will be created. The weight is Integer.MAX_VALUE and the coordList is empty.
     */
    public ShortestPath() {
        this.coordList = new ArrayList<>();
        this.pathWeight = Integer.MAX_VALUE;
        this.validPath = false;
    }
    /**
     * Constructor which constructs a shortest path which is a concatenation of the from and to paths. It checks if
     * a valid path results from the concatenation (i.e. the last coordinate of the start path is the same as the first
     * coordinate of the end path). Otherwise it sets the validPath flag to false, the coordList will be empty and the
     * weight will be Integer.MAX_VALUE.
     * @param start Path starts here.
     * @param end Path ends here.
     */
    public ShortestPath(ShortestPath start, ShortestPath end) {
        int startLastIdx = start.coordList.size() - 1;
        Coordinate lastStart = start.coordList.get(startLastIdx);
        Coordinate firstEnd = end.coordList.get(0);
        if (!lastStart.equals(firstEnd)) {
            // Path is not valid
            coordList = new ArrayList<>();
            validPath = false;
            pathWeight = Integer.MAX_VALUE;
        } else {
            this.coordList = new ArrayList<>(start.coordList);
            // Do not add point connecting the paths twice:
            Coordinate connection = start.coordList.get(startLastIdx);
            this.coordList.remove(startLastIdx);

            this.coordList.addAll(end.coordList);
            this.pathWeight = start.pathWeight + end.pathWeight - connection.getTerrainCost();
            this.validPath = true;
        }
    }

    public void setWeight() {
        int incrementingWeight = 0;
        for (int i=0; i< coordList.size(); i++) {
            Coordinate currCoord = coordList.get(i);
            incrementingWeight += currCoord.getTerrainCost();
        }
        if (coordList.size() == 0) {
            // No shortest Path found
            this.pathWeight =  Integer.MAX_VALUE;
        } else {
            this.pathWeight = incrementingWeight;
        }
    }

    /**
     * It is possible that a path is not valid if the constructor concatenating
     * two paths is used to create an instance and the last element of the start path and the first element of the
     * end path are not equal.
     * @return True if the path is valid.
     */
    public boolean isValidPath() {
        return validPath;
    }

    public int getWeight() {
        return this.pathWeight;
    }
}
