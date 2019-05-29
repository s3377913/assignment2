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
        ArrayList<ShortestPath> paths = new ArrayList<ShortestPath>();
        for (int j=0; j<originsLength; j++) {
          for (int i=0; i<destLength; i++) {
            System.out.println("i: " + i + ", j: " + j);
            Coordinate destCoord = map.destCells.get(i);
            Coordinate sourceCoord = map.originCells.get(j);
            PathCoordinate[][] pathCells = findPathsSingleSource(sourceCoord);
            List<Coordinate> list = createShortestPathList(pathCells, sourceCoord, destCoord);
            ShortestPath shortestPath = new ShortestPath(list);
            System.out.println("shortestPath weight for " + j + ", " + i + ": " + shortestPath.getWeight());
            paths.add(shortestPath);
          }
        }
        ShortestPath path = Collections.min(paths, Comparator.comparing(s -> s.getWeight()));
        return path.coordList;
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
  
  public ShortestPath(List<Coordinate> coordList) {
    this.coordList = coordList;
    setWeight();
  }
  
  public void setWeight() {
    int incrementingWeight = 0;
    for (int i=0; i< coordList.size(); i++) {
      Coordinate currCoord = coordList.get(i);
      incrementingWeight += currCoord.getTerrainCost();
    }
    this.pathWeight = incrementingWeight;
  }
  
  public int getWeight() {
    return this.pathWeight;
  }
  
}
