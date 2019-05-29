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
        if (map.originCells.size() == 1 && map.destCells.size() == 1) {
          return performTaskA();
        } else if (map.terrainCells.size() > 0) {
          return performTaskB();
        } else if (map.destCells.size() > 1 && map.destCells.size() > 1) {
          return performTaskC();
        } else {
          return performTaskA();
        }
    }
    
    private List<Coordinate> performTaskA() {
      Coordinate sourceCoord = map.originCells.get(0);
      Coordinate destCoord = map.destCells.get(0);
      PathCoordinate[][] pathCells = findWeightlessSingleSource(sourceCoord);
      return createShortestPathList(pathCells, sourceCoord, destCoord);
    }
    
    private List<Coordinate> performTaskB() {
      Coordinate sourceCoord = map.originCells.get(0);
      Coordinate destCoord = map.destCells.get(0);
      PathCoordinate[][] pathCells = findPathsSingleSource(sourceCoord);
      return createShortestPathList(pathCells, sourceCoord, destCoord);
    }
    
    private List<Coordinate> performTaskC() {
      int originsLength = map.originCells.size();
      int destLength = map.destCells.size();
      ArrayList<ShortestPath> paths = new ArrayList<>();
      for (int j=0; j<originsLength; j++) {
          Coordinate sourceCoord = map.originCells.get(j);
          PathCoordinate[][] pathCells = findPathsSingleSource(sourceCoord);
          for (int i=0; i<destLength; i++) {
              Coordinate destCoord = map.destCells.get(i);
              List<Coordinate> list = createShortestPathList(pathCells, sourceCoord, destCoord);
              ShortestPath shortestPath = new ShortestPath(list);
              paths.add(shortestPath);
          }
      }
      ShortestPath path = Collections.min(paths, Comparator.comparing(ShortestPath::getWeight));
      return path.coordList;
    }
    
    private List<Coordinate> performTaskD() {
      int originsLength = map.originCells.size();
      int destLength = map.destCells.size();
      ArrayList<ShortestPath> paths = new ArrayList<>();
      for (int j=0; j<originsLength; j++) {
          Coordinate sourceCoord = map.originCells.get(j);
          PathCoordinate[][] pathCells = findPathsSingleSource(sourceCoord);
          for (int i=0; i<destLength; i++) {
              Coordinate destCoord = map.destCells.get(i);
              List<Coordinate> list = createShortestPathList(pathCells, sourceCoord, destCoord);
              ShortestPath shortestPath = new ShortestPath(list);
              paths.add(shortestPath);
          }
      }
      ShortestPath path = Collections.min(paths, Comparator.comparing(ShortestPath::getWeight));
      return path.coordList;        
    }
    
    
    private PathQueue getPassableCoordinates(Coordinate sourceCoord) {
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
      return new PathQueue(pathCells,pathQueue,nPassableCoordinates);
    }
    
    private PathCoordinate[][] findWeightlessSingleSource(Coordinate sourceCoord) {
      PathQueue pathQueue = getPassableCoordinates(sourceCoord);
      HashSet<PathCoordinate> visitedCoords = new HashSet<>();
      for (int i = 0; i < pathQueue.nPassableCoordinates; i++) {
          PathCoordinate minCoord = pathQueue.pathQueue.poll();
          if (minCoord == null) break;
          visitedCoords.add(minCoord);
          // Update all adjacent coordinates:
          List<PathCoordinate> adjacentCoordinates = getAdjacentCoordinates(pathQueue.pathCells, minCoord);
          for (PathCoordinate adjacentCoord : adjacentCoordinates) {
              // Check if a new minimal distance for an adjacent coordinate can be found:
              if (!visitedCoords.contains(adjacentCoord)) {
                  int distCurr = minCoord.getDistance();
                  int newDist = distCurr + 1;
                  pathCoordinateUpdate(pathQueue.pathQueue, adjacentCoord, minCoord, newDist);
              }
          }
      }
      return pathQueue.pathCells;
    }

    /**
     * Dijkstra algorithm for the shortest paths starting from a single source. 
     * @return The shortest path from source to destination found.
     */
    private PathCoordinate[][] findPathsSingleSource(Coordinate sourceCoord) {
        PathQueue pathQueue = getPassableCoordinates(sourceCoord);
        HashSet<PathCoordinate> visitedCoords = new HashSet<>();
        // Update other Coordinates:
        for (int i = 0; i < pathQueue.nPassableCoordinates; i++) {
            PathCoordinate minCoord = pathQueue.pathQueue.poll();
            if (minCoord == null) break;
            visitedCoords.add(minCoord);
            // Update all adjacent coordinates:
            List<PathCoordinate> adjacentCoordinates = getAdjacentCoordinates(pathQueue.pathCells, minCoord);
            for (PathCoordinate adjacentCoord : adjacentCoordinates) {
                // Check if a new minimal distance for an adjacent coordinate can be found:
                if (!visitedCoords.contains(adjacentCoord)) {
                    int distCurr = minCoord.getDistance();
                    int distAdj = adjacentCoord.getDistance();
                    int terrainCost = adjacentCoord.getTerrainCost();
                    int newDist = distCurr + terrainCost;
                    if (newDist < distAdj) {
                        // New minimal distance to adjacent coordinate found:
                        pathCoordinateUpdate(pathQueue.pathQueue, adjacentCoord, minCoord, newDist);
                    }
                }
            }
        }
        return pathQueue.pathCells;
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

class PathQueue {
  PathCoordinate[][] pathCells;
  PriorityQueue<PathCoordinate> pathQueue;
  int nPassableCoordinates;
  
  public PathQueue(PathCoordinate[][] pathCells, PriorityQueue<PathCoordinate> pathQueue, int nPassableCoordinates) {
    this.pathCells = pathCells;
    this.pathQueue = pathQueue;
    this.nPassableCoordinates = nPassableCoordinates;
  }
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
        if (coordList.size() == 0) {
            // No shortest Path found
            this.pathWeight =  Integer.MAX_VALUE;
        } else {
            this.pathWeight = incrementingWeight;
        }
    }

    public int getWeight() {
        return this.pathWeight;
    }
}
