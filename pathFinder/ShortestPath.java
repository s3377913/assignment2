package pathFinder;

import map.Coordinate;

import java.util.ArrayList;
import java.util.List;

/**
 * This class represents paths (valid or not) which were found. This class just stores the original list of coordinates
 * representing the shortest path and additionally includes the weight of the path to compare it with other paths.
 */
class ShortestPath {
    List<Coordinate> coordList;
    private int pathWeight;
    private boolean validPath;

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