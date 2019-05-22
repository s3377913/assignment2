package pathFinder;

import map.Coordinate;

/**
 * Class of a coordinate used for the Dijkstra algorithm.
 * The coordinate stores additional information:
 * it's penultimate coordinate and the total distance from
 * the source coordinate.
 *
 * This class implements the Comparable interface and can
 * be used with a priority queue.
 *
 * Note: this class has a natural ordering that is inconsistent with equals.
 */
public class PathCoordinate implements Comparable<PathCoordinate>{
    private map.Coordinate penultimate;
    private map.Coordinate coordinate;
    private int distance;

    /**
     * Construct the path coordinate with the pneultimate as null and the distance as infinity (Integer.MAX_VALUE).
     * @param coordinate Coordinate which this class represents
     */
    public PathCoordinate(map.Coordinate coordinate) {
        this.coordinate = coordinate;
        this.penultimate = null;
        this.distance = Integer.MAX_VALUE;
    }

    /**
     * Construct the path coordinate with all instance variables initialized.
     * @param penultimate The penultimate to this coordinate (null if it is the source coordinate)
     * @param coordinate Coordinate which this class represents
     * @param distance Distance from the source coordinate (
     */
    public PathCoordinate(map.Coordinate penultimate, map.Coordinate coordinate, int distance) {
        this.penultimate = penultimate;
        this.coordinate = coordinate;
        this.distance = distance;
    }

    // Interface implementation
    @Override
    public int compareTo(PathCoordinate other) {
        return this.distance - other.getDistance();
    }

    // Getters and setters:

    public void setPenultimate(Coordinate penultimate) {
        this.penultimate = penultimate;
    }

    public void setDistance(int distance) {
        this.distance = distance;
    }

    public Coordinate getCoordinate() {
        return coordinate;
    }

    public Coordinate getPenultimate() {
        return penultimate;
    }

    public int getDistance() {
        return distance;
    }

    public int getTerrainCost() {
        return coordinate.getTerrainCost();
    }

    public boolean isImpassable() {
        return coordinate.getImpassable();
    }
}
