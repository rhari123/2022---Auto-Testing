package raiderlib.path;

import java.util.ArrayList;

import raiderlib.geometry.Point;

/**
 * This class is used as a segment between two waypoints
 */
public class PathSegment {

    WayPoint startPoint;
    WayPoint endPoint;
    ArrayList<TrajPoint> points;

    /**
     * Constructor for PathSegment class
     * 
     * @param startPoint starting waypoint in segment
     * @param endPoint   ending waypoint in segment
     */
    public PathSegment(final WayPoint startPoint, final WayPoint endPoint) {
        this.startPoint = startPoint;
        this.endPoint = endPoint;
        points = gen_points();
    }

    /**
     * This method generates trajectory points in a segment
     * 
     * @return ArrayList of generated points
     */
    ArrayList<TrajPoint> gen_points() {
        final ArrayList<TrajPoint> p = new ArrayList<>();
        final double d = get_divisor(1000);
        // System.out.println(d);
        for (double t = 1; t <= d; t++) {
            final double i = t / d;
            final double h1 = 2 * Math.pow(i, 3) - 3 * Math.pow(i, 2) + 1;
            final double h2 = -2 * Math.pow(i, 3) + 3 * Math.pow(i, 2);
            final double h3 = Math.pow(i, 3) - 2 * Math.pow(i, 2) + i;
            final double h4 = Math.pow(i, 3) - Math.pow(i, 2);
            final double x = h1 * this.startPoint.x + h2 * this.endPoint.x + h3 * this.startPoint.tanPoint.x
                    + h4 * this.endPoint.tanPoint.x;
            final double y = h1 * this.startPoint.y + h2 * this.endPoint.y + h3 * this.startPoint.tanPoint.y
                    + h4 * this.endPoint.tanPoint.y;
            p.add(new TrajPoint(x, y));
        }
        return p;
    }

    double get_divisor(final double currDivisor) {
        final double i = 1 / currDivisor;
        final double h1 = 2 * Math.pow(i, 3) - 3 * Math.pow(i, 2) + 1;
        final double h2 = -2 * Math.pow(i, 3) + 3 * Math.pow(i, 2);
        final double h3 = Math.pow(i, 3) - 2 * Math.pow(i, 2) + i;
        final double h4 = Math.pow(i, 3) - Math.pow(i, 2);
        final double x = h1 * this.startPoint.x + h2 * this.endPoint.x + h3 * this.startPoint.tanPoint.x
                + h4 * this.endPoint.tanPoint.x;
        final double y = h1 * this.startPoint.y + h2 * this.endPoint.y + h3 * this.startPoint.tanPoint.y
                + h4 * this.endPoint.tanPoint.y;
        if ((new Point(x, y)).dist(this.startPoint) <= 1)
            return currDivisor;
        return get_divisor(currDivisor * 2);
    }

    /**
     * This method returns the points in the segment
     * 
     * @return An ArrayList of the points in the segment
     */
    public ArrayList<TrajPoint> get_points() {
        return this.points;
    }
}