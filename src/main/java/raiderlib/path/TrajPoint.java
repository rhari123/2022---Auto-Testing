package raiderlib.path;

import raiderlib.geometry.Point;

/**
 * This class is used to represent a trajectory point in a path
 */
public class TrajPoint extends Point {
    public double curvature;
    public double velocity;
    public double time;

    /**
     * Constructor for TrajPoint class
     * 
     * @param x x coordinate
     * @param y y coordinate
     */
    public TrajPoint(final double x, final double y) {
        super(x, y);
        this.velocity = 0;
        this.curvature = 0;
        this.time = 0;
    }

    /**
     * Constructor for TrajPoint class
     * 
     * @param x         x coordinate
     * @param y         y coordinate
     * @param curvature lookup curvature
     */
    public TrajPoint(final double x, final double y, final double curvature) {
        super(x, y);
        this.curvature = curvature;
        this.velocity = 0;
        this.time = 0;
    }

    /**
     * Constructor for TrajPoint class
     * 
     * @param x         x coordinate
     * @param y         y coordinate
     * @param curvature lookup curvature
     * @param vel       lookup velocity
     */
    public TrajPoint(final double x, final double y, final double curvature, final double vel) {
        super(x, y);
        this.curvature = curvature;
        this.velocity = vel;
        this.time = 0;
    }   
     public TrajPoint(final double x, final double y, final double curvature, final double vel, final double time) {
        super(x, y);
        this.curvature = curvature;
        this.velocity = vel;
        this.time = time;
    }

}