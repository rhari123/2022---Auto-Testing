package raiderlib.control;

import java.util.ArrayList;

import raiderlib.geometry.Point;
import raiderlib.geometry.Pose;
import raiderlib.path.Path;
import raiderlib.path.TrajPoint;

public class PurePursuitController {
    double maxVelocity;
    double maxAcceleration;
    double maxDeceleration;
    double trackWidth;
    double minLD, maxLD;
    boolean isFinished;
    Path path;
    int lookAheadPoint = 0;
    int closestPoint = 0;

    /**
     * Constructor for PurePursuitController class
     * 
     * @param driveCharacterization driveCharacterization of robot
     * @param lookAheadDistance     max lookahead distance
     */
    public PurePursuitController(Path path, DriveCharacterization driveCharacterization, double minLD, double maxLD) {
        this(path, driveCharacterization.maxVelocity, driveCharacterization.maxAcceleration,
                driveCharacterization.maxDeceleration, driveCharacterization.trackWidth, minLD, maxLD);
    }

    public PurePursuitController(Path path, double maxVelocity, double maxAcceleration, double maxDeceleration,
            double trackWidth, double minLD, double maxLD) {
        this.minLD = minLD;
        this.maxLD = maxLD;
        this.path = path;
        this.maxVelocity = maxVelocity;
        this.maxAcceleration = maxAcceleration;
        this.maxDeceleration = maxDeceleration;
        this.trackWidth = trackWidth;
        calc_velocity(this.maxVelocity, this.maxAcceleration, this.maxDeceleration, this.path.get_points());
        isFinished = false;
    }

    public void reset() {
        this.isFinished = false;
        this.lookAheadPoint = 0;
        this.closestPoint = 0;
    }

    /**
     * This method is used to pursuit a path
     * 
     * @param path     path to be pursuited
     * @param currPose robot's current pose
     * @return a DriveCommand for the robot to follow
     */
    public DriveCommand pursuit_path(Pose currPose) {
        TrajPoint closestPoint = get_closest_point(path, currPose);
        // Point lookAheadPoint = get_lookAhead_point(path, currPose,
        // currPose.dist(closestPoint) + (1/ closestPoint.curvature <= 20 ? 3
        // : this.lookAheadDistance));
        TrajPoint lookAheadPoint = get_lookAhead_point(path, currPose, currPose.dist(closestPoint) + (maxLD - minLD)/(1 + Math.exp((float)(-(1/closestPoint.curvature - 30)))) + minLD);
        // System.out.println(Math.abs(path.getArcLength(path.getSize() - 1) - path.getArcLength(this.closestPoint)));
        if (Math.abs(path.getArcLength(path.getSize() - 1) - path.getArcLength(this.closestPoint))<= 5) {
            isFinished = true;
            return new DriveCommand(0, 0);
        }

        return pursuit_point(lookAheadPoint, currPose, closestPoint.velocity);
    }
    

    /**
     * This method tells whether the robot has finished the path
     * 
     * @return a boolean of whether the robot has finished the path
     */
    public boolean is_finished() {
        return isFinished;
    }

    /**
     * This method is used to pursuit a point
     * 
     * @param p        point to pursuit
     * @param currPose robot's current pose
     * @param velocity velocity to pursuit at
     * @return a DriveCommand for the robot to follow
     */
    DriveCommand pursuit_point(Point p, Pose currPose, double velocity) {
        double arcCurve = get_arc_curvature(p, currPose);
        if (!path.isReversed())
            return new DriveCommand(velocity * (2 + arcCurve * this.trackWidth),
                    velocity * (2 - arcCurve * this.trackWidth));
        else
            return new DriveCommand(-velocity * (2 - arcCurve * this.trackWidth),
                    -velocity * (2 + arcCurve * this.trackWidth));
    }

    /**
     * This method is used to calculate the curvature of the arc to a lookahead
     * point
     * 
     * @param lookAheadPoint the lookahead point
     * @param currPose       robot's current pose
     * @return the curvature of the arc
     */
    double get_arc_curvature(Point lookAheadPoint, Pose currPose) {
        double a = -Math.tan(currPose.theta);
        double c = Math.tan(currPose.theta) * currPose.x - currPose.y;
        double x = Math.abs(a * lookAheadPoint.x + lookAheadPoint.y + c) / Math.sqrt(a * a + 1);
        int sign = ((Math.sin(currPose.theta) * (lookAheadPoint.x - currPose.x)
                - Math.cos(currPose.theta) * (lookAheadPoint.y - currPose.y)) > 0) ? 1 : -1;
        if (!path.isReversed())
            return sign * 2 * x / (Math.pow(currPose.dist(lookAheadPoint), 2));
        else
            return -sign * 2 * x / (Math.pow(currPose.dist(lookAheadPoint), 2));
    }

    /**
     * This method is used to find the lookahead point on a path
     * 
     * @param path              the path the robot follows
     * @param point          robot's current pose
     * @param lookAheadDistance lookahead distance of robot
     * @return the point on the path within the lookahead radius. If there is more
     *         than one, the robot chooses the later one on the path
     */
    TrajPoint get_lookAhead_point(Path path, Pose pose, double lookAheadDistance) {
        int ID = lookAheadPoint;
        ArrayList<TrajPoint> points = path.get_points(ID);
        TrajPoint output = points.get(0);
        for (TrajPoint p : points) {
            if (Math.abs((float) (p.dist(pose) - lookAheadDistance)) <= 1) {
                lookAheadPoint = ID;
                output = p;
                break;
            }
            ID++;
        }
        return output;
    }

    /**
     * This method finds the closest point on the path to the robot
     * 
     * @param path     the path the robot follows
     * @param currPose robot's current pose
     * @return the point on the path which is closest to the robot
     */
    TrajPoint get_closest_point(Path path, Pose currPose) {
        int ID = closestPoint;
        ArrayList<TrajPoint> points = path.get_points(ID);
        TrajPoint output = points.get(0);
        for (TrajPoint p : points) {
            if (currPose.dist(p) <= 3) {
                closestPoint = ID;
                output = p;
                break;
            }
            ID++;
        }
        return output;
    }

    /**
     * This method is used to calculate the lookup velocites at each trajectory
     * point
     * 
     * @param driveCharacterization characterization of the drivetrain
     * @param points                arraylist of points to set velocites for
     */
    void calc_velocity(double maxVelocity, double maxAcceleration, double maxDeceleration,
            ArrayList<TrajPoint> points) {
        for (TrajPoint point : points) {
            if (point.curvature == 0)
                point.velocity = this.maxVelocity;
            else if (this.maxVelocity / (point.curvature) < this.maxVelocity)
                point.velocity = this.maxVelocity / (point.curvature);
            else
                point.velocity = this.maxVelocity;
        }
        points.get(0).velocity = 0;
        points.get(points.size() - 1).velocity = 0;
        double plausVel = 0;
        for (int i = 1; i < points.size() - 1; i++) {
            plausVel = Math.sqrt(Math.pow(points.get(i - 1).velocity, 2)
                    + 2 * this.maxAcceleration * points.get(i).dist(points.get(i - 1)));
            if (plausVel < points.get(i).velocity)
                points.get(i).velocity = plausVel;
        }
        plausVel = 0;
        points.get(0).velocity = 0;
        points.get(points.size() - 1).velocity = 0;
        for (int i = points.size() - 2; i > 0; i--) {
            plausVel = Math.sqrt(Math.pow(points.get(i + 1).velocity, 2)
                    + 2 * this.maxDeceleration * points.get(i).dist(points.get(i + 1)));
            if (plausVel < points.get(i).velocity)
                points.get(i).velocity = plausVel;
        }
        points.get(0).velocity = points.get(1).velocity;
    }
}