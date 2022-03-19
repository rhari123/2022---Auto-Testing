package raiderlib.control;

import java.util.ArrayList;

import raiderlib.geometry.Point;
import raiderlib.geometry.Pose;
import raiderlib.path.Path;
import raiderlib.path.TrajPoint;

public class RamseteController {
    double kBeta;// converging parameter
    double kZeta;// dampening parameter
    Path m_path;
    DriveCharacterization m_driveCharacterization;
    double startTime;

    int prevRefPoint;

    boolean isFinished;

    public RamseteController(Path path, DriveCharacterization driveCharacterization, double b, double z) {
        m_path = path;
        prevRefPoint = 0;
        m_driveCharacterization = driveCharacterization;
        kBeta = b;
        kZeta = z;
        this.startTime = 0;
        isFinished = false;
    }

    public RamseteController(Path path, DriveCharacterization driveCharacterization) {
        this(path, driveCharacterization, 2.0, 0.7);
    }

    public void reset(double s) {
        prevRefPoint = 0;
        startTime = s;
        isFinished = false;
    }

    public boolean isFinished() {
        return isFinished;
    }

    public DriveCommand update(double timestamp, Pose currPose) {
        TrajPoint reference = getReference(timestamp - startTime);
        if (isFinished)
            return new DriveCommand(0, 0);
        Pose errPose = getErrorPose(reference, currPose);
        double angVel = reference.velocity * get_arc_curvature(reference, currPose);
        double k = 2 * kZeta * Math.sqrt(angVel * angVel + kBeta * reference.velocity * reference.velocity);
        double sinc = Math.sin(errPose.theta) / errPose.theta;
        double velCmd = reference.velocity * Math.cos(errPose.theta) + k * errPose.x;
        double angCmd = angVel + k * errPose.theta + kZeta * reference.velocity * sinc * errPose.y;
        if (!m_path.isReversed())
            return new DriveCommand(velCmd - angCmd * m_driveCharacterization.trackWidth / 2,
                    velCmd + angCmd * m_driveCharacterization.trackWidth / 2);
        else
            return new DriveCommand(-(velCmd + angCmd * m_driveCharacterization.trackWidth / 2),
                    -(velCmd - angCmd * m_driveCharacterization.trackWidth / 2));
    }

    Pose getErrorPose(Point reference, Pose currPose) {
        double e_x = reference.x - currPose.x;
        double e_y = reference.y - currPose.y;
        return new Pose(e_x * Math.cos(currPose.theta) + e_y * Math.sin(currPose.theta),
                e_y * -Math.sin(currPose.theta) + e_y * Math.cos(e_y), Math.atan(e_y / e_x) - currPose.theta);
    }

    TrajPoint getReference(double time) {
        double dt = Math.abs(time - m_path.getPoint(prevRefPoint).time);
        while (dt <= Math.abs(time - m_path.getPoint(prevRefPoint + 1).time) && prevRefPoint < m_path.getSize()) {
            prevRefPoint++;
            dt = Math.abs(time - m_path.getPoint(prevRefPoint).time);
        }
        if (prevRefPoint >= m_path.getSize() - 1)
            isFinished = true;
        return m_path.getPoint(prevRefPoint);
    }

    /**
     * This method is used to calculate the curvature of the arc to a lookahead
     * point
     * 
     * @param reference the reference point
     * @param currPose  robot's current pose
     * @return the curvature of the arc
     */
    double get_arc_curvature(Point reference, Pose currPose) {
        double a = -Math.tan(currPose.theta);
        double c = Math.tan(currPose.theta) * currPose.x - currPose.y;
        double x = Math.abs(a * reference.x + reference.y + c) / Math.sqrt(a * a + 1);
        int sign = ((Math.sin(currPose.theta) * (reference.x - currPose.x)
                - Math.cos(currPose.theta) * (reference.y - currPose.y)) > 0) ? 1 : -1;
        if (!m_path.isReversed())
            return sign * 2 * x / (Math.pow(currPose.dist(reference), 2));
        else
            return -sign * 2 * x / (Math.pow(currPose.dist(reference), 2));
    }

    void calc_velocity(ArrayList<TrajPoint> points) {
        for (TrajPoint point : points) {
            if (point.curvature == 0)
                point.velocity = m_driveCharacterization.maxVelocity;
            else if (m_driveCharacterization.maxVelocity / (point.curvature * 5) < m_driveCharacterization.maxVelocity)
                point.velocity = m_driveCharacterization.maxVelocity / (point.curvature * 5);
            else
                point.velocity = m_driveCharacterization.maxVelocity;
        }
        points.get(0).velocity = 0;
        points.get(points.size() - 1).velocity = 0;
        double plausVel = 0;
        for (int i = 1; i < points.size() - 1; i++) {
            plausVel = Math.sqrt(Math.pow(points.get(i - 1).velocity, 2)
                    + 2 * m_driveCharacterization.maxAcceleration * points.get(i).dist(points.get(i - 1)));
            if (plausVel < points.get(i).velocity)
                points.get(i).velocity = plausVel;
        }
        plausVel = 0;
        points.get(0).velocity = 0;
        points.get(points.size() - 1).velocity = 0;
        for (int i = points.size() - 2; i > 0; i--) {
            plausVel = Math.sqrt(Math.pow(points.get(i + 1).velocity, 2)
                    + 2 * m_driveCharacterization.maxDeceleration * points.get(i).dist(points.get(i + 1)));
            if (plausVel < points.get(i).velocity)
                points.get(i).velocity = plausVel;
        }
        double dx = 0;
        points.get(0).time = 0;
        for (int i = 1; i < points.size(); i++) {
            dx = points.get(i).dist(points.get(i - 1));
            points.get(i).time = 2 * dx / (points.get(i).velocity + points.get(i - 1).velocity)
                    + points.get(i - 1).velocity;
        }
    }

}