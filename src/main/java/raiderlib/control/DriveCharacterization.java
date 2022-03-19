package raiderlib.control;

public class DriveCharacterization {

    public final double maxVelocity;
    public final double maxAcceleration;
    public final double maxDeceleration;
    public final double trackWidth;
    /**
     * Constructor for Drivecharacterization class
     * @param maxVelocity max velocity
     * @param maxAcceleration max acceleration
     * @param trackWidth trackWidth of robot
     */
    public DriveCharacterization(double maxVelocity, double maxAcceleration, double maxDeceleration, double trackWidth){
        this.maxVelocity = maxVelocity;
        this.maxAcceleration = maxAcceleration;
        this.maxDeceleration = maxDeceleration;
        this.trackWidth = trackWidth;
    }


}