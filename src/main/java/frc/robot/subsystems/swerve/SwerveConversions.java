package frc.robot.subsystems.swerve;

public class SwerveConversions {

    /**
     * @param positionCounts CANCoder Position Counts
     * @param gearRatio Gear Ratio between CANCoder and Mechanism
     * @return Degrees of Rotation of Mechanism
     */
    public static double CANcoderToDegrees(double positionCounts, double gearRatio) {
        return positionCounts * (360.0 / (gearRatio * 4096.0));
    }

    /**
     * @param degrees Degrees of rotation of Mechanism
     * @param gearRatio Gear Ratio between CANCoder and Mechanism
     * @return CANCoder Position Counts
     */
    public static double degreesToCANcoder(double degrees, double gearRatio) {
        return degrees / (360.0 / (gearRatio * 4096.0));
    }

    /**
     * @param counts Falcon Position Counts
     * @param gearRatio Gear Ratio between Falcon and Mechanism
     * @return Degrees of Rotation of Mechanism
     */
    public static double falconToDegrees(double positionCounts, double gearRatio) {
        return positionCounts * (360.0 / (gearRatio * 2048.0));
    }

    /**
     * @param degrees Degrees of rotation of Mechanism
     * @param gearRatio Gear Ratio between Falcon and Mechanism
     * @return Falcon Position Counts
     */
    public static double degreesToFalcon(double degrees, double gearRatio) {
        return degrees / (360.0 / (gearRatio * 2048.0));
    }

    /**
     * @param velocityCounts Falcon Velocity Counts
     * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
     * @return RPM of Mechanism
     */
    public static double falconToRPM(double velocityCounts, double gearRatio) {
        double motorRPM = velocityCounts * (600.0 / 2048.0);        
        double mechRPM = motorRPM / gearRatio;
        return mechRPM;
    }

    /**
     * @param RPS Rotations per Second for the Motor
     * @param gearRatioGear Ratio between Motor and Mechanism
     * @return Radians per Second of Rotation of Mechanism
     */
    public static double motorRPSToRadPS(double RPS, double gearRatio) {
        return RPS * (Math.PI * 2) / gearRatio;
    }

    /**
     * @param RPS Rotations per Second for the Motor
     * @param gearRatio Gear Ratio between Motor and Mechanism\
     * @param circumference Circumference of the Mechanism
     * @return Meters per Second of Movement of Mechanism
     */
    public static double motorRPSToMPS(double RPS, double circumference, double gearRatio) {
        return RPS / gearRatio * circumference;
    }

    /**
     * @param velocitycounts Falcon Velocity Counts
     * @return Falcon Velocity in Meters Per second
     */
    //edited by brandon
    public static double falconToMPS(double velocitycounts){
        double wheelRPM = falconToRPM(velocitycounts, SwerveConstants.GEAR_RATIO_DRIVE);
        double wheelMPS = (wheelRPM * SwerveConstants.WHEEL_CIRCUMFERENCE) / 60;
        return wheelMPS;
    }

    /**
     * @param velocityMPS Velocity MPS
     * @return Falcon Velocity Counts
     */
    //edited by brandon
    public static double MPSToKraken(double velocityMPS){
        double wheelRPM = ((velocityMPS * 60) / SwerveConstants.WHEEL_CIRCUMFERENCE);
        double motorVelocityRPS = wheelRPM*SwerveConstants.GEAR_RATIO_DRIVE/60;
        return motorVelocityRPS;
    }

    /**
     * @param positionCounts Falcon Position Counts
     * @return Meters
     */
    //edited by brandon
    public static double falconToMeters(double positionCounts){
        return positionCounts * (SwerveConstants.WHEEL_CIRCUMFERENCE / (SwerveConstants.GEAR_RATIO_DRIVE * 2048.0));
    }

    /**
     * @param meters Meters
     * @return Falcon Position Counts
     */
    //edited by brandon
    public static double MetersToFalcon(double meters){
        return meters / (SwerveConstants.WHEEL_CIRCUMFERENCE / (SwerveConstants.GEAR_RATIO_DRIVE* 2048.0));
    }
}