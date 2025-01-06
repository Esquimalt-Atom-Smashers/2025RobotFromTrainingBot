package frc.robot.subsystems.swerve;


import frc.robot.subsystems.PigeonGyro;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;



import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveSubsystem extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    private SwerveModule[] mSwerveMods;
    public PigeonGyro gyro;
    //this SwerveDrive kinematics needs to be created in the same order as the  array for the swerve modules
    //below This is why it has been moved to this class instead of the swerveconstants class. 
    public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(SwerveConstants.WHEEL_BASE / 2.0, SwerveConstants.TRACK_WIDTH / 2.0),//fR
            new Translation2d(-SwerveConstants.WHEEL_BASE / 2.0, SwerveConstants.TRACK_WIDTH / 2.0),//RR
            new Translation2d(-SwerveConstants.WHEEL_BASE / 2.0, -SwerveConstants.TRACK_WIDTH / 2.0),//RL
            new Translation2d(SwerveConstants.WHEEL_BASE / 2.0, -SwerveConstants.TRACK_WIDTH / 2.0));//FL


    public SwerveSubsystem() {
        gyro = new PigeonGyro(); //TODO fix this so gyro is created as a sensor in the robot container
        

        mSwerveMods = new SwerveModule[] {
            new SwerveModule( "FR", SwerveConstants.frontRightModule.MODULE_CONSTANTS_FRONT_RIGHT),//FR
            new SwerveModule( "RR", SwerveConstants.rearRightModule.MODULE_CONSTANTS_REAR_RIGHT),//RR
            new SwerveModule( "RL", SwerveConstants.rearLeftModule.MODULE_CONSTANTS_REAR_LEFT),//RL
            new SwerveModule( "FL", SwerveConstants.frontLeftModule.MODULE_CONSTANTS_FRONT_LEFT)//FL
        };

        /* By pausing init for a second before setting module offsets, we avoid a bug with inverting motors.
         * See https://github.com/Team364/BaseFalconSwerve/issues/8 for more info.
         */
        Timer.delay(1.0);
        resetModulesToAbsolute();

        swerveOdometry = new SwerveDriveOdometry(swerveKinematics, gyro.getYawRotation2d(), getModulePositions());
    }
    //Modified to only run in Closed loop. I don't know why we were running in open loop.
    public void driveMPS(Translation2d translationMPS, double rotationRadPerSecond, boolean fieldRelative) {
        SwerveModuleState[] swerveModuleStates =
        swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translationMPS.getX(), 
                                    translationMPS.getY(), 
                                    rotationRadPerSecond, 
                                    gyro.getYawRotation2d()
                                )
                                : new ChassisSpeeds(
                                    translationMPS.getX(), 
                                    translationMPS.getY(), 
                                    rotationRadPerSecond)
                                );
        setModuleStates(swerveModuleStates);
    }    

    /* Sets the states of each module using a closed loop motor velocity and angular position*/
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.MAX_WHEEL_SPEED_M_S);
        for (int i = 0; i < mSwerveMods.length; i++) {
            mSwerveMods[i].setDesiredState(desiredStates[i]);
        }
    }    

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(gyro.getYawRotation2d(), getModulePositions(), pose);
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < mSwerveMods.length; i++) {
            states[i] = mSwerveMods[i].getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (int i = 0; i < mSwerveMods.length; i++) {
            positions[i] = mSwerveMods[i].getPosition();
        }
        return positions;
    }

    //TODO: this should only be in the subsystem if it is making calculating an offset, otherwise it shuld be a method in the 
    // Gyro subsystem. This is redundant.
    public void zeroGyro(){
        gyro.reset();
    }

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }

    @Override
    public void periodic(){
        swerveOdometry.update(gyro.getYawRotation2d(), getModulePositions());  
        SmartDashboard.putNumber("yaw",gyro.getYaw());

        for(SwerveModule mod : mSwerveMods){

            SmartDashboard.putNumber("Mod " + mod.modulePosition + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.modulePosition + " Position (meters)", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.modulePosition + " Velocity", mod.getState().speedMetersPerSecond);    
        }
    }
}