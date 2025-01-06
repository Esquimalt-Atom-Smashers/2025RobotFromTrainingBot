package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.util.Units;

public final class SwerveConstants {
    public static final boolean INVERT_GYRO = false; // Always ensure Gyro is CCW+ CW-

    /* Drivetrain Constants */
    public static final double TRACK_WIDTH = Units.inchesToMeters(20.6); // This must be tuned to specific robot
    public static final double WHEEL_BASE = Units.inchesToMeters(25.5); // This must be tuned to specific robot
    public static final double WHEEL_CIRCUMFERENCE = Units.inchesToMeters(4.0) * Math.PI;
    public static final int MAX_RPM_DRIVE_MOTOR =6000;//6000 for kraken, 5200 for falcon

    /* Module Gear Ratios */
    public static final double GEAR_RATIO_DRIVE = (6.75 / 1.0); // 5.14:1
    public static final double GEAR_RATIO_ANGLE = (150/7 / 1.0); // 12.8:1 //TODO double check this value, 21 seems wrong

    /* Motor Inverts */
    public static final boolean INVERT_ANGLE_MOTOR = true;
    public static final boolean INVERT_DRIVE_MOTOR = false;

    /* Angle Encoder Invert */
    public static final SensorDirectionValue INVERT_CANCODER = SensorDirectionValue.Clockwise_Positive;

    /* Swerve Current Limiting */
    public static final int SUPPLY_CURRENT_LIMIT_ANGLE_MOTORS = 20;
    public static final int STATOR_CURRENT_LIMIT_ANGLE_MOTORS = 8;
    public static final boolean ENABLE_CURRENT_LIMITS_ANGLE_MOTORS = true;

    public static final int SUPPLY_CURRENT_LIMIT_DRIVE_MOTORS = 40;
    public static final int STATOR_CURRENT_LIMIT_DRIVE_MOTORS = 20;
    public static final boolean ENABLE_CURRENT_LIMITS_DRIVE_MOTORS = true;

    /* Angle Motor PID Values */
    public static final double KP_ANGLE_MOTORS = 45; // 43
    public static final double KI_ANGLE_MOTORS = 0.0;
    public static final double KD_ANGLE_MOTORS = 0.0;

    /* Drive Motor PID Values */
    public static final double KP_DRIVE_MOTORS = 0.2;
    public static final double KI_DRIVE_MOTORS = 0.0;//Don't use, KI value spools up too much
    public static final double KD_DRIVE_MOTORS = 0.01;

    /* Drive Motor Characterization Values 
        * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
    public static final double KS_DRIVE_MOTORS = (0.4); //TODO: This must be tuned to specific robot
    public static final double KV_DRIVE_MOTORS = (12.7/MAX_RPM_DRIVE_MOTOR*60);//0.12 in all docs units are Volts per revolution per second
    public static final double KA_DRIVE_MOTORS = 0;//(0.27 / 12); //TODO: This is final robot weight dependant 

    /* Swerve Profiling Values */
    /** Meters per Second */
    public static final double MAX_WHEEL_SPEED_M_S = MAX_RPM_DRIVE_MOTOR/GEAR_RATIO_DRIVE/60*WHEEL_CIRCUMFERENCE;//rpm/gear ratio*circumference/60rps
    /** Radians per Second */
    public static final double MAX_ANG_VEL_RAD_S = 10.0;// Radians per second //TODO: This must be tuned to specific robot

    /* Neutral Modes */
    public static final NeutralModeValue NEUTRAL_BREAK_MODE_ANGLE_MOTORS = NeutralModeValue.Coast;//TODO: check if the wheel can turn by hand in angle
    public static final NeutralModeValue NEUTRAL_BREAK_MODE_DRIVE_MOTORS = NeutralModeValue.Brake;

    /* Module Specific Constants */
    /* Front Left Module - Module 0 */
    public static final class frontLeftModule { //TODO: This must be tuned to specific robot
        public static final SwerveModuleConstants MODULE_CONSTANTS_FRONT_LEFT = 
            new SwerveModuleConstants(
                40,
                4,
                4,
                45-8//TODO angle offset in degrees
                );
    }

    /* Front Right Module - Module 1 */
    public static final class frontRightModule { //TODO: This must be tuned to specific robot
        public static final SwerveModuleConstants MODULE_CONSTANTS_FRONT_RIGHT = 
            new SwerveModuleConstants(
                10,
                1,
                1,
                0-8// TODO angle offset in degrees
                );
    }
    
    /* Back Left Module - Module 2 */
    public static final class rearLeftModule { //TODO: This must be tuned to specific robot
        public static final SwerveModuleConstants MODULE_CONSTANTS_REAR_LEFT = 
            new SwerveModuleConstants(
                30,
                3,
                3,
                -119.5-8// TODO angle offset in degrees
                );
    }

    /* Back Right Module - Module 3 */
    public static final class rearRightModule { //TODO: This must be tuned to specific robot
        public static final SwerveModuleConstants MODULE_CONSTANTS_REAR_RIGHT = 
            new SwerveModuleConstants(
                20,
                2,
                2,
                56.7-8//TODO angle offset in degrees
                );
    }
        
}
