package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;

public class SwerveModule {
    public String modulePosition;
    private Rotation2d angleOffset;
    private Rotation2d lastAngle;

    private TalonFX mAngleMotor;
    private TalonFX mDriveMotor;
    private CANcoder angleEncoder;

    private PositionVoltage positionVoltageRequestAngle = new PositionVoltage(0).withSlot(0);
    private VelocityVoltage velocityVoltageRequestDrive = new VelocityVoltage(0).withSlot(0);
    
    public SwerveModuleState getState(){
        return new SwerveModuleState(getSpeed(),getAngle()); 
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            SwerveConversions.falconToMeters(mDriveMotor.getPosition().refresh().getValue()),
            //TODO Add a correction factor for coupling between the azimuth and drive gears. 
            // every time azimuth spins, it affects the real position of the drive wheel, and 
            //therefore affects odometry.
            getAngle()
        );
    }

    public Rotation2d getCanCoder(){
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition().refresh().getValue());
    }

    public void resetToAbsolute(){//TODO find out if we need to do this since we are using the canCoder as a Remote encoder
        double absolutePosition = SwerveConversions.degreesToFalcon(getCanCoder().getDegrees() - angleOffset.getDegrees(), SwerveConstants.GEAR_RATIO_ANGLE);
        mAngleMotor.setPosition(absolutePosition);
    }
    
    protected SwerveModule(String modulePosition, SwerveModuleConstants moduleConstants){
        this.modulePosition= modulePosition;
        this.angleOffset = moduleConstants.angleOffset;
        
        /* Angle Encoder Config */
        angleEncoder = new CANcoder(moduleConstants.cancoderID);
        configAngleEncoder();

        /* Angle Motor Config */
        mAngleMotor = new TalonFX(moduleConstants.angleMotorID);
        configAngleMotor();

        /* Drive Motor Config */
        mDriveMotor = new TalonFX(moduleConstants.driveMotorID);
        configDriveMotor();

        lastAngle = getState().angle;
    }

    protected void setDesiredState(SwerveModuleState desiredState){
        /* This is a custom optimize function, since default WPILib optimize assumes continuous controller which CTRE and Rev onboard is not */
        desiredState = optimize(desiredState, getState().angle); 
        setAngle(desiredState);
        setSpeed(desiredState);
    }

    private void setSpeed(SwerveModuleState desiredState){
        double velocityMotorRPS = SwerveConversions.MPSToKraken(desiredState.speedMetersPerSecond);
        mDriveMotor.setControl(velocityVoltageRequestDrive.withVelocity(velocityMotorRPS)); 
    }

    //TODO go over this to make sure the angle makes sense
    private void setAngle(SwerveModuleState desiredState){
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (SwerveConstants.MAX_WHEEL_SPEED_M_S * 0.01)) ? lastAngle : desiredState.angle; //Prevent rotating module if speed is less then 1%. Prevents Jittering.
        double angleD= angle.getDegrees();
        mAngleMotor.setControl(positionVoltageRequestAngle.withPosition(angleD/360));
        lastAngle = angle;
    }

    private Rotation2d getAngle(){
        return Rotation2d.fromRotations(BaseStatusSignal.getLatencyCompensatedValue(mAngleMotor.getPosition(), mAngleMotor.getVelocity()));
    }
    private double getSpeed() {
        return SwerveConversions.falconToMPS(mDriveMotor.getVelocity().refresh().getValue());
    }

    private void configAngleEncoder(){   
        CANcoderConfiguration swerveCanCoderConfig= new CANcoderConfiguration();
        swerveCanCoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        swerveCanCoderConfig.MagnetSensor.SensorDirection = SwerveConstants.INVERT_CANCODER;

        angleEncoder.getConfigurator().apply(swerveCanCoderConfig);
        MagnetSensorConfigs myMagnetSensorConfigs = new MagnetSensorConfigs();
        angleEncoder.getConfigurator().refresh(myMagnetSensorConfigs);
        myMagnetSensorConfigs.MagnetOffset = -angleOffset.getDegrees() / 360;
        angleEncoder.getConfigurator().apply(myMagnetSensorConfigs);
    }

    private void  configAngleMotor() {
        CurrentLimitsConfigs angleCurrentLimits = new CurrentLimitsConfigs();
        angleCurrentLimits.SupplyCurrentLimitEnable = SwerveConstants.ENABLE_CURRENT_LIMITS_ANGLE_MOTORS;
        angleCurrentLimits.SupplyCurrentLimit = SwerveConstants.SUPPLY_CURRENT_LIMIT_ANGLE_MOTORS;
        angleCurrentLimits.SupplyTimeThreshold = 0.0; //hardcoded to prevent activation
        angleCurrentLimits.StatorCurrentLimitEnable =SwerveConstants.ENABLE_CURRENT_LIMITS_ANGLE_MOTORS;
        angleCurrentLimits.StatorCurrentLimit=SwerveConstants.STATOR_CURRENT_LIMIT_ANGLE_MOTORS;
        
        TalonFXConfiguration swerveAngleFXConfig = new TalonFXConfiguration();
        swerveAngleFXConfig.Slot0.kP = SwerveConstants.KP_ANGLE_MOTORS;
        swerveAngleFXConfig.Slot0.kI = SwerveConstants.KI_ANGLE_MOTORS;
        swerveAngleFXConfig.Slot0.kD = SwerveConstants.KD_ANGLE_MOTORS;
        swerveAngleFXConfig.CurrentLimits = angleCurrentLimits;
        swerveAngleFXConfig.ClosedLoopGeneral.ContinuousWrap = true;;
        
        TalonFXConfigurator configurator = mAngleMotor.getConfigurator();
        configurator.apply(swerveAngleFXConfig);

        MotorOutputConfigs outputConfigs = new MotorOutputConfigs();

        outputConfigs.Inverted = SwerveConstants.INVERT_ANGLE_MOTOR ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        outputConfigs.NeutralMode = SwerveConstants.NEUTRAL_BREAK_MODE_ANGLE_MOTORS;

        configurator.apply(outputConfigs);

        FeedbackConfigs feedbackConfigs = new FeedbackConfigs();
        feedbackConfigs.FeedbackRemoteSensorID = angleEncoder.getDeviceID();
        feedbackConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;//fused cancoder is pro only
        configurator.apply(feedbackConfigs);
    }

    private void configDriveMotor(){  
        
        /* Swerve Drive Motor Configuration */
        CurrentLimitsConfigs driveCurrentLimits = new CurrentLimitsConfigs();
        driveCurrentLimits.SupplyCurrentLimitEnable = SwerveConstants.ENABLE_CURRENT_LIMITS_DRIVE_MOTORS;
        driveCurrentLimits.SupplyCurrentLimit = SwerveConstants.SUPPLY_CURRENT_LIMIT_DRIVE_MOTORS;
        driveCurrentLimits.SupplyTimeThreshold = 0.0; //hardcoded to prevent activation
        driveCurrentLimits.StatorCurrentLimitEnable = SwerveConstants.ENABLE_CURRENT_LIMITS_DRIVE_MOTORS;
        driveCurrentLimits.StatorCurrentLimit=SwerveConstants.STATOR_CURRENT_LIMIT_DRIVE_MOTORS;
        TalonFXConfiguration swerveDriveFXConfig = new TalonFXConfiguration();

        swerveDriveFXConfig.Slot0.kP = SwerveConstants.KP_DRIVE_MOTORS;
        swerveDriveFXConfig.Slot0.kI = SwerveConstants.KI_DRIVE_MOTORS;
        swerveDriveFXConfig.Slot0.kD = SwerveConstants.KD_DRIVE_MOTORS;
        swerveDriveFXConfig.Slot0.kS = SwerveConstants.KS_DRIVE_MOTORS;
        swerveDriveFXConfig.Slot0.kV = SwerveConstants.KV_DRIVE_MOTORS;
        swerveDriveFXConfig.Slot0.kA = SwerveConstants.KA_DRIVE_MOTORS;
        swerveDriveFXConfig.CurrentLimits = driveCurrentLimits;

        TalonFXConfigurator configurator = mDriveMotor.getConfigurator();
        configurator.apply(swerveDriveFXConfig);

        MotorOutputConfigs outputConfigs = new MotorOutputConfigs();

        outputConfigs.NeutralMode = SwerveConstants.NEUTRAL_BREAK_MODE_DRIVE_MOTORS;
        outputConfigs.Inverted = SwerveConstants.INVERT_DRIVE_MOTOR ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        
        configurator.apply(outputConfigs);
    }



    /**
   * Minimize the change in heading the desired swerve module state would require by potentially
   * reversing the direction the wheel spins. Customized from WPILib's version to include placing
   * in appropriate scope for CTRE onboard control.
   *
   * @param desiredState The desired state.
   * @param currentAngle The current module angle.
   */
  private static SwerveModuleState optimize(SwerveModuleState desiredState, Rotation2d currentAngle) {
    //TODO check if this is why some wheels appeared to be oriented the opposite way during testing
    //TODO add boolean to make motors go to same orientation on startup may not work with static method
    double targetAngle = placeInAppropriate0To360Scope(currentAngle.getDegrees(), desiredState.angle.getDegrees());
    double targetSpeed = desiredState.speedMetersPerSecond;
    double delta = targetAngle - currentAngle.getDegrees();
    if (Math.abs(delta) > 90){
        targetSpeed = -targetSpeed;
        targetAngle = delta > 90 ? (targetAngle -= 180) : (targetAngle += 180);
    }        
    return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle));
  }

  /**
     * @param scopeReference Current Angle
     * @param newAngle Target Angle
     * @return Closest angle within scope
     */
    private static double placeInAppropriate0To360Scope(double scopeReference, double newAngle) {
      double lowerBound;
      double upperBound;
      double lowerOffset = scopeReference % 360;
      if (lowerOffset >= 0) {
          lowerBound = scopeReference - lowerOffset;
          upperBound = scopeReference + (360 - lowerOffset);
      } else {
          upperBound = scopeReference - lowerOffset;
          lowerBound = scopeReference - (360 + lowerOffset);
      }
      while (newAngle < lowerBound) {
          newAngle += 360;
      }
      while (newAngle > upperBound) {
          newAngle -= 360;
      }
      if (newAngle - scopeReference > 180) {
          newAngle -= 360;
      } else if (newAngle - scopeReference < -180) {
          newAngle += 360;
      }
      return newAngle;
  }
}