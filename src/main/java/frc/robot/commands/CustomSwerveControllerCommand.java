package frc.robot.commands;

    
import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;

import java.util.function.Consumer;
import java.util.function.Supplier;

/**
 * A command that uses two PID controllers ({@link PIDController}) and a ProfiledPIDController
 * ({@link ProfiledPIDController}) to follow a trajectory {@link Trajectory} with a swerve drive.
 *
 * <p>This command outputs the raw desired Swerve Module States ({@link SwerveModuleState}) in an
 * array. The desired wheel and module rotation velocities should be taken from those and used in
 * velocity PIDs.
 *
 * <p>The robot angle controller follows the angle given by the trajectory 
 *
 * <p>This class is provided by the NewCommands VendorDep
 */
public class CustomSwerveControllerCommand extends Command {
  private final Timer m_timer = new Timer();
  private final Trajectory m_trajectory;
  private final Supplier<Pose2d> m_pose;
  private final SwerveDriveKinematics m_kinematics;
  private final HolonomicDriveController m_controller;
  private final Consumer<SwerveModuleState[]> m_outputModuleStates;
  private boolean reachedDestination;

  /**
   * Constructs a new SwerveControllerCommand that when executed will follow the provided
   * trajectory. This command will not return output voltages but rather raw module states from the
   * position controllers which need to be put into a velocity PID.
   *
   * <p>Note: The controllers will *not* set the outputVolts to zero upon completion of the path.
   * This is left to the user to do since it is not appropriate for paths with nonstationary
   * endstates.
   *
   * @param trajectory The trajectory to follow.
   * @param pose A function that supplies the robot pose - use one of the odometry classes to
   *     provide this.
   * @param kinematics The kinematics for the robot drivetrain.
   * @param xController The Trajectory Tracker PID controller for the robot's x position.
   * @param yController The Trajectory Tracker PID controller for the robot's y position.
   * @param thetaController The Trajectory Tracker PID controller for angle for the robot.
   * @param outputModuleStates The raw output module states from the position controllers.
   * @param requirements The subsystems to require.
   */
  public CustomSwerveControllerCommand(
      Trajectory trajectory,
      Supplier<Pose2d> pose,
      SwerveDriveKinematics kinematics,
      PIDController xController,
      PIDController yController,
      ProfiledPIDController thetaController,
      Consumer<SwerveModuleState[]> outputModuleStates,
      Subsystem... requirements) {
    this(
        trajectory,
        pose,
        kinematics,
        new HolonomicDriveController(
            requireNonNullParam(xController, "xController", "SwerveControllerCommand"),
            requireNonNullParam(yController, "yController", "SwerveControllerCommand"),
            requireNonNullParam(thetaController, "thetaController", "SwerveControllerCommand")),
        outputModuleStates,
        requirements);
  }

  /**
   * Constructs a new SwerveControllerCommand that when executed will follow the provided
   * trajectory. This command will not return output voltages but rather raw module states from the
   * position controllers which need to be put into a velocity PID.
   *
   * <p>Note: The controllers will *not* set the outputVolts to zero upon completion of the path-
   * this is left to the user, since it is not appropriate for paths with nonstationary endstates.
   *
   * @param trajectory The trajectory to follow.
   * @param pose A function that supplies the robot pose - use one of the odometry classes to
   *     provide this.
   * @param kinematics The kinematics for the robot drivetrain.
   * @param controller The HolonomicDriveController for the drivetrain.
   * @param outputModuleStates The raw output module states from the position controllers.
   * @param requirements The subsystems to require.
   */
  @SuppressWarnings("this-escape")
  public CustomSwerveControllerCommand(
      Trajectory trajectory,
      Supplier<Pose2d> pose,
      SwerveDriveKinematics kinematics,
      HolonomicDriveController controller,
      Consumer<SwerveModuleState[]> outputModuleStates,
      Subsystem... requirements) {
    m_trajectory = requireNonNullParam(trajectory, "trajectory", "SwerveControllerCommand");
    m_pose = requireNonNullParam(pose, "pose", "SwerveControllerCommand");
    m_kinematics = requireNonNullParam(kinematics, "kinematics", "SwerveControllerCommand");
    m_controller = requireNonNullParam(controller, "controller", "SwerveControllerCommand");


    m_outputModuleStates =
        requireNonNullParam(outputModuleStates, "outputModuleStates", "SwerveControllerCommand");

    addRequirements(requirements);
  }

  @Override
  public void initialize() {
    m_timer.restart();
    reachedDestination=false;
  }

  @Override
  public void execute() {
    double curTime = m_timer.get();
    var desiredState = m_trajectory.sample(curTime);

    // Use the heading from the current trajectory state.
    Rotation2d targetHeading = desiredState.poseMeters.getRotation();

    // Calculate target speeds using the dynamically computed heading.
    var targetChassisSpeeds = m_controller.calculate(m_pose.get(), desiredState, targetHeading);
    var targetModuleStates = m_kinematics.toSwerveModuleStates(targetChassisSpeeds);

    m_outputModuleStates.accept(targetModuleStates);
  }

  @Override
  public void end(boolean interrupted) {
    m_timer.stop();
  }

  @Override
  public boolean isFinished() {
    Pose2d finalPose = m_trajectory.getStates().get(m_trajectory.getStates().size() - 1).poseMeters;
    Pose2d currentPose = m_pose.get();

    // Calculate the difference between the final pose and the current pose
    Transform2d poseDifference = finalPose.minus(currentPose);

    // Check if the absolute values of the differences are within the thresholds
    if (Math.abs(poseDifference.getX()) < Constants.Auto.TOLERANCE_XY_METERS &&
        Math.abs(poseDifference.getY()) < Constants.Auto.TOLERANCE_XY_METERS &&
        Math.abs(poseDifference.getRotation().getRadians()) < Constants.Auto.TOLERANCE_ROTATION_RAD) {
        reachedDestination = true;
    }
    

    //wait three seconds after time has elapsed from the predicted path, unless destination was reached
    return m_timer.hasElapsed(m_trajectory.getTotalTimeSeconds()+Constants.Auto.WAIT_TIME_S)|reachedDestination;
  }
}