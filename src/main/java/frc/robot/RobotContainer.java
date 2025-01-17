package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID;
//import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.Controllers.xBoxBrandon;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.swerve.swerveCommands.SwerveDriveMPSCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final XboxController driver = new XboxController(0);
    static final xBoxBrandon personalizedController =new xBoxBrandon(1);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = 4;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

    /* Subsystems */
    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        
        swerveSubsystem.setDefaultCommand(
            new SwerveDriveMPSCommand(
                swerveSubsystem,
                () -> MathUtil.applyDeadband(-driver.getRawAxis(translationAxis), Constants.Teleop.JOYSTICK_DEADBAND)*Constants.Teleop.MAX_SPEED_M_S,
                () -> MathUtil.applyDeadband(driver.getRawAxis(strafeAxis), Constants.Teleop.JOYSTICK_DEADBAND)*Constants.Teleop.MAX_SPEED_M_S,
                () -> MathUtil.applyDeadband(driver.getRawAxis(rotationAxis), Constants.Teleop.JOYSTICK_DEADBAND)*Constants.Teleop.MAX_ANG_SPEED_RAD_S,
                () -> !robotCentric.getAsBoolean()
               
            )
            
        );

        //Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> swerveSubsystem.zeroGyro()));
        new JoystickButton(personalizedController, personalizedController.zeroGyroButton())
            .onTrue(new InstantCommand(() -> swerveSubsystem.zeroGyro()));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        //return new exampleAuto(s_Swerve);
        return null;
    }
    public void init(){

    }
}
