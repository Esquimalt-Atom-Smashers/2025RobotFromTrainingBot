package frc.robot.subsystems.swerve.swerveCommands;

import frc.robot.subsystems.swerve.SwerveSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;


public class SwerveDriveMPSCommand extends Command {    
    private SwerveSubsystem swerveSS;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier fieldCentricSup;

    public SwerveDriveMPSCommand(SwerveSubsystem subsystem, DoubleSupplier forward, DoubleSupplier strafe, DoubleSupplier rotationRadPerS, BooleanSupplier fieldCentric) {
        this.swerveSS = subsystem;
        addRequirements(subsystem);

        this.translationSup = forward;
        this.strafeSup = strafe;
        this.rotationSup = rotationRadPerS;
        this.fieldCentricSup = fieldCentric;
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double translationVal = translationSup.getAsDouble();
        double strafeVal = strafeSup.getAsDouble();
        double rotationVal = rotationSup.getAsDouble();

        /* Drive */
        swerveSS.driveMPS(
            new Translation2d(translationVal, strafeVal), 
            rotationVal, 
            fieldCentricSup.getAsBoolean()
        );
    }
}