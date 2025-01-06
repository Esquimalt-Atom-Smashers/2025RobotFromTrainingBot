package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PigeonGyro extends SubsystemBase{
    private static final double yawCalibrationValue = 3600.0/3566;
    Pigeon2 gyro = new Pigeon2(0);

    public double getPitch(){
        double pitch = gyro.getPitch().getValueAsDouble();
        return pitch;
    }
    public double getYaw(){
        double yaw = gyro.getYaw().getValueAsDouble()*yawCalibrationValue;
        return -yaw;//flip yaw direction to be clockwise positive
    }
    public double getRoll(){
        double roll = gyro.getRoll().getValueAsDouble();
        return roll;
    }
    public Rotation2d getYawRotation2d(){
        Rotation2d yaw = Rotation2d.fromDegrees(getYaw());
        return yaw;
    }
    public void reset(){
        //gyro.reset();
        gyro.setYaw(0);
    }
}
