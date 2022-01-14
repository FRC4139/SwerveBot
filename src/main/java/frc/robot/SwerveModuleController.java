package frc.robot;
import com.ctre.phoenix.motorcontrol.can.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModuleController {
    private WPI_TalonFX steerFalcon, driveFalcon;

    private double targetAngle; // 0 - 360
    private double rotationSpeed, driveSpeed; 

    private static final float proportional_rotation_constant = 0.002f; 

    public SwerveModuleController(WPI_TalonFX steer, WPI_TalonFX drive) { 
        steerFalcon = steer;
        driveFalcon = drive; 
    }

    public void SetTargetAngleAndSpeed(double angle, double speed, double encoderAngle) { 
        targetAngle = angle;
        driveSpeed = speed; 
        
        //steerFalcon.set(rotationSpeed);
        driveFalcon.set(driveSpeed);
        if (Math.abs(targetAngle - encoderAngle) > 1) {
            rotationSpeed = (targetAngle - encoderAngle) * proportional_rotation_constant;  
            steerFalcon.set(rotationSpeed);
            SmartDashboard.putNumber("rotationSpeed", rotationSpeed);
        }
        
    }




}
