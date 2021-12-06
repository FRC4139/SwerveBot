package frc.robot;
import com.ctre.phoenix.motorcontrol.can.*;

public class SwerveModuleController {
    private WPI_TalonFX steerFalcon, driveFalcon;

    private float targetAngle; // 0 - 360
    private float rotationSpeed, driveSpeed; 

    private static final float proportional_rotation_constant = 0.15f; 

    public SwerveModuleController(WPI_TalonFX steer, WPI_TalonFX drive) { 
        steerFalcon = steer;
        driveFalcon = drive; 
    }

    public void SetTargetAngleAndSpeed(float angle, float speed) { 
        targetAngle = angle;
        driveSpeed = speed; 
        
        steerFalcon.set(rotationSpeed);
        driveFalcon.set(driveSpeed);
    }

    public void UpdateRotation(float encoderAngle) {
        if (Math.abs(targetAngle - encoderAngle) > 1) {
            rotationSpeed = (targetAngle - encoderAngle) * proportional_rotation_constant;  
        }
    }


}
