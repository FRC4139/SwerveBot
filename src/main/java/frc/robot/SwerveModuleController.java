package frc.robot;
import com.ctre.phoenix.motorcontrol.can.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModuleController {
    private WPI_TalonFX steerFalcon, driveFalcon;

    private double targetAngle; // 0 - 360
    private double rotationSpeed, driveSpeed; 

    private static final float PROPORTION_ROTATION_CONSTANT = 0.004f; 

    public SwerveModuleController(WPI_TalonFX steer, WPI_TalonFX drive) { 
        steerFalcon = steer;
        driveFalcon = drive; 
    }

    public void SetTargetAngleAndSpeed(double angle, double speed, double currentAngle) { 
        targetAngle = angle;
        driveSpeed = speed; 
        
        //steerFalcon.set(rotationSpeed);
        driveFalcon.set(driveSpeed);
        
        // 4 test cases
        // if we want to spin from current 350 to target 10, ccw (+ = ccw) (10 - 350) WANT THIS TO BE 20
        // if we want to spin from curent 20 to target 100, ccw (+ = ccw) 
        // if we want to spin from current 20 to target 340, cw (- = cw) 
        // if we want to spin from current 270 to target 260, cw (- = cw)
        
        rotationSpeed = targetAngle - currentAngle;
        if (Math.abs(rotationSpeed)>180) {
            rotationSpeed -= 360 * Integer.signum((int)rotationSpeed);
        }

        
        if (rotationSpeed > 0) {
            steerFalcon.set(rotationSpeed * PROPORTION_ROTATION_CONSTANT + 0.05);
        } else {
            steerFalcon.set(rotationSpeed * PROPORTION_ROTATION_CONSTANT - 0.05);
        }
        
        SmartDashboard.putNumber("rotationSpeed", rotationSpeed * PROPORTION_ROTATION_CONSTANT);
        
        
    }




}
