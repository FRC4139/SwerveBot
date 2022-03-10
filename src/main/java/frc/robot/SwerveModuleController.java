package frc.robot;
import com.ctre.phoenix.motorcontrol.can.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModuleController {
    private WPI_TalonFX steerFalcon, driveFalcon;

    private double targetAngle; // 0 - 360
    private double rotationSpeed, driveSpeed; 
    private double offset = 0; 
    private static final float PROPORTION_ROTATION_CONSTANT = 0.0020f; 

    public SwerveModuleController(WPI_TalonFX steer, WPI_TalonFX drive) { 
        steerFalcon = steer;
        driveFalcon = drive; 
    }
    public SwerveModuleController(WPI_TalonFX steer, WPI_TalonFX drive, double Ioffset) { 
        steerFalcon = steer;
        driveFalcon = drive; 
        offset = Ioffset; 
    }
    public void SetOffset(double newOffset) {
        offset = newOffset;
    }
    public void SetTargetAngleAndSpeed(double angle, double speed, double currentAngle) { 
        SmartDashboard.putNumber("current module angle", currentAngle);
        targetAngle = angle;
        driveSpeed = speed; 
        currentAngle += offset; 
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
            steerFalcon.set(rotationSpeed * PROPORTION_ROTATION_CONSTANT + 0.1);
        } else {
            steerFalcon.set(rotationSpeed * PROPORTION_ROTATION_CONSTANT - 0.01);
        }
        
        SmartDashboard.putNumber("rotationSpeed", rotationSpeed * PROPORTION_ROTATION_CONSTANT);
        
        
    }




}
