package frc.robot;
import com.ctre.phoenix.motorcontrol.can.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModuleController {
    private WPI_TalonFX steerFalcon, driveFalcon;

    private double targetAngle; // 0 - 360
    private double rotationSpeed, driveSpeed; 

    private static final float PROPORTION_ROTATION_CONSTANT = 0.002f; 

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
        
        if (targetAngle < currentAngle) {
            // if we want to spin from current 350 to target 10, ccw (+ = ccw) (10 - 350) WANT THIS TO BE 20
            // if we want to spin from current 270 to target 260, cw (- = cw)

            if (currentAngle - targetAngle < 180) { // if we want to spin from current 270 to target 260, cw (- = cw)
                rotationSpeed = (currentAngle - targetAngle) * -1; // should be negative, clockwise
            } else { // if we want to spin from current 350 to target 10, ccw (+ = ccw) (10 - 350) WANT THIS TO BE 20
                rotationSpeed = Math.abs(360 - currentAngle + targetAngle); //should be positive, counterclockwise
            }
        } else { // targetAngle > currentAngle
            // if we want to spin from curent 20 to target 100, ccw (+ = ccw) 
            // if we want to spin from current 20 to target 340, cw (- = cw) 

            if (targetAngle - currentAngle < 180) { // if we want to spin from curent 20 to target 100, ccw (+ = ccw) 
                rotationSpeed = (targetAngle - currentAngle); 
            } else { // if we want to spin from current 20 to target 340, cw (- = cw) 
                rotationSpeed = Math.abs(360 - targetAngle + currentAngle); 
            }
        }
        
        steerFalcon.set(rotationSpeed * PROPORTION_ROTATION_CONSTANT);
        SmartDashboard.putNumber("rotationSpeed", rotationSpeed * PROPORTION_ROTATION_CONSTANT);
        
        
    }




}
