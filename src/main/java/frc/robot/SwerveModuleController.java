package frc.robot;
import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModuleController {
    private WPI_TalonFX steerFalcon, driveFalcon;
    private CANCoder encoder;

    private double targetAngle; // 0 - 360
    private double rotationSpeed, driveSpeed; 
    private double offset = 0; 
    
    private static final float PROPORTION_ROTATION_CONSTANT = 0.0020f; 
    private static final float MINIMUM_ROTATION_SPEED = 0.025f;
    public SwerveModuleController(WPI_TalonFX steer, WPI_TalonFX drive) { 
        steerFalcon = steer;
        driveFalcon = drive; 
    }
    public SwerveModuleController(WPI_TalonFX steer, WPI_TalonFX drive, CANCoder iCoder, double Ioffset) { 
        steerFalcon = steer;
        driveFalcon = drive; 
        offset = Ioffset; 
        encoder = iCoder;
    }
    public void SetOffset(double newOffset) {
        offset = newOffset;
    }
    public void SetTargetAngleAndSpeed(double angle, double speed) { 
        
        double currentAngle = encoder.getAbsolutePosition() + offset; 

        targetAngle = angle;
        driveSpeed = speed; 
                
        
        
        // 4 test cases
        // if we want to spin from current 350 to target 10, ccw (+ = ccw) (10 - 350) WANT THIS TO BE 20
        // if we want to spin from curent 20 to target 100, ccw (+ = ccw) 
        // if we want to spin from current 20 to target 340, cw (- = cw) 
        // if we want to spin from current 270 to target 260, cw (- = cw)
        
        rotationSpeed = targetAngle - currentAngle;
        if (Math.abs(rotationSpeed)>180) {
            rotationSpeed -= 360 * Integer.signum((int)rotationSpeed);
        }

        driveFalcon.set(driveSpeed);
        if (rotationSpeed > 0) {
            steerFalcon.set(rotationSpeed * PROPORTION_ROTATION_CONSTANT + MINIMUM_ROTATION_SPEED);
        } else {
            steerFalcon.set(rotationSpeed * PROPORTION_ROTATION_CONSTANT - MINIMUM_ROTATION_SPEED);
        }
                
    }
    public double GetOffset() {
        return offset;
    }




}
