package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Turret {
    private WPI_TalonFX turretTalon;
    private DigitalInput limitSwitchInput; 
    public boolean isCalibrated; 
    private static final int MAX_SENSOR_POSITION = 185000;
    private static final double MAX_SPEED = 0.2;
    public Turret(int tf, int ls) { 
        turretTalon = new WPI_TalonFX(tf); 
        limitSwitchInput = new DigitalInput(ls);
    }

    public void calibrate() { 
        // negative is towards switch
        // positive is away 
        if (limitSwitchInput.get()) {
            turretTalon.set(-0.1);
        } else {
            isCalibrated = true; 
            turretTalon.getSensorCollection().setIntegratedSensorPosition(0,0);
        }        
        SmartDashboard.putBoolean("turret calibrated", isCalibrated);
    }

    public void turn(double speed) {
        SmartDashboard.putNumber("input speed", speed);
        if (turretTalon.getSensorCollection().getIntegratedSensorPosition() > MAX_SENSOR_POSITION) {
            // only want negative
            speed = Math.min(speed, 0);
        } 
        if (!limitSwitchInput.get()) {
            // only want positive
            speed = Math.max(speed, 0);
        }
        if (speed > MAX_SPEED) speed = MAX_SPEED;
        if (speed < -MAX_SPEED) speed = -MAX_SPEED; 
        turretTalon.set(speed);
        SmartDashboard.putNumber("output speed", speed);
        SmartDashboard.putNumber("turret rotation raw", turretTalon.getSensorCollection().getIntegratedSensorPosition());
    }

    public void lockOn(double tx) {
        
        turn(MAX_SPEED*tx / 20);
    }
    
}
