package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Turret {
    public WPI_TalonFX turretTalon;
    private DigitalInput limitSwitchInput; 
    public boolean isCalibrated; 
    private boolean searchDirectionPositive; 

    private static final int MAX_SENSOR_POSITION = 185000;
    public static final double MAX_SPEED = 0.2;
    
    public Turret(int tf, int ls) { 
        turretTalon = new WPI_TalonFX(tf); 
        limitSwitchInput = new DigitalInput(ls);
    }

    /*
    public void calibrate() { 
        // negative is towards switch
        // positive is away 
        SmartDashboard.putBoolean("limit switch status", limitSwitchInput.get());
        if (limitSwitchInput.get()) {
            turretTalon.set(-0.075);
        } else {
            isCalibrated = true; 
            turretTalon.getSensorCollection().setIntegratedSensorPosition(0,0);
        }        
        SmartDashboard.putBoolean("turret calibrated", isCalibrated);
    }
    */

    public void calibrate() {
        // we start the turret in the correct position
        turretTalon.getSensorCollection().setIntegratedSensorPosition(0, 0);
        isCalibrated = true;
    }

    public void turn(double speed) {
        //SmartDashboard.putNumber("input speed", speed);
        if (turretTalon.getSensorCollection().getIntegratedSensorPosition() > MAX_SENSOR_POSITION) {
            // only want negative
            speed = Math.min(speed, 0);
            searchDirectionPositive = false;
        } 
        if (/*!limitSwitchInput.get() ||*/ turretTalon.getSensorCollection().getIntegratedSensorPosition() < 5000) {
            // only want positive
            speed = Math.max(speed, 0);
            searchDirectionPositive = true;
        }
        if (speed > MAX_SPEED) speed = MAX_SPEED;
        if (speed < -MAX_SPEED) speed = -MAX_SPEED; 
        turretTalon.set(speed);
        SmartDashboard.putNumber("turret output speed", speed);
        SmartDashboard.putNumber("turret rotation raw", turretTalon.getSensorCollection().getIntegratedSensorPosition());
    }

    public void lockOn(double tx) {
        double offset = -5;
        if (Math.abs(tx-offset) >= 1 && tx != 0) {
            turn(MAX_SPEED*(tx-offset) / 20 + Math.signum(tx-offset) * 0.05);
        } else {
            turn(0);
        }
        
    }

    public void searchForTarget() {
        if (searchDirectionPositive) {
            turn(MAX_SPEED);
        } else {
            turn(-MAX_SPEED);
        }
        if (turretTalon.getSensorCollection().getIntegratedSensorPosition() > MAX_SENSOR_POSITION) {
            searchDirectionPositive = false;
        } else if (turretTalon.getSensorCollection().getIntegratedSensorPosition() < 5000) {
            searchDirectionPositive = true;
        }
    }

    public void turnToTargetSensorPosition(int targetSensorPosition) { 
        double currentSensorPosition = turretTalon.getSensorCollection().getIntegratedSensorPosition();
        double error = targetSensorPosition - currentSensorPosition;
        if (error > 1000) {
            turn(MAX_SPEED);
        } else if (error < 1000) {
            turn(-MAX_SPEED);
        } else {
            turn(0);
        }
    }

    public double getTurretPosition() {
        return turretTalon.getSensorCollection().getIntegratedSensorPosition();
    }
    
}
