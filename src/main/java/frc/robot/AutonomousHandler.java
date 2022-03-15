package frc.robot;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class AutonomousHandler {

    private SwerveDriveKinematics kinematics;
    private Robot robot;
    private double firstTargetReachedTime = 0;
    private double secondTargetReachedTime = 0;
    private final double TARGET_DISTANCE_FIRST_BALL = 6.5; // feet 
    private final double TARGET_DISTANCE_SECOND_BALL = 9; // feet
    private final double PROPORTION_DRIVE_CONSTANT = 1 / 15; 
    private int stage = -1; 

    // STAGES
    // 0: back up out of tarmac until distance from target is reached
    // 1: move magazine forward at max speed to shoot first ball
    // 2: turn intake on and back up out of tarmac until distance to intake second ball is reached
    // 3: sit with intake on for 3 seconds to make sure it is intaked
    // 4: move magazine forward at max speed to shoot second ball
    public AutonomousHandler(SwerveDriveKinematics kinematicsIn, Robot r) {
        kinematics = kinematicsIn;
        robot = r;
    }

    public void Update(double currentTime, double perceivedDistance) {
        if (stage <= 4 && stage >= 0) robot.ProcessLockOnAutonomous(); 
        else robot.shootTalon.set(0); 

        if (stage == -2) {
            if (!robot.turret.isCalibrated) {
                robot.turret.calibrate();
            } else {
                stage++; 
            }
        } else if (stage == -1) {
            if (robot.turret.getTurretPosition() < 40000) {
                robot.turret.turn(0.2);
            } else {
                stage++; 
                robot.turret.turn(0); 
            }
        } else if (stage == 0) {
            if (Math.abs(perceivedDistance - TARGET_DISTANCE_FIRST_BALL) > 0.25) {
                drive((perceivedDistance - TARGET_DISTANCE_FIRST_BALL) * PROPORTION_DRIVE_CONSTANT, 0, 0); 
            }
            else {
                stage++; 
                firstTargetReachedTime = currentTime; 
            }
        } else if (stage == 1) {
            if (currentTime - firstTargetReachedTime < 1) {
                drive(0,0,0); 
                robot.magazineTalon.set(-1);
            } else { 
                
                stage++; 
            }
        } else if (stage == 2) {
            if (Math.abs(perceivedDistance - TARGET_DISTANCE_SECOND_BALL) > 0.25) {
                drive((perceivedDistance - TARGET_DISTANCE_SECOND_BALL) * PROPORTION_DRIVE_CONSTANT,0,0);
                robot.intakeTalon.set(-0.8);
                robot.magazineTalon.set(0);
            } else {
                stage++; 
                secondTargetReachedTime = currentTime;
            }
        } else if (stage == 3) {
            if (currentTime - secondTargetReachedTime < 3) {
                drive(0,0,0); 
                robot.intakeTalon.set(0);
            } else {
                stage++; 
            }
        } else if (stage == 4) {
            if (currentTime - secondTargetReachedTime < 4) {
                drive(0,0,0); 
                robot.intakeTalon.set(0);
                robot.magazineTalon.set(-1);
            } else {
                stage++; 
            }
        } else {
            drive(0,0,0); 
            robot.intakeTalon.set(0);
            robot.magazineTalon.set(0);
        }



    }

    private void drive(double forward, double strafe, double rotation) {
        ChassisSpeeds notFieldRelativeSpeeds = new ChassisSpeeds(forward, strafe, rotation);

        SwerveModuleState states[] = kinematics.toSwerveModuleStates(notFieldRelativeSpeeds);

        robot.moduleFL.SetTargetAngleAndSpeed(states[0].angle.getDegrees(), states[0].speedMetersPerSecond);
        robot.moduleFR.SetTargetAngleAndSpeed(states[1].angle.getDegrees(), states[1].speedMetersPerSecond);
        robot.moduleBR.SetTargetAngleAndSpeed(states[2].angle.getDegrees(), states[2].speedMetersPerSecond);
        robot.moduleBL.SetTargetAngleAndSpeed(states[3].angle.getDegrees(), states[3].speedMetersPerSecond);
    }


}