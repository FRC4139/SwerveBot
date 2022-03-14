package frc.robot;

public class DriveController {

    private SwerveModuleController frontLeft, frontRight, backLeft, backRight;
    public DriveController(SwerveModuleController fr, SwerveModuleController fl, SwerveModuleController br, SwerveModuleController bl) {
        frontRight = fr;
        frontLeft = fl;
        backRight = br;
        backLeft = bl;
    }

    public void Drive(double rotationSpeed, double strafe, double forward, double yaw) {
        //double dx = forward; // dx is strafe, but this adjusts for the fact that the robot is facing the wrong way
        //double dz = -strafe; // dz is forward, but this adjusts for the fact that the robot is facing the wrong way
        // (0, 1) normally go left, but adjustment makes it (1, -0)
        double dx = strafe;
        double dz = forward;
        double[][] motions = SwerveMath.getDesiredAngleAndSpeed(rotationSpeed, dx, dz, yaw);
        frontRight.SetTargetAngleAndSpeed(motions[0][0], motions[0][1]);
        frontLeft.SetTargetAngleAndSpeed(motions[1][0], motions[1][1]);
        backLeft.SetTargetAngleAndSpeed(motions[2][0], motions[2][1]);
        backRight.SetTargetAngleAndSpeed(motions[3][0], motions[3][1]);
    }
}
