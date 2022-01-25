// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private static final double normalization = 0.5;

  private static final double wheelBase = 0.0; // distance between centers of wheels on the same side
  private static final double trackWidth = 0.0; // distance between centers of wheels on opposite sides
  private static final Translation2d locationFL = new Translation2d(wheelBase / 2, trackWidth / 2);
  private static final Translation2d locationFR = new Translation2d(wheelBase / 2, -trackWidth / 2);
  private static final Translation2d locationBL = new Translation2d(-wheelBase / 2, trackWidth / 2);
  private static final Translation2d locationBR = new Translation2d(-wheelBase / 2, -trackWidth / 2);

  private static final boolean FIELD_RELATIVE = false;
  private static final double PROPORTION_SPEED_CONSTANT = 0.1;

  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private XboxController controller;
  private WPI_TalonFX steerTalon, driveTalon;
  private SlewRateLimiter driveRateLimiter;
  private SlewRateLimiter rotationRateLimiter;
  //private SwerveModuleController testModule;
  private SwerveModuleController moduleFL, moduleFR, moduleBL, moduleBR;
  // private CANCoder testCanCoder; 
  private CANCoder canCoderFL, canCoderFR, canCoderBL, canCoderBR;

  private SwerveDriveKinematics kinematics;
  private SwerveDriveOdometry odometry;
  private Gyro gyro; // NEED GYRO

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    // m_chooser.addOption("My Auto", kCustomAuto);
    // SmartDashboard.putData("Auto choices", m_chooser);
    controller = new XboxController(0);
    steerTalon = new WPI_TalonFX(53);
    driveTalon = new WPI_TalonFX(52);
    driveRateLimiter = new SlewRateLimiter(3);
    rotationRateLimiter = new SlewRateLimiter(3);
    // testModule =  new SwerveModuleController(steerTalon, driveTalon);
    moduleFL = new SwerveModuleController(new WPI_TalonFX(53), new WPI_TalonFX(52));
    moduleFR = new SwerveModuleController(new WPI_TalonFX(55), new WPI_TalonFX(54));
    moduleBL = new SwerveModuleController(new WPI_TalonFX(57), new WPI_TalonFX(56));
    moduleBR = new SwerveModuleController(new WPI_TalonFX(59), new WPI_TalonFX(58));
    // testCanCoder = new CANCoder(40);
    canCoderFL = new CANCoder(40);
    canCoderFR = new CANCoder(41);
    canCoderBL = new CANCoder(42);
    canCoderBR = new CANCoder(43);

    // TODO: gyro

    kinematics = new SwerveDriveKinematics(locationFL, locationFR, locationBL, locationBR);
    odometry = new SwerveDriveOdometry(kinematics, gyro.getRotation2d());
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}
  private double targetAngle = 0; 
  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    System.out.println("Left Y:" + controller.getLeftY() + " | Right Y: " + controller.getRightY());

    double turnInput = controller.getRightY();
    double driveInput = controller.getLeftY();
    SmartDashboard.putNumber("turn input raw", turnInput);
    SmartDashboard.putNumber("drive input raw", driveInput);
    if (Math.abs(controller.getRightY()) < 0.12) turnInput = 0;
    if (Math.abs(controller.getLeftY()) < 0.12) driveInput = 0; 
    targetAngle += turnInput * 4;
    SmartDashboard.putNumber("turn input", turnInput);
    SmartDashboard.putNumber("drive input", driveInput);
    if (targetAngle > 360) targetAngle -= 360; 
    if (targetAngle < 0) targetAngle += 360;

    // output is -1 to 1 (steering speed)

    SmartDashboard.putNumber("Target Angle", targetAngle);
    //SmartDashboard.putNumber("Detected Angle", testCanCoder.getAbsolutePosition());

    //testModule.SetTargetAngleAndSpeed(targetAngle, driveInput, testCanCoder.getAbsolutePosition());
    System.out.println("Set angle and speed");
    // double driveSpeed = driveRateLimiter.calculate(MathUtil.applyDeadband(controller.getLeftY(), 0.05));
    // double rotationSpeed = rotationRateLimiter.calculate(MathUtil.applyDeadband(controller.getRightY(), 0.05));


    double forward = -MathUtil.applyDeadband(controller.getLeftY(), 0.12);
    double strafe = -MathUtil.applyDeadband(controller.getLeftX(), 0.12);
    double rotation = -MathUtil.applyDeadband(controller.getRightX(), 0.12);

    // falcon 500 w/ talon fx max speed: 6380 RPM
    ChassisSpeeds speeds = new ChassisSpeeds(forward, strafe, rotation);
    SwerveModuleState states[] = kinematics.toSwerveModuleStates(speeds);
    System.out.println("speed: " + states[0].speedMetersPerSecond);
    //moduleFL.SetTargetAngleAndSpeed(states[0].angle.getDegrees(), states[0].speedMetersPerSecond, canCoderFL.getAbsolutePosition());
    //moduleFR.SetTargetAngleAndSpeed(states[1].angle.getDegrees(), states[1].speedMetersPerSecond, canCoderFR.getAbsolutePosition());
    //moduleFL.SetTargetAngleAndSpeed(states[2].angle.getDegrees(), states[2].speedMetersPerSecond, canCoderBL.getAbsolutePosition());
    //moduleFL.SetTargetAngleAndSpeed(states[3].angle.getDegrees(), states[3].speedMetersPerSecond, canCoderBR.getAbsolutePosition());
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
