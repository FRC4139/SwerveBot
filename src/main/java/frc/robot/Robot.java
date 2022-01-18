// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
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
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private XboxController controller;
  private WPI_TalonFX steerTalon, driveTalon;
  private SlewRateLimiter driveRateLimiter;
  private SlewRateLimiter rotationRateLimiter;
  private SwerveModuleController testModule;
  private CANCoder testCanCoder; 
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
    testModule =  new SwerveModuleController(steerTalon, driveTalon);
    testCanCoder = new CANCoder(40);
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
    SmartDashboard.putNumber("Detected Angle", testCanCoder.getAbsolutePosition());

    testModule.SetTargetAngleAndSpeed(targetAngle, driveInput, testCanCoder.getAbsolutePosition());
    System.out.println("Set angle and speed");
    // double driveSpeed = driveRateLimiter.calculate(MathUtil.applyDeadband(controller.getLeftY(), 0.05));
    // double rotationSpeed = rotationRateLimiter.calculate(MathUtil.applyDeadband(controller.getRightY(), 0.05));
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