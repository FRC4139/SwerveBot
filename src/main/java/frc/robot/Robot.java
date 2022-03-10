// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.xml.crypto.dsig.keyinfo.KeyValue;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Servo;
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
  private static final double normalization = 0.5;

  private static final double wheelBase = 0.63; // distance between centers of wheels on the same side
  private static final double trackWidth = 0.47; // distance between centers of wheels on opposite sides
  private static final Translation2d locationFL = new Translation2d(wheelBase / 2, trackWidth / 2);
  private static final Translation2d locationFR = new Translation2d(wheelBase / 2, -trackWidth / 2);
  private static final Translation2d locationBL = new Translation2d(-wheelBase / 2, trackWidth / 2);
  private static final Translation2d locationBR = new Translation2d(-wheelBase / 2, -trackWidth / 2);

  //private static final double MAX_SPEED_MS = 0;
  private static final double MAX_SPEED_MS = 2.0; // placeholder

  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private XboxController controller;
  private WPI_TalonFX shootTalon, magazineTalon, lifterTalon; 
  private SlewRateLimiter driveRateLimiter;
  private SlewRateLimiter rotationRateLimiter;

  private SwerveModuleController moduleFL, moduleFR, moduleBR, moduleBL;

  private CANCoder canCoderFL, canCoderFR, canCoderBR, canCoderBL;
  private double[] offsets; 
  private SwerveDriveKinematics kinematics;
  private SwerveDriveOdometry odometry;
  private AHRS ahrs; //gyro
  // CALIBRATION CODE
  private int selectedModule; 
  private SwerveModuleController[] modules;
  private NetworkTable limelightTable;
  private NetworkTableEntry tx; // horizontal offset from crosshair to target (-27deg to 27deg)
  private NetworkTableEntry ty; // vertical offset from crosshair to target (-20.5deg to 20.5deg)
  private NetworkTableEntry ta; // target area of image (0% of image to 100% of image)

  private double prevX = 0, prevY = 0;

  //Servo
  Servo exampleServo = new Servo(0);
  Turret turret; 
  
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

    magazineTalon = new WPI_TalonFX(33);
    shootTalon = new WPI_TalonFX(34);
    turret = new Turret(35, 0); 
    lifterTalon = new WPI_TalonFX(37);
    lifterTalon.setNeutralMode(NeutralMode.Brake);

  
    driveRateLimiter = new SlewRateLimiter(3);
    rotationRateLimiter = new SlewRateLimiter(3);

    // KEEP THESE BETWEEN -180 and 180 ??? TEST THIS
    //                     FL   FR   BR   BL
    offsets = new double[]{-52.3,-95.4,55.5,-36};

    canCoderFL = new CANCoder(44);
    canCoderFR = new CANCoder(46);
    canCoderBR = new CANCoder(40);
    canCoderBL = new CANCoder(42);

    moduleFL = new SwerveModuleController(new WPI_TalonFX(54), new WPI_TalonFX(32), canCoderFL, offsets[0]);
    moduleFR = new SwerveModuleController(new WPI_TalonFX(56), new WPI_TalonFX(57), canCoderFR, offsets[1]);
    moduleBR = new SwerveModuleController(new WPI_TalonFX(50), new WPI_TalonFX(51), canCoderBR, offsets[2]);
    moduleBL = new SwerveModuleController(new WPI_TalonFX(52), new WPI_TalonFX(53), canCoderBL, offsets[3]);

    
    
    
    // gyro
    ahrs = new AHRS(SPI.Port.kMXP);
    ahrs.calibrate();
    ahrs.setAngleAdjustment(90);

    kinematics = new SwerveDriveKinematics(locationFL, locationFR, locationBR, locationBL);
    odometry = new SwerveDriveOdometry(kinematics, ahrs.getRotation2d());
    
    modules = new SwerveModuleController[]{moduleFL, moduleFR, moduleBR, moduleBL};

    limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    tx = limelightTable.getEntry("tx");
    ty = limelightTable.getEntry("ty");
    ta = limelightTable.getEntry("ta");

    limelightTable.getEntry("ledMode").setNumber(3); // always on
    limelightTable.getEntry("camMode").setNumber(0); // vision processor (not driver camera)
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {

  }

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
    //m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    //System.out.println("Auto selected: " + m_autoSelected);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    /*
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
    */

    
    double limeX = tx.getDouble(0.0);
    double limeY = ty.getDouble(0.0);
    double limeArea = ta.getDouble(0.0);

    if(limeX != 0) prevX = limeX;
    if(limeY != 0) prevY = limeY;

    if(limeArea > 15) {
      ChassisSpeeds speeds = new ChassisSpeeds(0, 0, prevX < 0 ? -(Math.PI / 12) : (Math.PI / 12));
      SwerveModuleState states[] = kinematics.toSwerveModuleStates(speeds);
      moduleFL.SetTargetAngleAndSpeed(states[0].angle.getDegrees(), states[0].speedMetersPerSecond);
      moduleFR.SetTargetAngleAndSpeed(states[1].angle.getDegrees(), states[1].speedMetersPerSecond);
      moduleBR.SetTargetAngleAndSpeed(states[2].angle.getDegrees(), states[2].speedMetersPerSecond);
      moduleBL.SetTargetAngleAndSpeed(states[3].angle.getDegrees(), states[3].speedMetersPerSecond);
    }

    if(controller.getYButtonPressed()) {
      turret.lockOn(limeX);
      SmartDashboard.putBoolean("isLockingOn", true);
    } else {
      SmartDashboard.putBoolean("isLockingOn", false);
    }

    SmartDashboard.putNumber("Limelight X", prevX);
    SmartDashboard.putNumber("Limelight Y", prevY);
    SmartDashboard.putNumber("Limelight Area", ta.getDouble(0.0));
    SmartDashboard.putNumber("Distance", Parallax.getDistanceToTarget(Math.toRadians(prevX),Math.toRadians(prevY)));
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    if(controller.getStartButtonPressed()) ahrs.calibrate();
    while(ahrs.isCalibrating()) {} // don't continue until calibrated // SUS

    // TURRET MOVEMENT
    if (!turret.isCalibrated) {
      turret.calibrate();
    } else {
      if (!(controller.getLeftBumper() && controller.getRightBumper())) {
        if (controller.getLeftTriggerAxis() > 0.25) turret.turn(controller.getLeftTriggerAxis() * Turret.MAX_SPEED);
        else if (controller.getRightTriggerAxis() > 0.25) turret.turn(-1 * controller.getRightTriggerAxis() * Turret.MAX_SPEED);
        else turret.turn(0);
      } else turret.turn(0);
    }

    // LIFTER MOVEMENT
    if (controller.getLeftBumper() && controller.getRightBumper()) { 
      if (controller.getLeftTriggerAxis() > 0.25) lifterTalon.set(0.25);
      else if (controller.getRightTriggerAxis() > 0.25) lifterTalon.set(-0.25); 
      else lifterTalon.set(0);     
    } else lifterTalon.set(0);

    // falcon 500 w/ talon fx max speed: 6380 RPM
    // ChassisSpeeds notFieldRelativeSpeeds = new ChassisSpeeds(forward * MAX_SPEED_MS, strafe * MAX_SPEED_MS, rotation);
    

    double forward = driveRateLimiter.calculate(-MathUtil.applyDeadband(controller.getLeftY(), 0.12));
    double strafe = rotationRateLimiter.calculate(-MathUtil.applyDeadband(controller.getLeftX(), 0.12));
    double rotation = -MathUtil.applyDeadband(controller.getRightX(), 0.12);
    forward *= 0.5f;
    strafe *= 0.5f;
    
    SmartDashboard.putNumber("Gyro", ahrs.getRotation2d().getDegrees());
    ChassisSpeeds frSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(forward * MAX_SPEED_MS, strafe * MAX_SPEED_MS, rotation, ahrs.getRotation2d());
    SwerveModuleState states[] = kinematics.toSwerveModuleStates(frSpeeds);

    SmartDashboard.putNumber("FL Module Speed", states[0].speedMetersPerSecond);
    SmartDashboard.putNumber("FL Module Angle", states[0].angle.getDegrees());

    moduleFL.SetTargetAngleAndSpeed(states[0].angle.getDegrees(), states[0].speedMetersPerSecond);
    moduleFR.SetTargetAngleAndSpeed(states[1].angle.getDegrees(), states[1].speedMetersPerSecond);
    moduleBR.SetTargetAngleAndSpeed(states[2].angle.getDegrees(), states[2].speedMetersPerSecond);
    moduleBL.SetTargetAngleAndSpeed(states[3].angle.getDegrees(), states[3].speedMetersPerSecond);
    
    ProcessLockOn();
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
  public void testPeriodic() {
    
    // CALIBRATION 
    if (controller.getYButtonPressed()) selectedModule++;
    if (controller.getAButtonPressed()) selectedModule--;
    selectedModule = selectedModule % 4;

    if (controller.getBButton()) offsets[selectedModule] += controller.getLeftY() * 0.25f;
    modules[selectedModule].SetOffset(offsets[selectedModule]);

    if (offsets[selectedModule] < -180) offsets[selectedModule] += 360;
    if (offsets[selectedModule] > 180) offsets[selectedModule] -= 360;

    // map selected module number to a string 0 = FL, 1 = FR, 2 = BR, 3 = BL
    String[] moduleNames = {"FL", "FR", "BR", "BL"};
    
    SmartDashboard.putString("Selected Module", moduleNames[selectedModule]);

    double forward = 0, strafe=0, rotation = 0; 
    if (controller.getBButton()) {
      forward = 0.05; 
      strafe = 0; 
      rotation = 0; 
    }

    ChassisSpeeds frSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(forward * MAX_SPEED_MS, strafe * MAX_SPEED_MS, rotation, new Rotation2d());
    SwerveModuleState states[] = kinematics.toSwerveModuleStates(frSpeeds);
    //var gyroAngle = Rotation2d.fromDegrees(ahrs.getAngle());
    moduleFL.SetTargetAngleAndSpeed(states[0].angle.getDegrees(), states[0].speedMetersPerSecond);
    moduleFR.SetTargetAngleAndSpeed(states[1].angle.getDegrees(), states[1].speedMetersPerSecond);
    moduleBR.SetTargetAngleAndSpeed(states[2].angle.getDegrees(), states[2].speedMetersPerSecond);
    moduleBL.SetTargetAngleAndSpeed(states[3].angle.getDegrees(), states[3].speedMetersPerSecond);
    
    SmartDashboard.putNumber("cancoder FL", canCoderFL.getAbsolutePosition());
    SmartDashboard.putNumber("cancoder FR", canCoderFR.getAbsolutePosition());
    SmartDashboard.putNumber("cancoder BR", canCoderBR.getAbsolutePosition());
    SmartDashboard.putNumber("cancoder BL", canCoderBL.getAbsolutePosition());
    
    SmartDashboard.putNumber("offset FL", moduleFL.GetOffset());
    SmartDashboard.putNumber("offset FR", moduleFR.GetOffset());
    SmartDashboard.putNumber("offset BR", moduleBR.GetOffset());
    SmartDashboard.putNumber("offset BL", moduleBL.GetOffset());
    
    SmartDashboard.putString("offsetString", "{" + moduleFL.GetOffset() + ", " + 
    moduleFR.GetOffset() + ", " + moduleBR.GetOffset() + ", " + moduleBL.GetOffset() + "}");
  }

  private void ProcessLockOn() {
    double limeX = tx.getDouble(0.0);
    double limeY = ty.getDouble(0.0);
    double limeArea = ta.getDouble(0.0);

    if(limeX != 0) prevX = limeX;
    if(limeY != 0) prevY = limeY;

    

    if(controller.getLeftTriggerAxis() > 0.75) {
      turret.lockOn(limeX);
      SmartDashboard.putBoolean("isLockingOn", true);
    } else {
      SmartDashboard.putBoolean("isLockingOn", false);
    }

    SmartDashboard.putNumber("Limelight X", prevX);
    SmartDashboard.putNumber("Limelight Y", prevY);
    SmartDashboard.putNumber("Limelight Area", ta.getDouble(0.0));
    SmartDashboard.putNumber("Distance", Parallax.getDistanceToTarget(Math.toRadians(prevX),Math.toRadians(prevY)));
    // if(limeArea > 15 && false) {
    //   ChassisSpeeds speeds = new ChassisSpeeds(0, 0, prevX < 0 ? -(Math.PI / 12) : (Math.PI / 12));
    //   SwerveModuleState states[] = kinematics.toSwerveModuleStates(speeds);
    //   moduleFL.SetTargetAngleAndSpeed(states[0].angle.getDegrees(), states[0].speedMetersPerSecond);
    //   moduleFR.SetTargetAngleAndSpeed(states[1].angle.getDegrees(), states[1].speedMetersPerSecond);
    //   moduleBR.SetTargetAngleAndSpeed(states[2].angle.getDegrees(), states[2].speedMetersPerSecond);
    //   moduleBL.SetTargetAngleAndSpeed(states[3].angle.getDegrees(), states[3].speedMetersPerSecond);
    // }
  }
}