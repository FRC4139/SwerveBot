// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  

  private static final double wheelBase = 0.5716; // distance between centers of wheels on the same side
  private static final double trackWidth = 0.5716; // distance between centers of wheels on opposite sides
  private static final Translation2d locationFL = new Translation2d(wheelBase / 2, trackWidth / 2);
  private static final Translation2d locationFR = new Translation2d(wheelBase / 2, -trackWidth / 2);
  private static final Translation2d locationBL = new Translation2d(-wheelBase / 2, trackWidth / 2);
  private static final Translation2d locationBR = new Translation2d(-wheelBase / 2, -trackWidth / 2);

  //private static final double MAX_SPEED_MS = 0;
  private static final double MAX_SPEED_MS = 2.0; // placeholder

  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private XboxController controller;
  public WPI_TalonFX shootTalon, magazineTalon, lifterTalon; 
  private SlewRateLimiter driveRateLimiter;
  private SlewRateLimiter rotationRateLimiter;

  public WPI_TalonSRX intakeTalon;

  private WPI_Pigeon2 pigeon;

  public SwerveModuleController moduleFL, moduleFR, moduleBR, moduleBL;

  private CANCoder canCoderFL, canCoderFR, canCoderBR, canCoderBL;
  private double[] offsets; 
  private SwerveDriveKinematics kinematics;
  private SwerveDriveOdometry odometry;

  // CALIBRATION CODE
  private int selectedModule; 
  private SwerveModuleController[] modules;
  private NetworkTable limelightTable;
  private NetworkTableEntry tx; // horizontal offset from crosshair to target (-27deg to 27deg)
  private NetworkTableEntry ty; // vertical offset from crosshair to target (-20.5deg to 20.5deg)
  private NetworkTableEntry ta; // target area of image (0% of image to 100% of image)

  private double prevX = 0, prevY = 0;
  private double lifterLockValue = 0; 
  //Servo
  Servo exampleServo = new Servo(0);
  Turret turret; 
  private DriveController driveController; 
  private AutonomousHandler autonomousHandler;
  Timer time; 

  private boolean fieldOriented;
  private double gyroOffset = 0; 
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
    intakeTalon = new WPI_TalonSRX(3);
    pigeon = new WPI_Pigeon2(2); // Pigeon IMU (gyro)
  
    driveRateLimiter = new SlewRateLimiter(3);
    rotationRateLimiter = new SlewRateLimiter(3);

    // KEEP THESE BETWEEN -180 and 180 ??? TEST THIS
    //                     FL   FR   BR   BL
    offsets = new double[]{-60.008, -114.44, 167.018, -14.128};
    canCoderFL = new CANCoder(44);
    canCoderFR = new CANCoder(46);
    canCoderBR = new CANCoder(40);
    canCoderBL = new CANCoder(42);

    moduleFL = new SwerveModuleController("FL", new WPI_TalonFX(54), new WPI_TalonFX(32), canCoderFL, offsets[0]);
    moduleFR = new SwerveModuleController("FR", new WPI_TalonFX(56), new WPI_TalonFX(57), canCoderFR, offsets[1]);
    moduleBR = new SwerveModuleController("BR", new WPI_TalonFX(50), new WPI_TalonFX(51), canCoderBR, offsets[2]);
    moduleBL = new SwerveModuleController("BL", new WPI_TalonFX(52), new WPI_TalonFX(53), canCoderBL, offsets[3]);

    driveController = new DriveController(moduleFL, moduleFR, moduleBR, moduleBL);
    
    time = new Timer();

    kinematics = new SwerveDriveKinematics(locationFL, locationFR, locationBR, locationBL);
    odometry = new SwerveDriveOdometry(kinematics, pigeon.getRotation2d());
    autonomousHandler = new AutonomousHandler(kinematics, this); 

    modules = new SwerveModuleController[]{moduleFL, moduleFR, moduleBR, moduleBL};

    limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    tx = limelightTable.getEntry("tx");
    ty = limelightTable.getEntry("ty");
    ta = limelightTable.getEntry("ta");

    limelightTable.getEntry("ledMode").setNumber(3); // always on
    limelightTable.getEntry("camMode").setNumber(0); // vision processor (not driver camera)

    fieldOriented = true;
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
    if (time.get() == 0) {
      time.start();
    }
    
    double limeX = tx.getDouble(0.0);
    double limeY = ty.getDouble(0.0);
    double limeArea = ta.getDouble(0.0);

    if(limeX != 0) prevX = limeX;
    if(limeY != 0) prevY = limeY;

    autonomousHandler.Update(time.get(), Parallax.getDistanceToTarget(Math.toRadians(prevX), Math.toRadians(prevY)));

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
    if(controller.getBackButtonPressed())
      fieldOriented = !fieldOriented;
    
    if (controller.getAButton()) { 
      magazineTalon.set(0.5);
    } else if (controller.getYButton()) { 
      magazineTalon.set(-1);
    } else magazineTalon.set(0);

    // Lifter only adjustable when both bumpers are pressed
    if (controller.getLeftBumper() && controller.getRightBumper()) { 

      if (controller.getLeftTriggerAxis() > 0.25) lifterTalon.set(0.25);
      else if (controller.getRightTriggerAxis() > 0.25) lifterTalon.set(-0.25); 
      else lifterTalon.set(0);     

    } else {

      // Lifter locks in position when bumpers are not pressed
      if ( (controller.getLeftBumperReleased() && controller.getRightBumper()) || (controller.getRightBumperReleased() && 
          controller.getLeftBumper()) || (controller.getLeftBumperReleased() && controller.getRightBumperReleased()) ) {

        lifterLockValue = lifterTalon.getSensorCollection().getIntegratedSensorPosition();
      }
      
      // Active locking mechanism
      if (Math.abs(lifterLockValue - lifterTalon.getSensorCollection().getIntegratedSensorPosition()) > 1024) {
        lifterTalon.set(0.1 * Math.signum(lifterLockValue - lifterTalon.getSensorCollection().getIntegratedSensorPosition()));
      } else {
        lifterTalon.set(0);
      }
      
      // Turret, intake, and vision tracking are only adjustable when both bumpers are not pressed at the same time
      if (controller.getXButton()) turret.turn(-0.1);
      else if (controller.getBButton()) turret.turn(0.1);
      else turret.turn(0);

      if(controller.getLeftTriggerAxis() > 0.25) {
        intakeTalon.set(-controller.getLeftTriggerAxis() * 0.8);
      } else intakeTalon.set(0);
      
      ProcessLockOn();
    } 

    if (!turret.isCalibrated) {
      turret.calibrate();
    }

    // falcon 500 w/ talon fx max speed: 6380 RPM
   

    double forward = driveRateLimiter.calculate(-MathUtil.applyDeadband(controller.getLeftY(), 0.2));
    double strafe = rotationRateLimiter.calculate(-MathUtil.applyDeadband(controller.getLeftX(), 0.2));
    double rotation = -MathUtil.applyDeadband(controller.getRightX(), 0.2);
    SmartDashboard.putNumber("forward", forward);
    SmartDashboard.putNumber("strafe", strafe);
    SmartDashboard.putNumber("rotation", rotation);
    forward *= 0.5f;
    strafe *= 0.5f;

    double yaw = pigeon.getYaw();
    if (controller.getRightStickButtonReleased()) pigeon.setYaw(0); 
    SmartDashboard.putNumber("Gyro", yaw);

    ChassisSpeeds notFieldRelativeSpeeds = new ChassisSpeeds(forward * MAX_SPEED_MS, strafe * MAX_SPEED_MS, rotation);
    ChassisSpeeds fieldRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(forward * MAX_SPEED_MS, strafe * MAX_SPEED_MS, rotation, pigeon.getRotation2d());

    SwerveModuleState states[];
    if(fieldOriented)
      states = kinematics.toSwerveModuleStates(fieldRelativeSpeeds);
    else
      states = kinematics.toSwerveModuleStates(notFieldRelativeSpeeds);
    SmartDashboard.putBoolean("Field Relative?", fieldOriented);
    moduleFL.SetTargetAngleAndSpeed(states[0].angle.getDegrees(), states[0].speedMetersPerSecond);
    moduleFR.SetTargetAngleAndSpeed(states[1].angle.getDegrees(), states[1].speedMetersPerSecond);
    moduleBR.SetTargetAngleAndSpeed(states[2].angle.getDegrees(), states[2].speedMetersPerSecond);
    moduleBL.SetTargetAngleAndSpeed(states[3].angle.getDegrees(), states[3].speedMetersPerSecond);
    
    //driveController.Drive(rotation, strafe, forward, Math.toRadians(yaw));
    
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
    double turnInput = MathUtil.applyDeadband(controller.getLeftY(), 0.2); 

    if (controller.getBButton() && turnInput != 0) offsets[selectedModule] +=  turnInput * Math.abs(turnInput) + Math.signum(turnInput) * 0.1; 

    modules[selectedModule].SetOffset(offsets[selectedModule]);

    if (offsets[selectedModule] < -180) offsets[selectedModule] += 360;
    if (offsets[selectedModule] > 180) offsets[selectedModule] -= 360;

    // map selected module number to a string 0 = FL, 1 = FR, 2 = BR, 3 = BL
    String[] moduleNames = {"FL", "FR", "BR", "BL"};
    
    SmartDashboard.putString("Selected Module", moduleNames[selectedModule]);

    double forward = 0.05;
    double strafe = 0; 
    double rotation = 0; 
    ChassisSpeeds notFieldRelativeSpeeds = new ChassisSpeeds(forward * MAX_SPEED_MS, strafe * MAX_SPEED_MS, rotation);
    
    SwerveModuleState states[] = kinematics.toSwerveModuleStates(notFieldRelativeSpeeds);

    moduleFL.SetTargetAngleAndSpeed(states[0].angle.getDegrees(), states[0].speedMetersPerSecond);
    moduleFR.SetTargetAngleAndSpeed(states[1].angle.getDegrees(), states[1].speedMetersPerSecond);
    moduleBR.SetTargetAngleAndSpeed(states[2].angle.getDegrees(), states[2].speedMetersPerSecond);
    moduleBL.SetTargetAngleAndSpeed(states[3].angle.getDegrees(), states[3].speedMetersPerSecond);
    
    //print offsets to three decimal places
    SmartDashboard.putString("offsetString", "{" + Math.round(offsets[0] * 1000.0) / 1000.0 + ", " + Math.round(offsets[1] * 1000.0) / 1000.0 
      + ", " + Math.round(offsets[2] * 1000.0) / 1000.0 + ", " + Math.round(offsets[3] * 1000.0) / 1000.0 + "}");
  
    double limeX = tx.getDouble(0.0);
    double limeY = ty.getDouble(0.0);
    double limeArea = ta.getDouble(0.0);
    double distance = Parallax.getDistanceToTarget(limeX, limeY);
    SmartDashboard.putNumber("Distance", distance);
    if(limeX != 0) prevX = limeX;
    if(limeY != 0) prevY = limeY;

    }
  private double[] distances = new double[]{0,0,0,0,0,0,0,0,0};
  private void ProcessLockOn() {
    double limeX = tx.getDouble(0.0);
    double limeY = ty.getDouble(0.0);
    double limeArea = ta.getDouble(0.0);
    if(limeX != 0) prevX = limeX;
    if(limeY != 0) prevY = limeY;
    double distance = Parallax.getDistanceToTarget(Math.toRadians(prevX),Math.toRadians(prevY));
    SmartDashboard.putNumber("Distance", distance);
    

    for (int i = 0; i < 8; i++) distances[i] = distances[i+1];
    distances[8] = distance; 
    

    if(controller.getRightTriggerAxis() > 0.75) {
      turret.lockOn(limeX);
      //magazineTalon.set(-0.4);
      double speed = -0.32 - (medianCal(9, distances) * 0.0374);
      if (speed < -1.0) speed = -1.0; 
      
      shootTalon.set(speed);
      SmartDashboard.putNumber("shooter speed", shootTalon.get()); 
      SmartDashboard.putBoolean("isLockingOn", true);
    } else {
      SmartDashboard.putBoolean("isLockingOn", false);
      //magazineTalon.set(0);
      shootTalon.set(0);
    }
    

    SmartDashboard.putNumber("Limelight X", prevX);
    SmartDashboard.putNumber("Limelight Y", prevY);
    SmartDashboard.putNumber("Limelight Area", ta.getDouble(0.0));
    //SmartDashboard.putNumber("Distance", Parallax.getDistanceToTarget(Math.toRadians(prevX),Math.toRadians(prevY)));

  }

  public void ProcessLockOnAutonomous() {
    double limeX = tx.getDouble(0.0);
    double limeY = ty.getDouble(0.0);
    double limeArea = ta.getDouble(0.0);
    double distance = Parallax.getDistanceToTarget(limeX, limeY);
    if(limeX != 0) prevX = limeX;
    if(limeY != 0) prevY = limeY;
    
    if (limeX != 0 && limeY != 0) {
      turret.lockOn(limeX);
      shootTalon.set(-0.338 - distance * 0.0374);
    } else {
      turret.searchForTarget();
      shootTalon.set(0); 
    }
        

    SmartDashboard.putNumber("Limelight X", prevX);
    SmartDashboard.putNumber("Limelight Y", prevY);
    SmartDashboard.putNumber("Limelight Area", ta.getDouble(0.0));
    SmartDashboard.putNumber("Distance", Parallax.getDistanceToTarget(Math.toRadians(prevX),Math.toRadians(prevY)));

  }
  private double medianCal(int n,double in[])
    {
      double m=0;	
      
      if(n%2==1)
      {
        m=in[((n+1)/2)-1];
        
      }
      else
      {
        m=(in[n/2-1]+in[n/2])/2;
        
      }
    return m;
      
    }
}