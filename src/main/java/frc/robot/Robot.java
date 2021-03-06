// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;

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
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import io.github.pseudoresonance.pixy2api.Pixy2;
import io.github.pseudoresonance.pixy2api.Pixy2CCC;
import io.github.pseudoresonance.pixy2api.Pixy2CCC.Block;
import io.github.pseudoresonance.pixy2api.links.SPILink;

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

  private static final double wheelBase = 0.63; // distance between centers of wheels on the same side
  private static final double trackWidth = 0.47; // distance between centers of wheels on opposite sides
  private static final Translation2d locationFL = new Translation2d(wheelBase / 2, trackWidth / 2);
  private static final Translation2d locationFR = new Translation2d(wheelBase / 2, -trackWidth / 2);
  private static final Translation2d locationBL = new Translation2d(-wheelBase / 2, trackWidth / 2);
  private static final Translation2d locationBR = new Translation2d(-wheelBase / 2, -trackWidth / 2);

  private static final boolean FIELD_RELATIVE = false;
  //private static final double MAX_SPEED_MS = 5.4864;
  private static final double MAX_SPEED_MS = 2.0; // placeholder

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
  private double[] offsets; 
  private SwerveDriveKinematics kinematics;
  private SwerveDriveOdometry odometry;
  private AHRS ahrs; //gyro
  // CALIBRATION CODE
  private int selectedModule; 
  private SwerveModuleController[] modules; 
  private Pixy2 pixy; 
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
    moduleFL = new SwerveModuleController(new WPI_TalonFX(50), new WPI_TalonFX(51));
    moduleFR = new SwerveModuleController(new WPI_TalonFX(56), new WPI_TalonFX(57));
    moduleBL = new SwerveModuleController(new WPI_TalonFX(52), new WPI_TalonFX(53));
    moduleBR = new SwerveModuleController(new WPI_TalonFX(54), new WPI_TalonFX(55));
    // testCanCoder = new CANCoder(40);
    canCoderFL = new CANCoder(40);
    canCoderFR = new CANCoder(46);
    canCoderBL = new CANCoder(42);
    canCoderBR = new CANCoder(44);

    // gyro
    ahrs = new AHRS(SPI.Port.kMXP);
    ahrs.calibrate();

    kinematics = new SwerveDriveKinematics(locationFL, locationFR, locationBL, locationBR);
    odometry = new SwerveDriveOdometry(kinematics, ahrs.getRotation2d());
    offsets = new double[]{161.25,-109.5,-68,132.6};
    moduleFL.SetOffset(offsets[0]);
    moduleFR.SetOffset(offsets[1]);
    moduleBL.SetOffset(offsets[2]);
    moduleBR.SetOffset(offsets[3]);
    modules = new SwerveModuleController[]{moduleFL, moduleFR, moduleBL, moduleBR};
    pixy = Pixy2.createInstance(new SPILink()); // Creates a new Pixy2 camera using SPILink
		pixy.init(); // Initializes the camera and prepares to send/receive data
		pixy.setLamp((byte) 0, (byte) 0); // Turns the LEDs on
		//pixy.setLED(255, 0, 255); // Sets the RGB LED to full white
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
    //pixy.setLED(0, 255, 0); // green
    getBiggestBlock();
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
    //System.out.println("Left Y:" + controller.getLeftY() + " | Right Y: " + controller.getRightY());

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

    if(controller.getStartButtonPressed()) ahrs.calibrate();
    while(ahrs.isCalibrating()) {} // don't continue until calibrated
    
    if (controller.getYButtonPressed()) selectedModule++;
    if (controller.getAButtonPressed()) selectedModule--;
    selectedModule = Math.min(Math.max(selectedModule, 0), 3);
    if (controller.getBButton()) offsets[selectedModule] += turnInput * 0.25f;
    modules[selectedModule].SetOffset(offsets[selectedModule]);

    SmartDashboard.putNumber("selected", selectedModule);
    SmartDashboard.putNumber("selected offset", offsets[selectedModule]);
    // output is -1 to 1 (steering speed)

    SmartDashboard.putNumber("Target Angle", targetAngle);
    //SmartDashboard.putNumber("Detected Angle", testCanCoder.getAbsolutePosition());

    //testModule.SetTargetAngleAndSpeed(targetAngle, driveInput, testCanCoder.getAbsolutePosition());
    //System.out.println("Set angle and speed");
    // double driveSpeed = driveRateLimiter.calculate(MathUtil.applyDeadband(controller.getLeftY(), 0.05));
    // double rotationSpeed = rotationRateLimiter.calculate(MathUtil.applyDeadband(controller.getRightY(), 0.05));


    double forward = driveRateLimiter.calculate(-MathUtil.applyDeadband(controller.getLeftY(), 0.12));
    double strafe = rotationRateLimiter.calculate(-MathUtil.applyDeadband(controller.getLeftX(), 0.12));
    forward *= 0.5f;
    strafe *= 0.5f;
    
    double rotation = -MathUtil.applyDeadband(controller.getRightX(), 0.12);
    if (controller.getBButton()) {
      forward = 0.1; 
      strafe = 0; 
      rotation = 0; 
    }

    // falcon 500 w/ talon fx max speed: 6380 RPM
    // ChassisSpeeds speeds = new ChassisSpeeds(forward * MAX_SPEED_MS, strafe * MAX_SPEED_MS, rotation);
    SmartDashboard.putNumber("Gyro", ahrs.getRotation2d().getDegrees());
    ChassisSpeeds frSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(forward * MAX_SPEED_MS, strafe * MAX_SPEED_MS, rotation, ahrs.getRotation2d());
    SwerveModuleState states[] = kinematics.toSwerveModuleStates(frSpeeds);
    SmartDashboard.putNumber("Speed", states[0].speedMetersPerSecond);
    SmartDashboard.putNumber("Angle", states[0].angle.getDegrees());
    //var gyroAngle = Rotation2d.fromDegrees(ahrs.getAngle());
    moduleFL.SetTargetAngleAndSpeed(states[0].angle.getDegrees(), states[0].speedMetersPerSecond, canCoderFL.getAbsolutePosition());
    moduleFR.SetTargetAngleAndSpeed(states[1].angle.getDegrees(), states[1].speedMetersPerSecond, canCoderFR.getAbsolutePosition());
    moduleBL.SetTargetAngleAndSpeed(states[2].angle.getDegrees(), states[2].speedMetersPerSecond, canCoderBL.getAbsolutePosition());
    moduleBR.SetTargetAngleAndSpeed(states[3].angle.getDegrees(), states[3].speedMetersPerSecond, canCoderBR.getAbsolutePosition());
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

  public Block getBiggestBlock() {
		// Gets the number of "blocks", identified targets, that match signature 1 on the Pixy2,
		// does not wait for new data if none is available,
		// and limits the number of returned blocks to 25, for a slight increase in efficiency
		int blockCount = pixy.getCCC().getBlocks(false, Pixy2CCC.CCC_SIG1, 25);
		if (blockCount != 0) System.out.println("Found " + blockCount + " blocks!"); // Reports number of blocks found
		SmartDashboard.putNumber("number of blocks", blockCount);
    if (blockCount <= 0) {
			return null; // If blocks were not found, stop processing
		}
    
		ArrayList<Block> blocks = pixy.getCCC().getBlockCache(); // Gets a list of all blocks found by the Pixy2
		SmartDashboard.putNumber("x", blocks.get(0).getX());
    SmartDashboard.putNumber("y", blocks.get(0).getY());
    Block largestBlock = null;
		for (Block block : blocks) { // Loops through all blocks and finds the widest one
			if (largestBlock == null) {
				largestBlock = block;
			} else if (block.getWidth() > largestBlock.getWidth()) {
				largestBlock = block;
			}
		}
		return largestBlock;
	}
}
