// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.wpilibj.TimedRobot;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  // Controllers

  private final boolean JOYSTICK_CONTROL = true;

  private final XboxController m_driverController = new XboxController(0);
  private final XboxController m_opperateController = new XboxController(1);
 
  private final Joystick driverJoystick = new Joystick(0);
  private final Joystick driverJoystick2 = new Joystick(2);

  // Control Values

  private double strafeControllerVal;
  private double driveControllerVal;
  private double rotateControllerVal;
  private boolean robotCentricControl;

  private boolean toggleInputControl;
  private boolean reverseInputControl;
  private boolean shootControl;
  private boolean pistonForwardControl;
  private boolean pistonBackwardControl;
  private boolean toggleAutoAimControl;
  
  private boolean raiseShooterControl;
  private boolean lowerShooterControl;
  
  private double swerveSpinSpeedModifier;

  private void updateControlValues() {
    // Set control variables with this function and call it at the top of teleop periodic
    if (JOYSTICK_CONTROL) {
      swerveSpinSpeedModifier = driverJoystick2.getRawButton(2) ? .25 : 1;
    } else {
      swerveSpinSpeedModifier = m_driverController.getXButton() ? .25 : 1;
    }

    toggleInputControl = m_opperateController.getAButtonPressed();
    toggleAutoAimControl = m_opperateController.getStartButtonPressed();

    pistonBackwardControl = m_opperateController.getLeftBumperPressed();
    pistonForwardControl = m_opperateController.getRightBumperPressed();

    shootControl = m_opperateController.getYButton();

    // Swerve Controls

    strafeControllerVal = JOYSTICK_CONTROL ? 
      Math.abs(driverJoystick.getX()) < Constants.JOYSTICK_DEADZONE ? 0 : -driverJoystick.getX() : 
      m_driverController.getLeftX();

    driveControllerVal = JOYSTICK_CONTROL ?
      Math.abs(driverJoystick.getY()) < Constants.JOYSTICK_DEADZONE ? 0 : driverJoystick.getY() :
      -m_driverController.getLeftY();

    rotateControllerVal = JOYSTICK_CONTROL ?
      Math.abs(driverJoystick2.getX()) < Constants.JOYSTICK_DEADZONE ? 0 : (-driverJoystick2.getX() * .5) * swerveSpinSpeedModifier :
      m_driverController.getRightX() * swerveSpinSpeedModifier;

    robotCentricControl = JOYSTICK_CONTROL ? driverJoystick.getRawButton(3) : m_driverController.getLeftBumper();

    raiseShooterControl = m_opperateController.getPOV() == Constants.DPAD_UP;
    lowerShooterControl = m_opperateController.getPOV() == Constants.DPAD_DOWN;

    reverseInputControl = m_opperateController.getPOV() == Constants.DPAD_RIGHT;
  }

  // Sensors

  private final AHRS gyro = new AHRS();
  double yaw;
  
  DigitalInput noteSensor = new DigitalInput(0);

  // Drivetrain

  private final WheelDrive backRight = new WheelDrive(5, 6, 11);
  private final WheelDrive backLeft = new WheelDrive(7, 8, 10);
  private final WheelDrive frontRight = new WheelDrive(3, 4, 12);
  private final WheelDrive frontLeft = new WheelDrive(1, 2, 9);
  private final Swervedrive swervedrive = new Swervedrive(backRight, backLeft, frontRight, frontLeft, gyro);
  
  // Shooter
  
  private final CANSparkMax leftShooter = new CANSparkMax(33, MotorType.kBrushless);
  private final CANSparkMax rightShooter = new CANSparkMax(34, MotorType.kBrushless);
  
  private final TalonFX shooterArm = new TalonFX(35);
  private double armSpeed; // for manual control
  private boolean autoAimEnabled;
  
  private double shooterArmPosition;
  
  private double shooterSetSpeed;
  private double shooterSpeed = 0;

  private double currentShooterRPM = 0;
  private double targetShooterRPM = 0;

  private final Timer shooterTimer = new Timer();

  // Intake

  private final CANSparkMax intake = new CANSparkMax(32, MotorType.kBrushless);
  private double intakeSpeed;
  private boolean shouldRunIntake;
  private boolean hasNote;

  // Climb
  final Compressor m_Compressor = new Compressor(30, PneumaticsModuleType.REVPH);
  final DoubleSolenoid climbPiston = new DoubleSolenoid(30, PneumaticsModuleType.REVPH, 8, 9);
  Value pistonValue = Value.kOff;

  // Limelight
  
  NetworkTable limelightData = NetworkTableInstance.getDefault().getTable("limelight");
  public double limelightY;
  public double detectedAprilTagId;

  private double speakerId;
  
  // Autonomous Mode
  
  String m_autoSelected;
  final SendableChooser<String> autonChooser = new SendableChooser<>();
  private static final String twoNoteKey = "two_note";
  private static final String threeNoteBlueKey = "three_note_blue";
  private static final String threeNoteRedKey = "three_note_red";
  private static final String crossLineKey = "cross_line";
  
  private static final Timer autonMasterTimer = new Timer();
  
  NetworkTable fmsTable = NetworkTableInstance.getDefault().getTable("FMSInfo");
  boolean isRed;

  // Auton running values 
  
  private boolean autonIsShooting = false;
  private double autonStrafeSpeed;
  private double autonDriveSpeed;
  private double autonStep;
  private double autonRotationVal = 0;
  
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    gyro.reset();
    m_Compressor.enableAnalog(90,120);
     
    CameraServer.startAutomaticCapture();
    Slot0Configs slot0Configs = new Slot0Configs();
    slot0Configs.kP = 4;
    slot0Configs.kI = 0;
    slot0Configs.kD = 0.1;
    shooterArm.getConfigurator().apply(slot0Configs);
    
    isRed = fmsTable.getEntry("IsRedAlliance").getBoolean(true);
    speakerId = isRed ? Constants.RED_SPEAKER_ID : Constants.BLUE_SPEAKER_ID;

    autonChooser.setDefaultOption("2 Note", twoNoteKey);
    autonChooser.addOption("4 Note Blue", threeNoteBlueKey);
    autonChooser.addOption("4 Note Red", threeNoteRedKey); // TODO this doesn't do anything
    autonChooser.addOption("Backwards", crossLineKey);
    SmartDashboard.putData("Auto choices", autonChooser);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    limelightY = limelightData.getEntry("ty").getDouble(0.0);
    detectedAprilTagId = limelightData.getEntry("tid").getDouble(-1);

    isRed = fmsTable.getEntry("IsRedAlliance").getBoolean(true);
    speakerId = isRed ? Constants.RED_SPEAKER_ID : Constants.BLUE_SPEAKER_ID;

    SmartDashboard.putNumber("LimelightY", limelightY);
    SmartDashboard.putNumber("Detected April Tag Id", detectedAprilTagId);
    SmartDashboard.putBoolean("isRed", isRed);
    SmartDashboard.putNumber("Selected Speaker ID", speakerId);

    yaw = gyro.getYaw();
    SmartDashboard.putNumber("Yaw", yaw);

    currentShooterRPM = Math.abs(leftShooter.getEncoder().getVelocity());
    SmartDashboard.putNumber("RPM", currentShooterRPM);
    SmartDashboard.putNumber("Shooter Position", shooterArmPosition);

    hasNote = !noteSensor.get();
    SmartDashboard.putBoolean("Have Note", hasNote);


    SmartDashboard.putNumber("Target RPM", targetShooterRPM);
    SmartDashboard.putNumber("Current RPM", currentShooterRPM);
    SmartDashboard.putNumber("Shooter Timer", shooterTimer.get());
    
    SmartDashboard.putNumber("Auton Angle", autonStrafeSpeed);
    SmartDashboard.putNumber("Auton Speed", autonDriveSpeed);
    SmartDashboard.putNumber("Auton Rotate", autonRotationVal);
    SmartDashboard.putNumber("Auton Step", autonStep);
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
    m_autoSelected = autonChooser.getSelected();
    System.out.println("Running Auton: " + m_autoSelected);
    
    autonStrafeSpeed = 0;
    autonStep = 0;
    autonMasterTimer.reset();
    autonMasterTimer.start();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);
    currentShooterRPM = leftShooter.getEncoder().getVelocity(); // TODO this is negative here but not in tele
    
    intakeSpeed = 0;
    autonDriveSpeed = 0;
    autonRotationVal = 0;
    // negative rotate clockwise positive clockwise

    if (detectedAprilTagId == speakerId) {
      if (limelightY != 0) {
        shooterSpeed = MathHelp.map(limelightY, -2, 30, .75, .4) * .9;
        targetShooterRPM = MathHelp.map(shooterSpeed, .4, .75, 2600, 4700);
        shooterArmPosition = MathHelp.map(limelightY, -2, 30, -10, -30);
      }

      shooterArmPosition = MathUtil.clamp(shooterArmPosition, -28, 0);
      shooterSpeed = MathUtil.clamp(shooterSpeed, .4, .75);
      targetShooterRPM = MathUtil.clamp(targetShooterRPM, -4700, -2600);
    }

    // TODO: if we dont get limelight data we should hard code a setpoint

    /*
     * Our goal with refactoring the autons is to make a solid cross the line, and a mirrorable two note auton
     * As a stretch goal we should try for a three note
     * 
     * Two Note: 
     */

    switch (m_autoSelected) {
      case threeNoteBlueKey: 
        if (autonStep == 0) { // backup
          autonStrafeSpeed = 0;
          autonDriveSpeed = .4;
          if (autonMasterTimer.get() > .125) {
            autonMasterTimer.reset();
            autonStep++;
          }
        } else if (autonStep == 1) {
          autonIsShooting = true;
          if (autonMasterTimer.get() > 1.75) {
            autonMasterTimer.reset();
            autonIsShooting = false;
            autonStep++;  
          }
        } else if (autonStep == 2) {
          autonStrafeSpeed = 0;
          autonDriveSpeed = .4;
          if (!shouldRunIntake) {
            autonMasterTimer.reset();
            autonStep++;
          }
          shouldRunIntake = true;
        } else if (autonStep == 3) {
          autonIsShooting = true;
          if (autonMasterTimer.get() > 1.6) {
            autonMasterTimer.reset();
            autonIsShooting = false;
            autonStep++;  
          }
        } else if (autonStep == 4) {
          autonStrafeSpeed = 315;
          if (autonMasterTimer.get() > .25) {
            autonMasterTimer.reset();
            autonStep++;
          }
        } else if (autonStep == 5) {
          autonStrafeSpeed = 315;
          autonDriveSpeed = .75;
          if (autonMasterTimer.get() > 1.35) {
            autonMasterTimer.reset();
            autonStep++;
          }
        } else if (autonStep == 6) {
          autonStrafeSpeed = 0;
          autonDriveSpeed = .75;
          if (autonMasterTimer.get() > .8) {
            autonMasterTimer.reset();
            autonStep++;
          }
        } else if (autonStep == 7) {
          if (yaw > 0) {
            autonRotationVal = -.1;
            if (MathHelp.isEqualApprox(yaw, 7 , 1)) {
              autonMasterTimer.reset();
              autonStep++;
            }
          } else {
            autonRotationVal = .1;
            if (MathHelp.isEqualApprox(yaw, 5 , 1)) {
              autonMasterTimer.reset();
              autonStep++;
            }
          }
        } else if (autonStep == 8) {
          autonStrafeSpeed = 0;
          autonDriveSpeed = .8;
          if (!shouldRunIntake || autonMasterTimer.get() > .6) {
            autonMasterTimer.reset();
            autonStep++;
          }
        } else if (autonStep == 9) {
          autonStrafeSpeed = 165;
          autonDriveSpeed = .8;
          if (autonMasterTimer.get() > 2) {
            autonMasterTimer.reset();
            autonStep++;
          }
        } else if (autonStep == 10) {
          autonRotationVal = -.1;
            if (MathHelp.isEqualApprox(yaw,  -10, 2)) {
              autonMasterTimer.reset();
              autonStep++;
            }
        } else if (autonStep == 11) { 
          if (autonMasterTimer.get() > .35) {
            autonMasterTimer.reset();
            autonStep++;
          }
        }  
        break;
      case crossLineKey:
        if (autonStep == 0) {
          autonStrafeSpeed = 0;
          autonDriveSpeed = .6;
          if (autonMasterTimer.get() > 3) {
            autonMasterTimer.reset();
            autonStep++;
          }
        }
        break;
      case twoNoteKey:
      default:
        if (autonStep == 0) { // backup
          autonStrafeSpeed = 0;
          autonDriveSpeed = .4;
          if (autonMasterTimer.get() > .125) {
            autonMasterTimer.reset();
            autonStep++;
          }
        } else if (autonStep == 1) { // shoot
          autonIsShooting = true;
          if (autonMasterTimer.get() > 1.75) {
            autonMasterTimer.reset();
            autonIsShooting = false;
            autonStep++;  
          }
        } else if (autonStep == 2) { // backup more
          autonStrafeSpeed = 0;
          autonDriveSpeed = .4;
          if (!shouldRunIntake) {
            autonMasterTimer.reset();
            autonStep++;
          }
          shouldRunIntake = true;
        } else if (autonStep == 3) { // shoot
          autonIsShooting = true;
          if (autonMasterTimer.get() > 1.6) {
            autonMasterTimer.reset();
            autonIsShooting = false;
            autonStep++;  
          }
        }
        break;
      }


    // Set motors based on control values

    if (autonIsShooting) { // shooting
      if (currentShooterRPM < targetShooterRPM || autonMasterTimer.get() > 1.5) {
        shouldRunIntake = true;
      }
    } else { // not shooting
      if (hasNote) {
        shouldRunIntake = false;
      }
    }

    swervedrive.autoDrive(autonStrafeSpeed - yaw, autonDriveSpeed, autonRotationVal); // TODO: Should yaw be subtracted from rotate?
    
    intakeSpeed = shouldRunIntake ? -1 : 0;
    intake.set(intakeSpeed);

    shooterArm.setControl(m_request.withPosition(shooterArmPosition));
    rightShooter.set(shooterSpeed);
    leftShooter.set(-shooterSpeed);
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    shouldRunIntake = false;
    autoAimEnabled = true;
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);
    updateControlValues();
    
    if (detectedAprilTagId == speakerId) {
      // y = y > 30 ? y : y - 3;
      if (limelightY != 0) {
        shooterSpeed = MathHelp.map(limelightY, -2, 30, .75, .4) * .9;
        targetShooterRPM = MathHelp.map(shooterSpeed, .4, .75, 2600, 4700);
        shooterArmPosition = MathHelp.map(limelightY, -2, 30, -10, -30);
      }

      shooterArmPosition = MathUtil.clamp(shooterArmPosition, -28, 0);
      shooterSpeed = MathUtil.clamp(shooterSpeed, .4, .74);
      targetShooterRPM = MathUtil.clamp(targetShooterRPM, 2600, 4700);
    }

    swervedrive.drive(strafeControllerVal, driveControllerVal, rotateControllerVal, robotCentricControl);
    
    // intake decision logic

    // intended behavior: 
    // if you hit a toggle the intake
    // if you pick up a note turn off the intake
    // if you press y, rev up shooter until target OR .25 sec (whichever is first) then run the intake to trigger a shot

    // NOTE this doesn't allow you to spit notes out without delay, do we need that for feeding?
    
    // shooter is running and we have no note, we just shot or we aborted a shot
    if ((shooterTimer.get() > 0 && !hasNote) || !shootControl) {
      shooterTimer.stop();
      shooterTimer.reset();
    }
    
    if (hasNote && shootControl) {
      // we are loaded and trying to shoot
      shooterTimer.start();
      if ((autoAimEnabled && currentShooterRPM >= targetShooterRPM) || shooterTimer.hasElapsed(Constants.SHOOT_TIMEOUT)) {
        shouldRunIntake = true;
      }
    } else if (hasNote) {
      // we are loaded stop the intake
      shouldRunIntake = false;
    } else if (toggleInputControl) {
      // manual toggle
      shouldRunIntake = !shouldRunIntake;
    }

    // TODO should this get moved up to the intake code as well? or possibly be its own y button press?
    // maybe something like if (buttonPressed && shooterSetSpeed == 0) -> shooterSetSpeed = autoAimEnabled ? shooterSpeed : .65;
    // and then set shooterSetSpeed = 0 when we fire the note
    
    if (shootControl) {
      shooterSetSpeed = autoAimEnabled ? shooterSpeed : .65;
    } else {
      shooterSetSpeed = 0;
    }
    
    if (reverseInputControl) {
      // override to backdrive intake
      intakeSpeed = 1;
    } else {
      intakeSpeed = shouldRunIntake ? -1 : 0;
    }
    
    if (pistonForwardControl) {
      pistonValue = Value.kForward;
    } else if (pistonBackwardControl) {
      pistonValue = Value.kReverse;
    } else {
      pistonValue = Value.kOff;
    }

    armSpeed = 0;
    if (lowerShooterControl) {
      armSpeed = -.4;
    } else if (raiseShooterControl) {
      armSpeed = .1;
    }
    else if (toggleAutoAimControl) {
      autoAimEnabled = !autoAimEnabled;
    }
    
    if (!autoAimEnabled) {
      shooterArm.set(armSpeed);
    } else {
      shooterArm.setControl(m_request.withPosition(shooterArmPosition));
    }
    
    climbPiston.set(pistonValue);
    intake.set(intakeSpeed);
    shooterSetSpeed *= -1;
    rightShooter.set(shooterSetSpeed);
    leftShooter.set(-shooterSetSpeed);
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


  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}
  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}

  
}
