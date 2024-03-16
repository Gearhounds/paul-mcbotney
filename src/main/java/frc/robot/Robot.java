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
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
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

  private boolean toggleInputControl;
  private boolean shootControl;
  private boolean pistonForwardControl;
  private boolean pistonBackwardControl;
  private boolean toggleAutoAimControl;
  private boolean raiseClawsControl;
  
  private int operatorDPad;
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

    pistonBackwardControl = m_opperateController.getLeftBumperPressed();
    pistonForwardControl = m_opperateController.getRightBumperPressed();

    shootControl = m_opperateController.getYButton();
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
  final Compressor m_Compressor = new Compressor(PneumaticsModuleType.REVPH);
  final DoubleSolenoid climbPiston = new DoubleSolenoid(PneumaticsModuleType.REVPH, 8, 9);
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
  boolean isRed = fmsTable.getEntry("IsRedAlliance").getBoolean(true);
  
  // Auton running values 
  private boolean isShooting = false;
  private double angle;
  private double speed;
  private double step;
  private double rotate = 0;
  
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    gyro.reset();
    m_Compressor.enableDigital();
     
    CameraServer.startAutomaticCapture();
    Slot0Configs slot0Configs = new Slot0Configs();
    slot0Configs.kP = 4;
    slot0Configs.kI = 0;
    slot0Configs.kD = 0.1;
    shooterArm.getConfigurator().apply(slot0Configs);
    
    if (isRed)
    {
      speakerId = 4; 
    }
    else
    {
      speakerId = 8;
    }
    autonChooser.setDefaultOption("2 Note", twoNoteKey);
    autonChooser.addOption("4 Note Blue", threeNoteBlueKey);
    autonChooser.addOption("4 Note Red", threeNoteRedKey);
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

    SmartDashboard.putNumber("LimelightY", limelightY);
    SmartDashboard.putNumber("April Tag Id", detectedAprilTagId);

    yaw = gyro.getYaw();
    SmartDashboard.putNumber("Yaw", yaw);

    currentShooterRPM = leftShooter.getEncoder().getVelocity();
    SmartDashboard.putNumber("RPM", currentShooterRPM);
    SmartDashboard.putNumber("Shooter Position", shooterArmPosition);

    hasNote = !noteSensor.get();
    SmartDashboard.putBoolean("Have Note", hasNote);
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
    
    angle = 0;
    step = 0;
    autonMasterTimer.reset();
    autonMasterTimer.start();
    
    speakerId = isRed ? 4 : 7;
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);
    currentShooterRPM = leftShooter.getEncoder().getVelocity();
    intakeSpeed = 0;
    speed = 0;
    // negative rotate clockwise positive clockwise
    rotate = 0;


    if (detectedAprilTagId == speakerId) {
      shooterArmPosition = MathHelp.map(limelightY, -2, 30, -10, -28);
      targetShooterRPM = MathHelp.map(shooterSpeed, .4, .75, -2600, -4700);
      
      if (limelightY != 0) {
        shooterSpeed = MathHelp.map(limelightY, -2, 30, .75, .4);
      }

      shooterArmPosition = MathUtil.clamp(shooterArmPosition, -28, 0);
      shooterSpeed = MathUtil.clamp(shooterSpeed, .4, .75);
      targetShooterRPM = MathUtil.clamp(targetShooterRPM, -4700, -2600);
    }

    switch (m_autoSelected) {
      case threeNoteBlueKey: 
        if (step == 0) { // backup
        angle = 0;
        speed = .4;
        if (autonMasterTimer.get() > .125) {
          autonMasterTimer.reset();
          step++;
        }
        } else if (step == 1) {
          isShooting = true;
          if (autonMasterTimer.get() > 1.75) {
            autonMasterTimer.reset();
            isShooting = false;
            step++;  
          }
        } else if (step == 2) {
          angle = 0;
          speed = .4;
          if (!shouldRunIntake)
          {
            autonMasterTimer.reset();
            step++;
          }
          shouldRunIntake = true;
        } else if (step == 3) {
          isShooting = true;
          if (autonMasterTimer.get() > 1.6) {
            autonMasterTimer.reset();
            isShooting = false;
            step++;  
          }
        } else if (step == 4) {
          angle = 315;
          if (autonMasterTimer.get() > .25) {
            autonMasterTimer.reset();
            step++;
          }
        } else if (step == 5) {
          angle = 315;
          speed = .75;
          if (autonMasterTimer.get() > 1.35)
          {
            autonMasterTimer.reset();
            step++;
          }
        } else if (step == 6) {
          angle = 0;
          speed = .75;
          if (autonMasterTimer.get() > .8)
          {
            autonMasterTimer.reset();
            step++;
          }
        } else if(step == 7) {
          if (yaw > 0)
          {
            rotate = -.1;
            if (MathHelp.isEqualApprox(yaw, 7 , 1))
            {
              autonMasterTimer.reset();
              step++;
            }
          } else {
            rotate = .1;
            if (MathHelp.isEqualApprox(yaw, 5 , 1))
            {
              autonMasterTimer.reset();
              step++;
            }
        }
  
      } else if (step == 8) {
        angle = 0;
        speed = .8;
        if (!shouldRunIntake || autonMasterTimer.get() > .6) {
          autonMasterTimer.reset();
          step++;
        }
      } else if (step == 9) {
        angle = 165;
        speed = .8;
        if (autonMasterTimer.get() > 2) {
          autonMasterTimer.reset();
          step++;
        }
      } else if (step == 10) {
        rotate = -.1;
          if (MathHelp.isEqualApprox(yaw,  -10, 2)) {
            autonMasterTimer.reset();
            step++;
          }
      } else if (step == 11) { 
        if (autonMasterTimer.get() > .35) {
            autonMasterTimer.reset();
            step++;
          }
        }  
        break;
      case crossLineKey:
        if (step == 0) {
          angle = 0;
          speed = .6;
          if(autonMasterTimer.get() > 3) {
            autonMasterTimer.reset();
            step++;
          }
        }
        break;
      case twoNoteKey:
      default:
        if (step == 0) { // backup
          angle = 0;
          speed = .4;
          if (autonMasterTimer.get() > .125) {
            autonMasterTimer.reset();
            step++;
          }
        } else if (step == 1) { // shoot
          isShooting = true;
          if (autonMasterTimer.get() > 1.75) {
            autonMasterTimer.reset();
            isShooting = false;
            step++;  
          }
        } else if (step == 2) { // backup more
          angle = 0;
          speed = .4;
          if (!shouldRunIntake) {
            autonMasterTimer.reset();
            step++;
          }
          shouldRunIntake = true;
        } else if (step == 3) { // shoot
          isShooting = true;
          if (autonMasterTimer.get() > 1.6) {
            autonMasterTimer.reset();
            isShooting = false;
            step++;  
          }
        }
        break;
    }

    if (isShooting) { // shooting
      if (currentShooterRPM < targetShooterRPM || autonMasterTimer.get() > 1.5) {
          shouldRunIntake = true;
      }
    } else { // not shooting
      if (hasNote) { // sensor reads false when it has a note
        shouldRunIntake = false;
      }
    }

    intakeSpeed = shouldRunIntake ? -1 : 0;

    swervedrive.autoDrive(angle-yaw, speed, rotate);
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
      shooterArmPosition = MathHelp.map(limelightY, -2, 30, -10, -30);
      targetShooterRPM = MathHelp.map(shooterSpeed, .4, .75, -2600, -4700);
      
      if (limelightY != 0) {
        shooterSpeed = MathHelp.map(limelightY, -2, 30, .75, .4);
      }

      shooterArmPosition = MathUtil.clamp(shooterArmPosition, -28, 0);
      shooterSpeed = MathUtil.clamp(shooterSpeed, .74, .4);
      targetShooterRPM = MathUtil.clamp(targetShooterRPM, -4700, -2600);
    }

    

    if (JOYSTICK_CONTROL) {
      swervedrive.drive(driverJoystick.getX() > -.075 && driverJoystick.getX() < .075 ? 0 : driverJoystick.getX() , -driverJoystick.getY() > -.05 && -driverJoystick.getY() < .05 ? 0 : -driverJoystick.getY(), driverJoystick2.getX() > -.05 && driverJoystick2.getX() < .05 ? 0 : (driverJoystick2.getX() * .5) * swerveSpinSpeedModifier, driverJoystick.getRawButton(2));
    } else {
      swervedrive.drive(m_driverController.getLeftX(), -m_driverController.getLeftY(), m_driverController.getRightX() * swerveSpinSpeedModifier, m_driverController.getLeftBumper());
    }
    
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
      if (currentShooterRPM >= targetShooterRPM || shooterTimer.hasElapsed(Constants.SHOOT_TIMEOUT)) {
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
    
    if (m_opperateController.getPOV() == 90) {
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
    climbPiston.set(pistonValue);
    
    

    armSpeed = 0;
    if (m_opperateController.getPOV() == 0) {
      armSpeed = -.4;
    } else if (m_opperateController.getPOV() == 180) {
      armSpeed = .1;
    }
    else if (m_opperateController.getStartButtonPressed()) {
      autoAimEnabled = !autoAimEnabled;
    }

    

    
    intake.set(intakeSpeed);
    
    if (!autoAimEnabled) {
      shooterArm.set(armSpeed);
    } else {
      shooterArm.setControl(m_request.withPosition(shooterArmPosition));
    }
    
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
