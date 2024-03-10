// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


//// Required Imports to run any code ////
package frc.robot;
import edu.wpi.first.wpilibj.TimedRobot;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
//// Imports for motors ////
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
//// Imports for controls ////
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//// Imports for Shuffleboard ////
// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
// import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

//// Imports for autonomus ////
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.Encoder;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  String m_autoSelected;
  int masterWheelAngle = 0;
  final SendableChooser<String> m_chooser = new SendableChooser<>();
  private static final String kDefaultAuto = "2 Note";
  private static final String k4NoteBlue = "4 Note Blue";
  private static final String k4NoteRed = "4 Note Red";
  private static final String kBackwards = "Backwards";

  final XboxController m_driverController = new XboxController(0);
  final XboxController m_opperateController = new XboxController(1);
 
  
  final Joystick driverJoystick = new Joystick(0);
  final Joystick driverJoystick2 = new Joystick(2);

  final AHRS gyro = new AHRS();
  //final CANSparkMax spark = new CANSparkMax(0, null);
  
  // private ShuffleboardTab tab = new Shuffleboard.get

  private WheelDrive backRight = new WheelDrive(5, 6, 11);
  private WheelDrive backLeft = new WheelDrive(7, 8, 10);
  private WheelDrive frontRight = new WheelDrive(3, 4, 12);
  private WheelDrive frontLeft = new WheelDrive(1, 2, 9);

  TalonFX winch = new TalonFX(31);
  CANSparkMax intake = new CANSparkMax(32, MotorType.kBrushless);

  double intakeSpeed;
  double winchSpeed;

  CANSparkMax shootLeft = new CANSparkMax(33, MotorType.kBrushless);
  CANSparkMax shootRight = new CANSparkMax(34, MotorType.kBrushless);

  double shootSpeed;     

  TalonFX shootArm = new TalonFX(35);

  double armSpeed;

  PIDController pid = new PIDController(.02, .001, 0);

  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

  public double x;
  public double y;
  public double area;
  public double id;

  NetworkTable fmsTable = NetworkTableInstance.getDefault().getTable("FMSInfo");

  NetworkTableEntry Red = fmsTable.getEntry("IsRedAlliance");

  

  boolean isRed = Red.getBoolean(true);

  double ampId;
  double speakId;

  private Swervedrive swervedrive = new Swervedrive(backRight, backLeft, frontRight, frontLeft, gyro);
  public double angle;
  public double speed;
  public double step;
  public Timer m_timer = new Timer();

  double pos;
  double oldPos;
  double actSetPos;

  double posLat;

  StatusSignal<Double> posit;

  boolean runningIntake;

  boolean aim;

  double position;

  boolean note;


  DigitalInput sensor = new DigitalInput(0);
  
  
  DutyCycleEncoder encode = new DutyCycleEncoder(1);

  double shooterSpeed = 0;
  double oldSpeed = 0;

  double shooterRPM = 0;
  double targetRPM = 0;

  boolean slowedTurn = false;

  double slow = 1;

  double rotate = 0;

  boolean isShooting = false;

  Timer shootTimer = new Timer();

  double yaw;

  boolean control = false;

  VictorSP climbArm = new VictorSP(1);

  boolean climbUp = false;


   public double clawSpeed = 0;


  
  // private Swervedrive swervedrive = new Swervedrive(null, null, null, frontLeft);
  
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    gyro.reset();
     
    CameraServer.startAutomaticCapture();
    oldPos = 0;
    Slot0Configs slot0Configs = new Slot0Configs();
    slot0Configs.kP = 4;
    slot0Configs.kI = 0;
    slot0Configs.kD = 0.1;
    shootArm.getConfigurator().apply(slot0Configs);
    
    if (isRed)
    {
      speakId = 4; 
    }
    else
    {
      speakId = 8;
    }
    m_chooser.setDefaultOption("2 Note", kDefaultAuto);
    m_chooser.addOption("4 Note Blue", k4NoteBlue);
    m_chooser.addOption("4 Note Red", k4NoteRed);
    m_chooser.addOption("Backwards", kBackwards);
    SmartDashboard.putData("Auto choices", m_chooser);
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
    SmartDashboard.putNumber("pos", pos);
    
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    NetworkTableEntry tid = table.getEntry("tid");

    x = tx.getDouble(0.0);
    y = ty.getDouble(0.0);
    area = ta.getDouble(0.0);
    id = tid.getDouble(-1);


    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);
    SmartDashboard.putNumber("April Tag Id", id);
    
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
    
    System.out.println("Auto selected: " + m_autoSelected);
    // gyro.reset();
    angle = 0;
    speed = .5;
    step = 0;
    m_timer.reset();
    m_timer.start();
    
    speakId = isRed ? 4 : 7;
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);
    shooterRPM = shootLeft.getEncoder().getVelocity();
    yaw = gyro.getYaw();
    SmartDashboard.putNumber("yaw", yaw);
    intakeSpeed = 0;
    speed = 0;
    rotate = 0;

    // negative rotate clockwise positive clockwise

    if (id == speakId) {
      actSetPos = MathHelp.map(y, -2, 30, -10, -28);
      shooterSpeed = MathHelp.map(y, -2, 30, .75, .4);
      targetRPM = MathHelp.map(shooterSpeed, .4, .75, -2600, -4700);
      if (y != 0) {
        oldPos = actSetPos;
        oldSpeed = shooterSpeed;
      } else {
        actSetPos = oldPos;
        shooterSpeed = oldSpeed;
      }

      actSetPos = MathUtil.clamp(actSetPos, -28, 0);
      shooterSpeed = MathUtil.clamp(shooterSpeed, .4, .75);
      targetRPM = MathUtil.clamp(targetRPM, -4700, -2600);
    }

    switch (m_autoSelected) {
      case k4NoteBlue: 
        System.out.println("Running 4Note Auton " + m_autoSelected);
        if (step == 0) { // backup
        angle = 0;
        speed = .4;
        if (m_timer.get() > .125) {
          m_timer.reset();
          step++;
        }
        } else if (step == 1) {
          isShooting = true;
          if (m_timer.get() > 1.75) {
            m_timer.reset();
            isShooting = false;
            step++;  
          }
        } else if (step == 2) {
          angle = 0;
          speed = .4;
          if (!runningIntake)
          {
            m_timer.reset();
            step++;
          }
          runningIntake = true;
        } else if (step == 3) {
          isShooting = true;
          if (m_timer.get() > 1.6) {
            m_timer.reset();
            isShooting = false;
            step++;  
          }
        } else if (step == 4) {
          angle = 315;
          if(m_timer.get() > .25) {
            m_timer.reset();
            step++;
          }
        } else if (step == 5) {
          angle = 315;
          speed = .75;
          if (m_timer.get() > 1.35)
          {
            m_timer.reset();
            step++;
          }
        } else if (step == 6) {
          angle = 0;
          speed = .75;
          if (m_timer.get() > .8)
          {
            m_timer.reset();
            step++;
          }
        } else if(step == 7) {
          if (yaw > 0)
          {
            rotate = -.1;
            if(MathHelp.isEqualApprox(yaw, 7 , 1))
            {
              m_timer.reset();
              step++;
            }
          } else {
            rotate = .1;
            if(MathHelp.isEqualApprox(yaw, 5 , 1))
            {
              m_timer.reset();
              step++;
            }
        }
  
      } else if (step == 8) {
        angle = 0;
        speed = .8;
        if (!runningIntake || m_timer.get() > .6)
        {
          m_timer.reset();
          step++;
        }
      } else if (step == 9) {
        angle = 165;
        speed = .8;
        if (m_timer.get() > 2)
        {
          m_timer.reset();
          step++;
        }
      } else if (step == 10) {
        rotate = -.1;
          if(MathHelp.isEqualApprox(yaw,  -10, 2))
          {
            m_timer.reset();
            step++;
          }
      } else if (step == 11) { 
        if(m_timer.get() > .35)
          {
            m_timer.reset();
            step++;
          }
        }  
        break;
      case kBackwards:
        System.out.println("Running Backwards Auton " + m_autoSelected);
        if (step == 0) {
          angle = 0;
          speed = .6;
          if(m_timer.get() > 3)
          {
            m_timer.reset();
            step++;
          }
        }
        break;
      case kDefaultAuto:
      default:
        System.out.println("Running Default Auton " + m_autoSelected);
        if (step == 0) { // backup
          angle = 0;
          speed = .4;
          if (m_timer.get() > .125) {
            m_timer.reset();
            step++;
          }
        } else if (step == 1) { // shoot
          isShooting = true;
          if (m_timer.get() > 1.75)
          {
            m_timer.reset();
            isShooting = false;
            step++;  
          }
        } else if (step == 2) { // backup more
          angle = 0;
          speed = .4;
          if (!runningIntake) {
            m_timer.reset();
            step++;
          }
          runningIntake = true;
        } else if (step == 3) { // shoot
          isShooting = true;
          if (m_timer.get() > 1.6) {
            m_timer.reset();
            isShooting = false;
            step++;  
          }
        }
        break;
    }
    
    /*
     * 
     // boolean debugging = false;
     // if (debugging) {
     //   if (step == 0) {
     //     angle = 0;
     //     speed = .4;
     //     if(m_timer.get() > .3)
     //     {
     //       m_timer.reset();
     //       step++;
     //     }
     //   }
     //   if (step == 1) {
         
     //       rotate = -.1;
     //       if(MathHelp.isEqualApprox(yaw,  -10, 2))
     //       {
     //         m_timer.reset();
     //         step++;
     //       }
         
     //   } 
     // } else {
     //   if (step == 0) { // backup
     //     angle = 0;
     //     speed = .4;
     //     if (m_timer.get() > .125)
     //     {
     //       m_timer.reset();
     //       step++;
     //     }
     //   } else if (step == 1) {
     //     isShooting = true;
     //     if (m_timer.get() > 1.75)
     //     {
     //       m_timer.reset();
     //       isShooting = false;
     //       step++;  
     //     }
     //   } else if (step == 2) {
     //     angle = 0;
     //     speed = .4;
     //     if (!intook)
     //     {
     //       m_timer.reset();
     //       step++;
     //     }
     //     intook = true;
     //   } else if (step == 3) {
     //     isShooting = true;
     //     if (m_timer.get() > 1.6)
     //     {
     //       m_timer.reset();
     //       isShooting = false;
     //       step++;  
     //     }
     //   } else if (step == 4) {
     //     angle = 315;
     //     if(m_timer.get() > .25)
     //     {
     //       m_timer.reset();
     //       step++;
     //     }
     //   } else if (step == 5) {
     //     angle = 315;
     //     speed = .75;
     //     if (m_timer.get() > 1.35)
     //     {
     //       m_timer.reset();
     //       step++;
     //     }
     //   } else if (step == 6) {
     //     angle = 0;
     //     speed = .75;
     //     if (m_timer.get() > .8)
     //     {
     //       m_timer.reset();
     //       step++;
     //     }
     //   } else if(step == 7) {
     //     if(yaw > 0)
     //     {
     //       rotate = -.1;
     //       if(MathHelp.isEqualApprox(yaw, 7 , 1))
     //       {
     //         m_timer.reset();
     //         step++;
     //       }
     //     }
     //     else
     //     {
     //       rotate = .1;
     //       if(MathHelp.isEqualApprox(yaw, 5 , 1))
     //       {
     //         m_timer.reset();
     //         step++;
     //       }
     //     }
   
     //   } else if (step == 8) {
     //     angle = 0;
     //     speed = .8;
     //     if (!intook || m_timer.get() > .6)
     //     {
     //       m_timer.reset();
     //       step++;
     //     }
     //   } else if (step == 9) {
     //     angle = 165;
     //     speed = .8;
     //     if (m_timer.get() > 2)
     //     {
     //       m_timer.reset();
     //       step++;
     //     }
     //   } else if (step == 10) {
     //     rotate = -.1;
     //       if(MathHelp.isEqualApprox(yaw,  -10, 2))
     //       {
     //         m_timer.reset();
     //         step++;
     //       }
   
     //   } else if (step == 11) {
         
     //     if(m_timer.get() > .35)
     //     {
     //       m_timer.reset();
     //       step++;
     //     }
     //   }  else if (step == 12) {
     //     isShooting = true;
     //     actSetPos = -23;
     //     if(m_timer.get() > 1)
     //     {
     //       m_timer.reset();
     //       step++;
     //     }
     //   } 
     // }
     
     */

    if (isShooting) { // shooting
      if (shooterRPM < targetRPM || m_timer.get() > 1.5) {
          runningIntake = true;
      }
    } else { // not shooting
      if (!sensor.get()) { // sensor reads false when it has a note
        runningIntake = false;
      }
    }


    if (runningIntake) {
      intakeSpeed = -1;
    }
    
    // if ( angle < 0 ) 
    // {
    //   angle = 360 - angle;
    // }
    
    swervedrive.autoDrive(angle-yaw, speed, rotate);
    climbArm.set(-1);
    intake.set(intakeSpeed);
    shootArm.setControl(m_request.withPosition(actSetPos));
    shootRight.set(shooterSpeed);
    shootLeft.set(-shooterSpeed);
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    runningIntake = false;
    aim = true;
    clawSpeed = -1;
    note = false;
    oldPos = 0;
    oldSpeed = 0;
    m_timer.reset();
    slowedTurn = false;
    slow = 1;
    control = true;
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {  
    final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);
    SmartDashboard.putBoolean("sense", !sensor.get());

    SmartDashboard.putNumber("arm", encode.getAbsolutePosition());

    yaw = gyro.getYaw();
    SmartDashboard.putNumber("yaw", yaw);
    
    if (id == speakId)
    {
      // y = y > 30 ? y: y - 3;
      actSetPos = MathHelp.map(y, -2, 30, -10, -30);
      shooterSpeed = MathHelp.map(y, -2, 30, .75, .4);
      targetRPM = MathHelp.map(shooterSpeed, .4, .75, -2600, -4700);
      if (y != 0)
      {
        oldPos = actSetPos;
        oldSpeed = shooterSpeed;
      }
      else
      {
        actSetPos = oldPos;
        shooterSpeed = oldSpeed;
      }
      SmartDashboard.putNumber("Shoot Angle", actSetPos);
      

      if (actSetPos > 0)
      {
        actSetPos = 0;
      }
      if (actSetPos < -28)
      {
        actSetPos = -28;
      }
      if (shooterSpeed > .75)
      {
        shooterSpeed = .75;
      }
      if (shooterSpeed < .4)
      {
        shooterSpeed = .4;
      }
      if (targetRPM > -2600)
      {
        targetRPM = -2600;
      }
      if (targetRPM < -4700)
      {
        targetRPM = -4700;
      }
    }
    position = actSetPos;
    
    shooterRPM = shootLeft.getEncoder().getVelocity();
    SmartDashboard.putNumber("RPM", shooterRPM);

    if (m_driverController.getXButton())
    {
      slowedTurn = !slowedTurn;
    }

    if (!control) {
      slow = !m_driverController.getXButton() ? 1 : .25;
    }
    else {
      slow = !driverJoystick2.getRawButton(2) ? 1 : .25;
    }
    if (!control)
    {
      swervedrive.drive(m_driverController.getLeftX(), -m_driverController.getLeftY(), m_driverController.getRightX() * slow, m_driverController.getLeftBumper());
    }
    else
    {
      swervedrive.drive(driverJoystick.getX() > -.075 && driverJoystick.getX() < .075 ? 0 : driverJoystick.getX() , -driverJoystick.getY() > -.05 && -driverJoystick.getY() < .05 ? 0 : -driverJoystick.getY(), driverJoystick2.getX() > -.05 && driverJoystick2.getX() < .05 ? 0 : (driverJoystick2.getX() * .5) * slow, driverJoystick.getRawButton(2));
    }
    
      

    intakeSpeed = 0;
    if(m_opperateController.getAButtonPressed())
    {
      runningIntake = !runningIntake;
    }
    if(m_opperateController.getYButton() == false && m_driverController.getBButton() == false)
    {
      if(!sensor.get())
      {
        runningIntake = false;
      }
    }
    if (m_opperateController.getYButton() == true && aim)
    {
      if (shooterRPM < targetRPM)
      {
        m_timer.start();
        if(m_timer.get() > .25)
        {
          runningIntake = true;
        }
      }
      
    }
    else
    {
      m_timer.reset();
    }
    
    
    if (runningIntake)
    {
      intakeSpeed = -1;
    }
    
    if (m_opperateController.getPOV() == 90)
    {
      intakeSpeed = 1;
    }
    // -4700 == 75
    // -2600 == 40


    
    winchSpeed = 0;
    if (m_opperateController.getRawButton(7))
    {
      winchSpeed = -1;
      
    }
    if (m_opperateController.getRightBumper())
    {
      winchSpeed = 1;
    }
    
    

    shootSpeed = 0;
    if (m_opperateController.getYButton() && aim) {
      shootSpeed = shooterSpeed;
    }
    else if (m_opperateController.getYButton())
    {
      shootSpeed = .65;
    }
    

    armSpeed = 0;
    // if (m_opperateController.getRightBumperPressed() && aim)
    // {
    //   position = -31.17;
    // }
    // else if (m_opperateController.getRightStickButtonPressed() && aim)
    // {
    //   position = 0;
    // }
    
    // else if (m_opperateController.getLeftBumperPressed() && aim)
    // {
    //   position = -42;
    // }
    // else 
    if (m_opperateController.getPOV() == 0)
    {
      armSpeed = -.4;
    }
    else if (m_opperateController.getPOV() == 180)
    {
      armSpeed = .1;
    }
    else if (m_opperateController.getStartButtonPressed())
    {
      aim = !aim;
    }

    
    
    if (m_opperateController.getLeftBumperPressed())
    {
      clawSpeed = 1;
      aim = !aim;
    }
    climbArm.set(clawSpeed);
    
    // -34.6
    // if(aim)
    // {
    //   armSpeed = actSpeed;
    // }
    // armSpeed = 0;
    winch.set(winchSpeed);
    intake.set(intakeSpeed);
    
    
    if (!aim) {
      shootArm.set(armSpeed);
    } else {
      shootArm.setControl(m_request.withPosition(position));
    }
    

    shootRight.set(shootSpeed);
    shootLeft.set(-shootSpeed);
    
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
