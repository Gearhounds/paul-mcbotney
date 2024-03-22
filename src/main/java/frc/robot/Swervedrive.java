package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.kauailabs.navx.frc.AHRS;

public class Swervedrive {
  private AHRS gyro;

  private WheelDrive backRight;
  private WheelDrive backLeft;
  private WheelDrive frontRight;
  private WheelDrive frontLeft;

  final XboxController m_driverController = new XboxController(0);

  public Swervedrive (WheelDrive backRight, WheelDrive backLeft, WheelDrive frontRight, WheelDrive frontLeft, AHRS gyro){
    this.backRight = backRight;
    this.backLeft = backLeft;
    this.frontRight = frontRight;
    this.frontLeft = frontLeft;
    this.gyro = gyro;
  }
  
  public void turn(double turnPower){
      frontLeft.SetDirection(45, turnPower);
      frontRight.SetDirection(135, turnPower);
      backLeft.SetDirection(225, turnPower);
      backRight.SetDirection(315, turnPower);
  }

  double oldAngle;

  public double calcJoystickAngle(double lX, double lY) {
    double angle = Math.atan2(lX, lY); // radians
    return angle * 180 / Math.PI;
  }

  public void drive(double lX, double lY, double rX, boolean isRobotCentric) {
      double speed = Math.hypot(lX, lY);
      double yaw = gyro.getYaw();
      double angle = calcJoystickAngle(lX, lY);
      SmartDashboard.putNumber("Joystick Angle", angle);
      if ((lY == 0 && lX == 0) && oldAngle != 0) {
        angle = oldAngle;
      }
      oldAngle = angle;
      // speed *= 0.5;

      SmartDashboard.putNumber("unmodified yaw", yaw);
      yaw = isRobotCentric ? yaw * 0: yaw;
      SmartDashboard.putNumber("modified yaw", yaw);
      SmartDashboard.putBoolean("isRobotCentric", isRobotCentric);
      if(rX == 0){
          frontLeft.SetDirection(angle-yaw, -speed);
          frontRight.SetDirection(angle-yaw, speed);
          backLeft.SetDirection(angle-yaw, speed);
          backRight.SetDirection(angle-yaw, speed);
      } else if (speed == 0){
        // Opposite diagonal axle distance = 33.2 inches
        frontLeft.SetDirection(42.5, -rX); //42.5 degrees
        frontRight.SetDirection(137.5, rX);  // 137.5 degrees
        backLeft.SetDirection(317.5, rX);  // 317.5 degrees
        backRight.SetDirection(222.5, rX); // 222.5 degrees
      }
        // else{
        //   frontLeft.turnAndDrive(angle-yaw, -speed, -rX, 315);
        //   frontRight.turnAndDrive(angle-yaw, speed, rX, 45);
        //   backLeft.turnAndDrive(angle-yaw, speed, rX, 135);
        //   backRight.turnAndDrive(angle-yaw, speed, rX, 225);
        // }
        
        
        SmartDashboard.putNumber("angle", angle);
        SmartDashboard.putNumber("Yaw", yaw);
    }

  public void autoDrive(double angle, double speed, double rotate) {
    oldAngle = angle;
    angle = Math.abs(angle);


    if (rotate == 0) {
      frontLeft.SetDirection(angle, -speed);
      frontRight.SetDirection(angle, speed);
      backLeft.SetDirection(angle, speed);
      backRight.SetDirection(angle, speed);
    } else {
      frontLeft.SetDirection(45, -rotate);
      frontRight.SetDirection(135, rotate);
      backLeft.SetDirection(315, rotate);
      backRight.SetDirection(225, rotate);
    }
  }

  private String getTurnDirection(double setAngle) {
    double yaw = gyro.getYaw();
    if (MathHelp.isEqualApprox(yaw, setAngle, 3)) {
      return "straight";
  }
    return 360 - setAngle + yaw < setAngle - yaw ? "counter-clockwise" : "clockwise";
  }

  public void autoTankDrive(double setSpeed, double setAngle) {
    double[] motorSpeeds = {setSpeed, setSpeed};

    final String directionToTurn = getTurnDirection(setAngle);

    switch (directionToTurn) {
      case "clockwise":
        System.out.println("turning clockwise towards " + setAngle);
        motorSpeeds[0] = 0.25;
        motorSpeeds[1] = -0.25;
        break;
      case "counter-clockwise":
        System.out.println("OPTIMIZATION: turning counter-clockwise towards " + setAngle);
        motorSpeeds[0] = 0.25;
        motorSpeeds[1] = -0.25;
        break;
      case "straight":
        System.out.println("driving straight towards " + setAngle);
        motorSpeeds[0] = setSpeed;
        motorSpeeds[1] = setSpeed;
        break;
    }

    frontLeft.SetDirection(0, -motorSpeeds[0]);
    frontRight.SetDirection(0, motorSpeeds[1]);
    backLeft.SetDirection(0, motorSpeeds[0]);
    backRight.SetDirection(0, motorSpeeds[1]);
  }

  public void autoTurn(double setAngle) {
    double yaw = gyro.getYaw();
    if (MathHelp.isEqualApprox(yaw, setAngle, setAngle)) {
      return;
    }
    double speed = 360 - setAngle + yaw < setAngle - yaw ? 0.5 : -0.5;
    frontLeft.SetDirection(45, speed);
    frontRight.SetDirection(135, speed);
    backLeft.SetDirection(225, speed);
    backRight.SetDirection(315, speed);
  }
}
