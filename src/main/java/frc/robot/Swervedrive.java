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

  double frontLeftOutputSpeed = 0;
  double frontRightOutputSpeed = 0;
  double backLeftOutputSpeed = 0;
  double backRightOutputSpeed = 0;

  double normalSpeed = 0;

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

  public void drive(double lX, double lY, double rX, boolean isRobotCentric) {
      double speed = Math.sqrt((lX*lX) + (lY*lY));

      double yaw = gyro.getYaw();

      
      
      // double angle = -Math.atan2(lY, lX); //lX/1
      double angle = -Math.atan(lX);
  
      angle *= 100;


      //angle += yaw;
  
      if((lX == 0 && lY == 0) && oldAngle != 0){
        angle = oldAngle;
      }
      // else{
      //   angle = 0;
      // }

      
      SmartDashboard.putNumber("Raw angle", angle);
      
      if (lX>=0 && lY>=0){ //1
        angle = 90 - angle - 90;
      }
      if(lX>0 && lY<0){//2
        angle = angle + 180;
      }
      if(lX<0 && lY>0){//4
        angle = 90-angle + 270;
      }
      if(lX<0 && lY<0){//3
        angle = angle + 180;
      }
      if(lY < 0 && angle == 0){
        angle = 180;
      }
      if(lY == 0 && lX < 0) {
        angle = 270;
      }
      if(lY == 0 && lX > 0) {
        angle = 90;
      }

        // if(speed < .1){
        //     speed = 0;
        // }

        oldAngle = angle;

        angle = Math.abs(angle);

        // double robotCentric = 1;
        // double robotCentric = !m_driverController.getRawButton(2) ? 1 : 0;

        // if (m_driverController.getRawButton(2))
        // {
        //   robotCentric = 0;
        // }
        // else if (!m_driverController.getRawButton(2))
        // {
        //   robotCentric = 1;
        // }

        // angle = Math.sqrt(angle*angle);
        // angle *= 2;
        // angle = 0;
        // speed *=0.25;

        SmartDashboard.putNumber("unmodified yaw", yaw);
        yaw = isRobotCentric ? yaw * 0: yaw;
        SmartDashboard.putNumber("modified yaw", yaw);
        SmartDashboard.putBoolean("isRobotCentric", isRobotCentric);
        // if(rX == 0){
        //     frontLeft.SetDirection(angle-yaw, -speed);
        //     frontRight.SetDirection(angle-yaw, speed);
        //     backLeft.SetDirection(angle-yaw, speed);
        //     backRight.SetDirection(angle-yaw, speed);
        // } else if (speed == 0){
        //   // Opposite diagonal axle distance = 33.2 inches
        //   frontLeft.SetDirection(42.5, -rX); //42.5 degrees
        //   frontRight.SetDirection(137.5, rX);  // 137.5 degrees
        //   backLeft.SetDirection(317.5, rX);  // 317.5 degrees
        //   backRight.SetDirection(222.5, rX); // 222.5 degrees
        // }

        frontLeftOutputSpeed = frontLeft.turnAndDrive(angle-yaw, -speed, -rX, 315);
        frontRightOutputSpeed = frontRight.turnAndDrive(angle-yaw, speed, rX, 45);
        backLeftOutputSpeed = backLeft.turnAndDrive(angle-yaw, speed, rX, 135);
        backRightOutputSpeed = backRight.turnAndDrive(angle-yaw, speed, rX, 225);
        
        normalSpeed = Math.max(Math.max(frontLeftOutputSpeed, frontRightOutputSpeed), Math.max(backLeftOutputSpeed, backRightOutputSpeed));
        
        frontLeft.setWheelSpeed(normalSpeed, frontLeftOutputSpeed);
        frontRight.setWheelSpeed(normalSpeed, frontRightOutputSpeed);
        backLeft.setWheelSpeed(normalSpeed, backLeftOutputSpeed);
        backRight.setWheelSpeed(normalSpeed, backRightOutputSpeed);
        
        SmartDashboard.putNumber("angle", angle);
        SmartDashboard.putNumber("Yaw", yaw);
    }

  public void autoDrive(double angle, double speed, double rotate) {
    oldAngle = angle;
    angle = Math.abs(angle);

    // angle = Math.sqrt(angle*angle);
    // angle *= 2;
    // angle = 0;
    // speed *=0.25;
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
}
