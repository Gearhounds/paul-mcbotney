

// 
// Last Updated: 10/19/23
// Updated by: Charley[x] Josh[o] Will[o]
// Change made: Attempted to have wheels stay at angle when no input is given
// Status of code: Works Great[o] It Works[x] Misbehaving [o]
// 





package frc.robot;


import com.ctre.phoenix.sensors.CANCoder;
// import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import com.revrobotics.SparkPIDController;
// import edu.wpi.first.wpilibj.AnalogInput;
// import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;

// import java.util.Timer;

import edu.wpi.first.wpilibj.XboxController;


public class WheelDrive {
    private CANSparkMax angleMotor;
    private CANSparkMax speedMotor;
    private PIDController anglePID;
    private SparkPIDController speedPID;
    private CANCoder encoder;
    // XboxController xControl = new XboxController(0);
    Joystick joystick1 = new Joystick(0);
    Joystick joystick2 = new Joystick(2);
    double outputSpeed = 0;
    
    
    
    public WheelDrive(int angleMotor, int speedMotor, int encoderID) {
        this.angleMotor = new CANSparkMax(angleMotor, MotorType.kBrushless);
        this.speedMotor = new CANSparkMax(speedMotor, MotorType.kBrushless);

        anglePID = new PIDController(.02, .001, 0);

        speedPID = this.speedMotor.getPIDController();
        
        encoder = new CANCoder(encoderID);

        anglePID.enableContinuousInput(-180, 180);      

    }


    public void Reset() {
        anglePID.reset();
    }

    boolean reverse = false;

    public void SetDirection(double setAngle, double speed) {
        

        
        

        double currentAngle = -(this.encoder.getAbsolutePosition());
        
        // setAngle = MathHelp.pickCloserAngleDeg(currentAngle, setAngle, setAngle-180);

        double currentAngleRadian = currentAngle * (Math.PI / 180);
        double setAngleRadians = setAngle * (Math.PI / 180);
        double oppositeSetAngleRadians = (setAngle - 180) * (Math.PI / 180);

        // if (reverse){
        //     setAngle -= 180;
        // }

        if (Math.abs(MathHelp.differenceBetweenAngles(currentAngleRadian, oppositeSetAngleRadians)) < Math.abs(MathHelp.differenceBetweenAngles(currentAngleRadian, setAngleRadians)))
        {
        //   return Bngle;
            setAngle -= 180;
            reverse = !reverse;
            speed *= -1;
        }

        if (reverse){
        }
        
        double pidOutput = anglePID.calculate(currentAngle, setAngle);
        pidOutput *= 0.45;
        
        // if((setAngle == 0)&&(xControl.getLeftY() < 0))
        
        // {

        //     setAngle = currentAngle;

        // }






        this.angleMotor.set((pidOutput));
        
        // if(!MathHelp.isEqualApprox(currentAngle, setAngle, .1)){
        //     speed = 0;
        // }
        this.speedMotor.set(speed);
        
        SmartDashboard.putNumber("pid number", pidOutput);
        SmartDashboard.putNumber("encoder pos", currentAngle);
        SmartDashboard.putNumber("Speed", speed);

        
    }

    public void setAngle(double angle) {
        double position = -(this.encoder.getAbsolutePosition());
        // position += 45;

        MathHelp.pickCloserAngle(position, angle, -angle);
        
        double pidOutput = anglePID.calculate(position, angle);
        pidOutput *= 0.45;
        
        if (MathHelp.isEqualApprox(angle, position, .25))
        {
            pidOutput = 0;
        }

        this.angleMotor.set((pidOutput));
    }
    
    
    public double turnAndDrive(double strafeAngle, double strafeSpeed, double turnSpeed, double turnAngle) {
        
        strafeAngle *= Math.PI/180;
        turnAngle *= Math.PI/180;

        double x1 = Math.cos(strafeAngle) * strafeSpeed;
        double y1 = Math.sin(strafeAngle) * strafeSpeed;

        double x2 = Math.cos(turnAngle) * turnSpeed;
        double y2 = Math.sin(turnAngle) * turnSpeed;

        double x3 = x1 + x2;
        double y3 = y1 + y2;


        double setAngle = Math.atan2(x3, y3);
        setAngle *= 180/Math.PI;
        double currentAngle = -(this.encoder.getAbsolutePosition());

        double speed = Math.hypot(x3, y3);


        double wheelAngle = currentAngle - setAngle;
        wheelAngle = Math.abs(wheelAngle);

        double currentAngleRadian = currentAngle * (Math.PI / 180);
        double setAngleRadians = setAngle;
        double oppositeSetAngleRadians = setAngle - Math.PI;

        

        if (Math.abs(MathHelp.differenceBetweenAngles(currentAngleRadian, oppositeSetAngleRadians)) < Math.abs(MathHelp.differenceBetweenAngles(currentAngleRadian, setAngleRadians)))
        {
        //   return Bngle;
            setAngle -= 180;
            reverse = !reverse;
            strafeSpeed *= -1;
        }

        if (reverse){
        }

        if (!(wheelAngle <= 180)) {
            speed = -speed;
        }
        
        
        // if((setAngle == 0)&&(xControl.getLeftY() < 0)) { setAngle = currentAngle;}
        
        
        
        
        double pidOutput = anglePID.calculate(currentAngle, setAngle);
        pidOutput *= 0.45;
        
        this.angleMotor.set(pidOutput);
        

        double distanceFromSetAngle = Math.abs(MathHelp.differenceBetweenAngles(currentAngle, setAngle));
        double speedScalar = MathHelp.map(distanceFromSetAngle, 0, 90, 1, 0);

        MathUtil.clamp(speedScalar, 0, 1);
        
        outputSpeed = speed * speedScalar;

        return outputSpeed;
    }

    public void setWheelSpeed(double normalizedSpeed, double setSpeed) {
        if (Math.abs(joystick1.getX()) < 0.075 && Math.abs(joystick1.getY()) < 0.075 && Math.abs(joystick2.getX()) < 0.075)
        {
            this.speedMotor.set(0);
        }
        else
        {
            this.speedMotor.set(setSpeed / normalizedSpeed);
        }
        
    }

}