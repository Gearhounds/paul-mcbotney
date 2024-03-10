package frc.robot;

public class MathHelp {
    public static double differenceBetweenAngles(double Angle, double Bngle){
        Angle = (Angle % (2 * Math.PI) + 2 * Math.PI) % (2 * Math.PI);
        Bngle = (Bngle % (2 * Math.PI) + 2 * Math.PI) % (2 * Math.PI);
        
        double returnVal = Bngle - Angle;
        
        if (returnVal > Math.PI) returnVal -= 2 * Math.PI;
        else if (returnVal < -Math.PI) returnVal += 2 * Math.PI;
        
        if (isEqualApprox(Angle,Bngle,0.01)) returnVal = 0;
        if (isEqualApprox(Angle,Bngle - (2*Math.PI),0.01)) returnVal = 0;
        
        return returnVal;
    }
    
    public static boolean isEqualApprox(double a, double b, double tolerance){
        return (a + tolerance > b && a - tolerance < b);
    }

    public static double pickCloserAngle(double myAngle, double Angle,double Bngle){

        if (Math.abs(differenceBetweenAngles(myAngle, Bngle)) < Math.abs(differenceBetweenAngles(myAngle, Angle)))
        {
          return Bngle;
        }
        return Angle;
      
      
      }
      
      public double fixRads(double rad){
        return (rad % (2 * Math.PI) + (2 * Math.PI)) % (2 * Math.PI);
      }

      static public final double map(double value, 
      double istart, 
      double istop, 
      double ostart, 
      double ostop) {
      return ostart + (ostop - ostart) * ((value - istart) / (istop - istart));
      }

      public static double pickCloserAngleDeg(double myAngle, double Angle,double Bngle){

        double myAng = myAngle * (Math.PI/180);
        double Ang = Angle * (Math.PI/180);
        double Bng = Bngle * (Math.PI/180);

        if (Math.abs(differenceBetweenAngles(myAng, Bng)) < Math.abs(differenceBetweenAngles(myAng, Ang)))
        {
          return Bngle;
        }else{
          return Angle;
        }
      
      
      }


}

