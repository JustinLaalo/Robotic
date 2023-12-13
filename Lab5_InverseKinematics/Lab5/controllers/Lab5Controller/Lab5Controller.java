// Author: NAME (SN: STUDENT NUMBER)
//Justin Laalo (SN: 101232038)
//Sunny Parmar (SN: 101159023)

import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.Robot;
import com.cyberbotics.webots.controller.PositionSensor;

public class Lab5Controller {

  static final double  MAX_SPEED      = 0.25;  //6.28 is maximum but value of 1 is required for simulator to work properly for kinematics
  static final double  WHEEL_RADIUS   = 2.05; // cm 
  static final double  WHEEL_BASE     = 5.80; // cm
  
  static Robot           Epuck;
  static Motor           LeftMotor;
  static Motor           RightMotor;
  static PositionSensor  rightEncoder;
  static double          PreviousReading = 0;
  static int             TimeStep;


  // The locations to visit in sequence, and starting angle and position
  static int           x[] = {0, 30, 30, -10, 10, -10, -60, -50, -40, -30, -20, 0};
  static int           y[] = {0, 30, 60, 70, 50, 50, 40, 30, 30, 20, 20, 0};
  static double startAngle = 90;


  // Set each motor to a specific speed and wait until the left sensor reaches 
  // the specified number of radians.
  private static void move(double leftSpeed, double rightSpeed, double thisManyRadians) {
    LeftMotor.setVelocity(leftSpeed);
    RightMotor.setVelocity(rightSpeed);
    while(true) {
      double reading = rightEncoder.getValue() - PreviousReading;
      if ((thisManyRadians > 0) && (reading >= thisManyRadians)) 
        break;
      if ((thisManyRadians < 0) && (reading <= thisManyRadians)) 
        break;
      Epuck.step(TimeStep);
    }
    LeftMotor.setVelocity(0);
    RightMotor.setVelocity(0);
    Epuck.step(TimeStep);
    PreviousReading = rightEncoder.getValue();
    Epuck.step(TimeStep);
  }
  
	
  public static void main(String[] args) {
    Epuck = new Robot();
    TimeStep = (int) Math.round(Epuck.getBasicTimeStep());
    double a1 = 90, a2 = 0.0, ac = 0.0,ls =0.0, rs =0.0, d = 0.0, xc = 0.0, yc =0.0;
    // Get the motors
    LeftMotor = Epuck.getMotor("left wheel motor");
    RightMotor = Epuck.getMotor("right wheel motor");
    LeftMotor.setPosition(Double.POSITIVE_INFINITY);// indicates continuous rotation servo
    RightMotor.setPosition(Double.POSITIVE_INFINITY);// indicates continuous rotation serv
    LeftMotor.setVelocity(0);
    RightMotor.setVelocity(0);
    
    // Get the encoders
    rightEncoder = Epuck.getPositionSensor("right wheel sensor");
    rightEncoder.enable(TimeStep);
       
    // Travel through the points in the array one at a time in sequence
    for(int i = 1; i < x.length; i++){
      
      xc = x[i] - x[i-1];
      yc = y[i] - y[i-1];
      a2 = Math.atan2(yc,xc)*180/Math.PI;
      ac = (a2-a1)%360;
      
      
      if(ac < -180){
        ac = ac + 360;
      }else if(ac > 180){
        ac = ac -360;
      }
      
      a1 += ac;
      
      ac = Math.toRadians(ac);
      ac = 1.41463415*ac; //try to find a way to get number using wheele base and radius
      if(ac > 0){
        rs = MAX_SPEED;
        ls = -1 * MAX_SPEED;
      }else{
        rs = -1 * MAX_SPEED;
        ls = MAX_SPEED;
      }
      move(ls,rs,ac);
      
      ac = 0.0;
      d = Math.sqrt(Math.pow(xc,2) + Math.pow(yc,2));
      ac = d/WHEEL_RADIUS;
      rs = MAX_SPEED;
      ls = MAX_SPEED;
      move(ls,rs,ac);
      
       
    }
  }
}
