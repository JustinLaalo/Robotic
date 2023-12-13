// Author: NAME (SN: STUDENT NUMBER)
//Justin Laalo (SN: 101232038)
//Chung Hayden (SN: 101222796)
import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.Robot;
import com.cyberbotics.webots.controller.PositionSensor;

public class Lab4Controller {

  static final double  MAX_SPEED      = 1;  //6.28 is maximum but value of 1 is required for simulator to work properly for kinematics
  static final double  WHEEL_RADIUS   = 2.05; // cm 
  static final double  WHEEL_BASE     = 5.80; // cm
  static final double  CURVE_VALUE    = 20.2510945; // degrees // WHEEL_RADIUS/WHEEL_BASE in radians
  static final double  CURVE_VALUE2   = 2.9; // degrees
  
  static Robot           Epuck;
  static Motor           LeftMotor;
  static Motor           RightMotor;
  static PositionSensor  LeftEncoder;
  static PositionSensor  RightEncoder;
  static double          LeftReading, RightReading, PreviousLeft, PreviousRight, PreviousAngle;
  static int             TimeStep;


  // Set each motor to a specific speed and wait for te given amount of seconds
  // Then stop the motors and update the position sensor readings.
  private static void Move(double leftSpeed, double rightSpeed, double seconds) {
    LeftMotor.setVelocity(leftSpeed);
    RightMotor.setVelocity(rightSpeed);
    for (double time = 0.0; time<seconds; time += (TimeStep/1000.0)) {
      Epuck.step(TimeStep);
    }
    LeftMotor.setVelocity(0);
    RightMotor.setVelocity(0);
    Epuck.step(TimeStep);
    LeftReading = LeftEncoder.getValue() - PreviousLeft;
    RightReading = RightEncoder.getValue() - PreviousRight;
    PreviousLeft = LeftEncoder.getValue();
    PreviousRight = RightEncoder.getValue();
    Epuck.step(TimeStep);
  }
  
  
  public static void main(String[] args) {
    Epuck = new Robot();
    TimeStep = (int) Math.round(Epuck.getBasicTimeStep());
    
    System.out.println("Time Step = " + TimeStep);

    // Get the motors
    LeftMotor = Epuck.getMotor("left wheel motor");
    RightMotor = Epuck.getMotor("right wheel motor");
    LeftMotor.setPosition(Double.POSITIVE_INFINITY);// indicates continuous rotation servo
    RightMotor.setPosition(Double.POSITIVE_INFINITY);// indicates continuous rotation serv
    LeftMotor.setVelocity(0);
    RightMotor.setVelocity(0);
    
    // Get the encoders
    LeftEncoder = Epuck.getPositionSensor("left wheel sensor");
    RightEncoder = Epuck.getPositionSensor("right wheel sensor");
    LeftEncoder.enable(TimeStep);
    RightEncoder.enable(TimeStep);
    PreviousLeft = 0; PreviousRight = 0;
    
    // Store the (x, y) location amd angle (degrees) estimate
    double x = 0.0, y = 0.0, a = 90.0, r = 0.0, td = 0.0,a2 =0.0;
    
    
    // Move the robot forward for 5 seconds
    System.out.printf("x = %2.1f, y = %2.1f, a = %2.1f\n", x,y,a);
    
    Move(MAX_SPEED, MAX_SPEED, 5);
    td = (LeftReading * WHEEL_RADIUS);
    x = x + td * (Math.cos(Math.toRadians(a)));
    y = y + td * (Math.sin(Math.toRadians(a)));
    a = a;
    PreviousAngle = a;
    System.out.printf("x = %2.1f, y = %2.1f, a = %2.1f\n", x,y,a);
    
    // Spin the robot right for 6 seconds
    Move(MAX_SPEED, -MAX_SPEED, 6);
    x = x;
    y = y;
    a = (RightReading - LeftReading) * Math.toDegrees(WHEEL_RADIUS/WHEEL_BASE);
    if (a > 180) {a = a - 360;}
    else if (a < -180) {a = a + 360;}
    a = a + PreviousAngle;
    PreviousAngle = a;
    
    System.out.printf("x = %2.1f, y = %2.1f, a = %2.1f\n", x,y,a);
    // Curve the robot right for 10 seconds with right speed 0.2 less than full left speed
    Move(MAX_SPEED, MAX_SPEED-0.2, 10);

    r = (WHEEL_BASE * (LeftReading/(RightReading - LeftReading))+(WHEEL_BASE/2));
    a = ((RightReading- LeftReading) * Math.toDegrees(WHEEL_RADIUS/WHEEL_BASE));
    a2 = a;
    if (a > 180) {a = a - 360;}
    else if (a < -180) {a = a + 360;}
    x = (r * (Math.cos(Math.toRadians(a))) * (Math.sin(Math.toRadians(PreviousAngle))) + r *(Math.cos(Math.toRadians(PreviousAngle))) * (Math.sin(Math.toRadians(a))) + x - r *  (Math.sin(Math.toRadians(PreviousAngle))));
    y = (r * (Math.sin(Math.toRadians(a))) * (Math.sin(Math.toRadians(PreviousAngle))) - r *(Math.cos(Math.toRadians(PreviousAngle))) * (Math.cos(Math.toRadians(a))) + y + r *  (Math.cos(Math.toRadians(PreviousAngle))));
    a = a + PreviousAngle;
    PreviousAngle = a;
    System.out.printf("x = %2.1f, y = %2.1f, a = %2.1f,r = %2.1f ,td = %2.1f\n", x,y,a,r, a2);

    // Curve the robot left for 20 seconds with left speed 1/3 of full right speed
    Move(MAX_SPEED/3, MAX_SPEED, 20);
    
    r = (WHEEL_BASE * (LeftReading/(RightReading - LeftReading))+(WHEEL_BASE/2));
    a = ((RightReading- LeftReading) * Math.toDegrees(WHEEL_RADIUS/WHEEL_BASE));
    a2 = a;
    if (a > 180) {a = a - 360;}
    else if (a < -180) {a = a + 360;}
    x = (r * (Math.cos(Math.toRadians(a))) * (Math.sin(Math.toRadians(PreviousAngle))) + r *(Math.cos(Math.toRadians(PreviousAngle))) * (Math.sin(Math.toRadians(a))) + x - r *  (Math.sin(Math.toRadians(PreviousAngle))));
    y = (r * (Math.sin(Math.toRadians(a))) * (Math.sin(Math.toRadians(PreviousAngle))) - r *(Math.cos(Math.toRadians(PreviousAngle))) * (Math.cos(Math.toRadians(a))) + y + r *  (Math.cos(Math.toRadians(PreviousAngle))));
    a = a + PreviousAngle;
    PreviousAngle = a;
    System.out.printf("x = %2.1f, y = %2.1f, a = %2.1f,r = %2.1f ,td = %2.1f\n", x,y,a,r,a2);

    // Move the robot forward for 10 seconds
    Move(MAX_SPEED, MAX_SPEED, 10);
    td = (LeftReading * WHEEL_RADIUS);
    x = x + td * (Math.cos(Math.toRadians(a)));
    y = y + td * (Math.sin(Math.toRadians(a)));
    a = a;
    PreviousAngle = a;
    System.out.printf("x = %2.1f, y = %2.1f, a = %2.1f\n", x,y,a);

    // Spin the robot left for 7.5 seconds
    Move(-MAX_SPEED, MAX_SPEED, 7.5);
    x = x;
    y = y;
    a = (RightReading - LeftReading) * Math.toDegrees(WHEEL_RADIUS/WHEEL_BASE);
    if (a > 180) {a = a - 360;}
    else if (a < -180) {a = a + 360;}
    a = a + PreviousAngle;
    PreviousAngle = a;

    System.out.printf("x = %2.1f, y = %2.1f, a = %2.1f\n", x,y,a);

    // Move the robot forward for 20 seconds
    Move(MAX_SPEED, MAX_SPEED, 20);
    td = (LeftReading * WHEEL_RADIUS);
    x = x + td * (Math.cos(Math.toRadians(a)));
    y = y + td * (Math.sin(Math.toRadians(a)));
    a = a;
    PreviousAngle = a;
    System.out.printf("x = %2.1f, y = %2.1f, a = %2.1f\n", x,y,a);
  }
}
