//Breakout room: 27
//Brendan Bower (SN: 101220993)
//Justin Laalo (SN: 101232039)
 
import com.cyberbotics.webots.controller.DistanceSensor;
import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.PositionSensor;
import com.cyberbotics.webots.controller.Field;
import com.cyberbotics.webots.controller.Node;
import com.cyberbotics.webots.controller.Supervisor;
import com.cyberbotics.webots.controller.Compass;
import com.cyberbotics.webots.controller.Display;

public class Lab8Controller {

  // Various modes that the robot may be in
  static final byte    STRAIGHT = 0;
  static final byte    SPIN_LEFT = 1;
  static final byte    PIVOT_RIGHT = 2;
  static final byte    CURVE_LEFT = 3;
  static final byte    CURVE_RIGHT = 4;
  
  static final double  MAX_SPEED      = 1;    // maximum speed of the epuck robot
  static final double  WHEEL_RADIUS   = 2.05; // cm 
  static final double  WHEEL_BASE     = 5.80; // cm

  static PositionSensor  LeftEncoder;
  static PositionSensor  RightEncoder;
  static double          LeftReading, RightReading, PreviousLeft, PreviousRight;
  
  // Store the (x, y) location and angle (degrees) estimate
  static double X, Y, A, R, TD,Xactual,Yactual,Aactual;


  // Read the compass
  private static int getCompassReadingInDegrees(Compass compass) {
    
    // FILL IN YOUR CODE HERE TO READ THE COMPASS AND CONVERT TO DEGREES
    double compassReadings[] = compass.getValues();
    double rad = Math.atan2(compassReadings[0], compassReadings[1]);
    double bearing = (rad - Math.PI/2) / Math.PI * 180.0;
    if (bearing > 180)
    bearing = 360 - bearing;
    if (bearing < -180)
    bearing = 360 + bearing;
    return (int)(Math.ceil(bearing / 5) * 5);
  }
      
      
  // Display the actual value as well as the estimate
  static void displayEstimate(TrackerApp tracker, Field translationField) {
    // Display the actual position [PART 2]
    double values[] = translationField.getSFVec3f();

    Xactual = (values[0]*100);
    Yactual = -(values[2]*100); // Need to negate the Y value
    Aactual = 180;
    tracker.addActualLocation((int)Xactual, (int)Yactual);
    
    // Display the estimated position [PART 3]
    // WRITE CODE HERE
    tracker.addEstimatedLocation((int)X, (int)Y);
  }
  
  
  // Update the estimated position
  static void updateEstimate(byte previousMode, Compass compass) {
    // Subtract the previous readings so that things start fresh from the last calcuation made.
    LeftReading = LeftEncoder.getValue() - PreviousLeft;
    RightReading = RightEncoder.getValue() - PreviousRight;
    PreviousLeft = LeftEncoder.getValue();
    PreviousRight = RightEncoder.getValue();
    
    switch(previousMode) {
        case SPIN_LEFT:
          //A = A + ((RightReading-LeftReading)*WHEEL_RADIUS/WHEEL_BASE/Math.PI*180);
          A = getCompassReadingInDegrees(compass);
          break;
          
        case PIVOT_RIGHT:
        case CURVE_LEFT:
        case CURVE_RIGHT:
          if (RightReading != LeftReading){
            R = WHEEL_BASE*(LeftReading/(RightReading-LeftReading)) + WHEEL_BASE/2;
            TD = (RightReading-LeftReading)*WHEEL_RADIUS/WHEEL_BASE/Math.PI*180;
            X = X + (R*Math.cos(Math.toRadians(TD))*Math.sin(Math.toRadians(A))) + (R*Math.cos(Math.toRadians(A))*Math.sin(Math.toRadians(TD))) -(R*Math.sin(Math.toRadians(A)));
            Y = Y + (R*Math.sin(Math.toRadians(TD))*Math.sin(Math.toRadians(A))) -(R*Math.cos(Math.toRadians(A))*Math.cos(Math.toRadians(TD))) + (R*Math.cos(Math.toRadians(A)));
            //A = A + TD;
            A = getCompassReadingInDegrees(compass);
            // if (A > 180) A = A - 360;
            // if (A < -180) A = 360 + A;
            }else{
              X = X + (LeftReading * WHEEL_RADIUS) * Math.cos(Math.toRadians(A));
              Y = Y + (LeftReading * WHEEL_RADIUS) * Math.sin(Math.toRadians(A));
            }
          break;
          
        default:
          X = X + (LeftReading * WHEEL_RADIUS) * Math.cos(Math.toRadians(A));
          Y = Y + (LeftReading * WHEEL_RADIUS) * Math.sin(Math.toRadians(A));
    }
  }
  
  public static void main(String[] args) {
    Supervisor robot = new Supervisor();
    int timeStep = (int) Math.round(robot.getBasicTimeStep());
    
    Display d = robot.getDisplay("display");
    System.out.println(d.getWidth());
    TrackerApp  tracker = new TrackerApp(d);
    
    // Code required for being able to get the robot's location
    Node robotNode = robot.getSelf();
    Field   translationField = robotNode.getField("translation");

    // Get the motors
    Motor leftMotor = robot.getMotor("left wheel motor");
    Motor rightMotor = robot.getMotor("right wheel motor");
    leftMotor.setPosition(Double.POSITIVE_INFINITY);// indicates continuous rotation servo
    rightMotor.setPosition(Double.POSITIVE_INFINITY);// indicates continuous rotation servo

    // Get and enable the sensors
    DistanceSensor leftAheadSensor = robot.getDistanceSensor("ps7"); 
    DistanceSensor rightAheadSensor = robot.getDistanceSensor("ps0"); 
    DistanceSensor rightAngledSensor = robot.getDistanceSensor("ps1"); 
    DistanceSensor rightSideSensor = robot.getDistanceSensor("ps2"); 
    leftAheadSensor.enable(timeStep);
    rightAheadSensor.enable(timeStep);
    rightAngledSensor.enable(timeStep);
    rightSideSensor.enable(timeStep);

    // Get the encoders
    LeftEncoder = robot.getPositionSensor("left wheel sensor");
    RightEncoder = robot.getPositionSensor("right wheel sensor");
    LeftEncoder.enable(timeStep);
    RightEncoder.enable(timeStep);
    PreviousLeft = 0; PreviousRight = 0;
    
    // Get the Compass sensor
    Compass  compass = robot.getCompass("compass");
    compass.enable(timeStep);


    // Initialize the logic variable for turning
    byte    currentMode = STRAIGHT;
    byte    previousMode = STRAIGHT;
    double  leftSpeed, rightSpeed;
    
    // Set the first estimate to match the current location
    // WRITE CODE HERE
    
    double values[] = translationField.getSFVec3f();
    X = (values[0]*100);
    Y = -(values[2]*100); // Need to negate the Y value
    //A = 180;
    System.out.printf("Robot starts at (x, y) = (%2.1f, %2.1f, %3.0f degrees)\n", X+d.getWidth()/2, Y+d.getHeight()/2+15, A);
    displayEstimate(tracker, translationField);
    
    while (robot.step(timeStep) != -1) {
      // SENSE: Read the distance sensors
      boolean sideTooClose  = rightSideSensor.getValue() > 300;
      boolean sideTooFar  = rightSideSensor.getValue() < 80;
      boolean frontTooClose  = (rightAheadSensor.getValue() > 80) || (leftAheadSensor.getValue() > 80) || (rightAngledSensor.getValue() > 80);
      boolean lostContact  = rightSideSensor.getValue() < 80;
      
     // System.out.printf("%d\n", getCompassReadingInDegrees(compass));
      
      // THINK: Check for obstacle and decide how we need to turn      
      switch (currentMode) {
        case STRAIGHT: 
          if (lostContact) { currentMode = PIVOT_RIGHT; break; }
          if (sideTooFar) { currentMode = CURVE_RIGHT; break; }
          if (sideTooClose) { currentMode = CURVE_LEFT; break; }
          if (frontTooClose) { currentMode = SPIN_LEFT; break; }
          break;
        case CURVE_RIGHT:
          if (!sideTooFar) { currentMode = STRAIGHT; break; }
          if (sideTooClose) { currentMode = CURVE_LEFT; break; }
          if (frontTooClose) { currentMode = SPIN_LEFT; break; }
          break;
        case PIVOT_RIGHT: 
          if (sideTooClose) { currentMode = CURVE_LEFT; break; }
          if (frontTooClose) { currentMode = SPIN_LEFT; break; }
          break;
        case SPIN_LEFT:
          if (!frontTooClose) { currentMode = STRAIGHT; break; }
          break;
        case CURVE_LEFT: 
          if (!sideTooClose) { currentMode = STRAIGHT;  break; }
          break;
       }
       
       
       // WRITE CODE HERE
       if (currentMode != previousMode) {
         updateEstimate(previousMode, compass);
         if (currentMode != SPIN_LEFT)
           displayEstimate(tracker, translationField);
       }
       
       
       
       
       
       
       previousMode = currentMode;
       
      // REACT: Move motors accordingly
      switch(currentMode) {
        case SPIN_LEFT:
          leftSpeed  = -1 * MAX_SPEED;
          rightSpeed = 1 * MAX_SPEED;
          break;
        case PIVOT_RIGHT:
          leftSpeed  = 1 * MAX_SPEED;
          rightSpeed = 0.25 * MAX_SPEED;
          break;
        case CURVE_LEFT:
          leftSpeed  = 0.9 * MAX_SPEED;
          rightSpeed = 1 * MAX_SPEED;
          break;
        case CURVE_RIGHT:
          leftSpeed  = 1 * MAX_SPEED;
          rightSpeed = 0.9 * MAX_SPEED;
          break;
        default:
          leftSpeed  = 1 * MAX_SPEED;
          rightSpeed = 1 * MAX_SPEED;
          break;
      }
      leftMotor.setVelocity(leftSpeed);
      rightMotor.setVelocity(rightSpeed);
    }
  }
}
