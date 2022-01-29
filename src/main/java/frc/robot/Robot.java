package frc.robot;
    //WINCH ENCODER NOT WORKING
    //FIGURE OUT A WAY TO CHECK THE WINCH IN PIT
import edu.wpi.first.wpilibj.Joystick;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Limelight.State;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

public class Robot extends TimedRobot {
/*
///////////////////////////////////////////////////////////
//                        DRIVE                          //
///////////////////////////////////////////////////////////

CAN SPARK MAX DRIVE IDS

LEFT DRIVE: 4, 5, 6 

RIGHT DRIVE: 1, 2, 3

SHIFTER: 1 , 6

///////////////////////////////////////////////////////////
//                    BALL MANAGEMENT                    //
///////////////////////////////////////////////////////////

INTAKE BAG MOTOR: 1 (VICTOR)

CONVEYOR SENSOR: 0 (DIGITAL IO)

CONVEYOR BAG MOTOR: 5 (VICTOR)

TRANSITIONAL WHEEL BAG MOTOR: 8 (TALON SRX)

PICK UP SENSOR: 1 (DIGITAL IO)

DROP DOWN SENSOR: 2 (DIGITAL IO)

///////////////////////////////////////////////////////////
//                       SHOOTER                         //
///////////////////////////////////////////////////////////

SHOOTER MOTOR 1: 13 (CAN SPARK MAX)

SHOOTER MOTOR 2: 14 (CAN SPARK MAX)

LAZY SUSAN BAG MOTOR: 3 (TALON SRX)

FEEDER BAG MOTOR: 4 (TALON SRX)

RIGHT LIMIT (LIMIT SWITCH): 3 (DIGITAL IO)

///////////////////////////////////////////////////////////
//                     WINCH & HANG                      //
///////////////////////////////////////////////////////////

WINCH MOTOR: 6 (TALON SRX)

HANG MOTOR: 0 (TALON SRX)

  */
  
  // JOYSTICKS //                       
  private Joystick leftJoyStick;         // LEFT DRIVE JOYSTICK
  private Joystick rightJoyStick;        // RIGHT DRIVE JOYSTICK
  //private Joystick stickControl;       //ARCADE JOYSTICK
  private Joystick mechJoyStick;         // MECHANISM JOYSTICK

  // DRIVE //
  private CANSparkMax leftFront;         // LEFT SIDE MOTORS 
  private CANSparkMax leftBack;
  private CANSparkMax rightFront;        // RIGHT SIDE MOTORS
  private CANSparkMax rightBack;
 
  private AHRS gyro;                     // NAVX GYRO
  private CANEncoder leftFrontEnc;       // LEFT FRONT ENCODERS
  private CANEncoder rightFrontEnc;      // RIGHT FRONT ENCODERS

  private DoubleSolenoid doubleSolenoid; // SHIFTER FOR THE DRIVE
 
  Drive driveClass;                      // DRIVE CLASS VARIABLE
  Shifter shifterClass;                  // SHIFTER CLASS VARIABLE

  // CAMERA //

  private Camera camera;
  // INTAKE //
  private WPI_VictorSPX intakeBagMotor;  // MOTOR FOR THE INTAKE

  private DoubleSolenoid intakePiston;  // DOUBLE SOLENOID FOR THE PISTONS OF THE INTAKE
  
  // CONVEYOR BELT: UNKNOWN PORTS //
  private WPI_VictorSPX conveyorBeltBagMotor;      // MOTOR FOR THE CONVEYOR

  private DigitalInput conveyorManagementSensor;  // SENSOR FOR COUNTING THE BALLS COMING IN THROUGH THE CONVEYOR

  // TRANSITIONAL WHEEL: UNKNOWN PORTS //
  private WPI_TalonSRX transWheelBagMotor;        // MOTOR FOR THE TRANSITION WHEEL

  private DigitalInput pickUpManagementSensor;    // SENSOR FOR STORING BALLS IN THE TRANSITIONAL WHEEL
  private DigitalInput dropDownManagementSensor;  // SENSOR FOR THE DROP DOWN POINT 

  // BALL MANAGEMENT SYSTEM //
  private Timer conveyorTimer;             // TIMER FOR CONVEYOR DELAY
  private Timer pickUpTimer;               // TIMER FOR INTAKING DELAY
  private Timer dropDownTimer;             // TIMER FOR FEEDING DELAY

  BallManagement ballManagementClass;      // BALL MANAGEMENT SYSTEM CLASS VARAIBLE

  // WINCH: UNKNOWN PORTS //
  private WPI_TalonSRX winchMotor;         // MOTOR FOR THE WINCH

  private TalonEncoder winchEnc;                // ENCODER FOR THE WINCH

  // HANG: UNKNOWN PORTS //
  private WPI_TalonSRX elevatorLiftMotor;  // MOTOR FOR THE LIFT

  private DoubleSolenoid armSolenoid;      // HANG PISTON 

  private TalonEncoder elevatorLiftEnc;         // ENCODER FOR THE LIFT  

  Hang hangClass; // HANG CLASS VARIABLE
  
  // SHOOTER //
  private WPI_TalonSRX feederBagMotor;     // FEEDER WHEEL
  private WPI_TalonSRX lazySusanBagMotor;  // LAZY SUSAN
  private CANSparkMax shooterWheel1;       // SHOOTER MOTOR 1
  private CANSparkMax shooterWheel2;       // SHOOTER MOTOR 2

  private DigitalInput rightLimit;         // LIMIT SWITCH FOR FINDRIGHT
  private DigitalInput leftLimit;

  private CANEncoder shooterEnc;           // ENCODER FOR THE SHOOTER
  private TalonEncoder lazySusanEnc;       // ENCODER FOR THE LAZY SUSAN
  private TalonEncoder feederEnc;      // ENCODER FOR THE FEEDER

  Shooter shooterClass; // SHOOTER CLASS VARIABLE

  // LIMELIGHT //
  Limelight limelightClass;
  boolean driving = true;

  // ROUTINES //
  Routines routines;


  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {   

///////////////////////////////////////////////////////////
//                        CAMERA                         //
///////////////////////////////////////////////////////////

    camera = new Camera();
    camera.init();

///////////////////////////////////////////////////////////
//                        JOYSTICKS                      //
///////////////////////////////////////////////////////////

    leftJoyStick = new Joystick(0);
    rightJoyStick = new Joystick(1);

    //stickControl = new Joystick(0);

    mechJoyStick = new Joystick(2);
    
///////////////////////////////////////////////////////////
//                        DRIVE                          //
///////////////////////////////////////////////////////////
    //CHANGE LATER BECAUSE COMP ROBOT
    leftFront = new CANSparkMax(11, MotorType.kBrushless);                                  // CAN IDS LEFT: 11, 12 //    
    leftBack = new CANSparkMax(12, MotorType.kBrushless);
    rightFront = new CANSparkMax(7, MotorType.kBrushless);                                  // CAN IDS RIGHT: 7, 8 //
    rightBack = new CANSparkMax(8, MotorType.kBrushless);

    gyro = new AHRS(SPI.Port.kMXP);
    
    leftFrontEnc = new CANEncoder(leftFront);                                               // ENCODER FOR THE LEFT FRONT 
    rightFrontEnc = new CANEncoder(rightFront);                                             // ENCODER FOR THE RIGHT FRONT 
    
    doubleSolenoid = new DoubleSolenoid(1, 6);                                              // DRIVER SHIFTER

    driveClass = new Drive(rightFront, rightBack, leftFront, leftBack);                     //  DRIVE  //
    shifterClass = new Shifter(doubleSolenoid);                                             // SHIFTER //
  
///////////////////////////////////////////////////////////
//                     BALL MANAGEMENT                   //
///////////////////////////////////////////////////////////

    // INTAKE //
    
    intakeBagMotor = new WPI_VictorSPX(1);
    intakePiston = new DoubleSolenoid(2, 5);
    
    // CONVEYOR BELT // 
    
    conveyorBeltBagMotor = new WPI_VictorSPX(5);
    conveyorManagementSensor = new DigitalInput(0);
    
    // TRANSITIONAL WHEEL //

    transWheelBagMotor = new WPI_TalonSRX(8);
    pickUpManagementSensor = new DigitalInput(1);
    dropDownManagementSensor = new DigitalInput(2);

    conveyorTimer = new Timer();
    pickUpTimer = new Timer();
    dropDownTimer = new Timer();

    ballManagementClass = new BallManagement(intakeBagMotor, conveyorBeltBagMotor, transWheelBagMotor, intakePiston, conveyorManagementSensor, pickUpManagementSensor, dropDownManagementSensor, conveyorTimer, pickUpTimer, dropDownTimer);
    
///////////////////////////////////////////////////////////
//                      WINCH & HANG                     //
///////////////////////////////////////////////////////////


    winchMotor = new WPI_TalonSRX(6);

    winchEnc = new TalonEncoder(winchMotor);

    elevatorLiftMotor = new WPI_TalonSRX(0);

    elevatorLiftEnc = new TalonEncoder(elevatorLiftMotor);

    armSolenoid = new DoubleSolenoid(0, 7);

    hangClass = new Hang(elevatorLiftMotor, winchMotor, armSolenoid, elevatorLiftEnc, winchEnc);
///////////////////////////////////////////////////////////
//                       LIMELIGHT                        //
///////////////////////////////////////////////////////////

    limelightClass = new Limelight();
///////////////////////////////////////////////////////////
//                       SHOOTER                         //
///////////////////////////////////////////////////////////

    feederBagMotor = new WPI_TalonSRX(4);
    lazySusanBagMotor = new WPI_TalonSRX(3);
    shooterWheel1 = new CANSparkMax(13, MotorType.kBrushless);
    shooterWheel2 = new CANSparkMax(14, MotorType.kBrushless);

    rightLimit = new DigitalInput(3);
    leftLimit = new DigitalInput(4);

    shooterEnc = new CANEncoder(shooterWheel1);
    feederEnc = new TalonEncoder(feederBagMotor);
    lazySusanEnc = new TalonEncoder(lazySusanBagMotor);

    shooterClass = new Shooter(lazySusanBagMotor, feederBagMotor, shooterWheel1, shooterWheel2, lazySusanEnc, shooterEnc, rightLimit, leftLimit, limelightClass);
  
///////////////////////////////////////////////////////////
//                    Autonomous                         //
///////////////////////////////////////////////////////////

    routines = new Routines(rightFrontEnc, driveClass, shifterClass, gyro, limelightClass, ballManagementClass, shooterClass);  //CHANGE LATER */
  }

  @Override
  public void robotPeriodic() {
  }

  @Override
  public void autonomousInit() {
  }


  @Override
  public void disabledPeriodic() {
    routines.resetCounters();
    shooterClass.setStop();
    shooterClass.setStoped();
    hangClass.setStop();
    ballManagementClass.setStop();
    ballManagementClass.display();
    shooterClass.displayShooter();
    SmartDashboard.putNumber("Gyro:", gyro.getYaw());
 
    if(leftJoyStick.getRawButton(4)){
    routines.setPortMiddleTrack();
    SmartDashboard.putString("AUTONOMOUS", "PORT MIDDLE (TRACK)");
    }

    else if(leftJoyStick.getRawButton(5)){
      routines.setTrenchRightTwo();
     SmartDashboard.putString("AUTONOMOUS", "TRENCH RIGHT 2");
    }

    else if(leftJoyStick.getRawButton(6)){
      routines.setTrenchRightOne();
      SmartDashboard.putString("AUTONOMOUS", "TRENCH RIGHT 1");
    }

    else if(leftJoyStick.getRawButton(7)){
      routines.setNothing();
      SmartDashboard.putString("AUTONOMOUS", "DO NOTHING");
    }

    else if(leftJoyStick.getRawButton(8)){
      routines.setPortLeft();
      SmartDashboard.putString("AUTONOMOUS", "PORT LEFT");
    }

    else if(leftJoyStick.getRawButton(9)){
      routines.setInitiationLine();
      SmartDashboard.putString("AUTONOMOUS", "CROSS INITIATION");
    }

    else if(leftJoyStick.getRawButton(10)){
      routines.setPortMiddle();
      SmartDashboard.putString("AUTONOMOUS", "PORT MIDDLE (NO TRACK)");
    }

    if(leftJoyStick.getRawAxis(3) > 0){
      SmartDashboard.putString("MODE:", "TESTING");
    }

    else if(leftJoyStick.getRawAxis(3) < 0){
      SmartDashboard.putString("MODE:", "HYPE");
    }
  }  
  
  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    routines.run();
  }

  /**
   * This function is called periodically during operator control.
   */

  @Override
  public void teleopInit() {
    limelightClass.setDriving();
  }


  @Override
  public void teleopPeriodic() {
///////////////////////////////////////////////////////////
//                        TESTING                        //
///////////////////////////////////////////////////////////
//set all to testing
//TELL BALL MANAGEMENT TO MAKE A SET TESTING
  if(leftJoyStick.getRawAxis(3) > 0){
  SmartDashboard.putString("MODE:", "TESTING");
  SmartDashboard.putNumber("Left:", leftFrontEnc.getPosition());
  SmartDashboard.putNumber("Right:", rightFrontEnc.getPosition());

  if(leftJoyStick.getRawButton(1)){                       //RESET ENCODERS
    shooterClass.encReset();
    hangClass.resetElevatorEncoder();
  }

  else if(leftJoyStick.getRawButton(2)){                   //LAZY SUSAN
    shooterClass.setTesting();
    shooterClass.setTest();
    shooterClass.driveTurn(-leftJoyStick.getX());
  }

  else if(leftJoyStick.getRawButton(3)){                  //FEEDER AND SHOOTER
    shooterClass.setTesting();
    shooterClass.setTest();
    shooterClass.driveFeeder(leftJoyStick.getY());
    shooterClass.driveShooter(leftJoyStick.getY());
  }

  else if(leftJoyStick.getRawButton(4)){                  //ELEVATOR
    hangClass.setTesting();
    hangClass.elevatorTest(-leftJoyStick.getY());
  }

  else if(leftJoyStick.getRawButton(5)){                //EXTEND INTAKE
    ballManagementClass.setExtend();
  }

  else if(leftJoyStick.getRawButton(6)){                //RETRACT INTAKE
    ballManagementClass.setRetract();
  }

  else if(leftJoyStick.getRawButton(7)){               //ARM UP
    hangClass.setTesting();
    hangClass.rotateArmUp();
  } 
  
  else if(leftJoyStick.getRawButton(8)){               //ARM DOWN
    hangClass.setTesting();
    hangClass.rotateArmDown();
  }

  else if (leftJoyStick.getRawButton(9)){              //SHIFT SPEED
    shifterClass.shiftSpeed();
  }

  else if (leftJoyStick.getRawButton(10)){              //SHIFT POWER
    shifterClass.shiftPower();
  }

  else if(leftJoyStick.getRawButton(11)){               //OUTPUT
    ballManagementClass.setUnjam();
  }

  else{
  shooterClass.setStop();
  shooterClass.setStoped();
  hangClass.setStop();
  ballManagementClass.setStop();
  driveClass.arcadeRun(0, leftJoyStick.getY());       //ARCADE DRIVE
  }
  }

///////////////////////////////////////////////////////////
//                       HYPE MODE                       //
///////////////////////////////////////////////////////////

else if(leftJoyStick.getRawAxis(3) < 0){
  SmartDashboard.putString("MODE:", "HYPE");

///////////////////////////////////////////////////////////
//                         DRIVE                         //
///////////////////////////////////////////////////////////

    SmartDashboard.putNumber("DRIVE LEFT FRONT VALUE: ", -leftFrontEnc.getPosition());  // DISPLAYS LEFT FRONT ENCODER VALUE
    SmartDashboard.putNumber("DRIVE RIGHT FRONT VALUE: ", rightFrontEnc.getPosition()); // DISPLAYS RIGHT FRONT ENCODER VALUE


    driveClass.setLeftSpeed(leftJoyStick.getY());                                       // MOVES THE LEFT SIDE OF THE DRIVE
    driveClass.setRightSpeed(rightJoyStick.getY());                                     // MOVES THE RIGHT SIDE OF THE DRIVE
    //driveClass.arcadeRun(stickControl.getY(), stickControl.getX());                   //SETS TO ARCADE CONTROL


///////////////////////////////////////////////////////////
//                   TRACKING / DRIVING                  //
///////////////////////////////////////////////////////////


if(mechJoyStick.getRawButtonPressed(5)){
  if(limelightClass.camState == State.TRACKING){
    limelightClass.setDriving();
  }
  else if(limelightClass.camState == State.DRIVING){
    limelightClass.setTracking();
  }
}

///////////////////////////////////////////////////////////
//                       SHIFTER                         //
///////////////////////////////////////////////////////////

    if(leftJoyStick.getRawButton(3)){
      shifterClass.shiftSpeed();
    }
    else if(leftJoyStick.getRawButton(4)){
      shifterClass.shiftPower();
    }

/*
    if(rightJoyStick.getRawButton(3)){
      shifterClass.shiftSpeed();       // SHIFT SPEED
    }

    else if(rightJoyStick.getRawButton(4)){
      shifterClass.shiftPower();       // SHIFT TORQUE
    }
    */



///////////////////////////////////////////////////////////
//                   BALL MANAGEMENT                     //
///////////////////////////////////////////////////////////
    
    if(rightJoyStick.getRawButton(1)){                    //EXTEND INTAKE
      ballManagementClass.setIntakeManagement();
    }
  /*
    else if(leftJoyStick.getRawButton(1)){                //RETRACT INTAKE
      ballManagementClass.setRetract();
    }
    */

    else if(mechJoyStick.getRawButton(1)){
      ballManagementClass.setFeedManagement();
    }

    /*
    else if(mechJoyStick.getPOV() == 180){                //INTAKE
      ballManagementClass.setIntakeManagement();
    }
    /*

    
    
    /*
    else if(rightJoyStick.getPOV() == 180){
      ballManagementClass.setRetract();
    }
    */
    else if(rightJoyStick.getPOV() == 0){
      ballManagementClass.setExtend();
    }

    else if(rightJoyStick.getPOV() == 180){
      ballManagementClass.setRetract();
    }
    
    else if(mechJoyStick.getPOV() == 0){
      ballManagementClass.setLoadOverride();              //TEMPORARY
    }
    
    else if(mechJoyStick.getRawButton(12)){               //UNJAM
      ballManagementClass.setOutPut();
    }

    else{
      ballManagementClass.setStop();
    }    

///////////////////////////////////////////////////////////
//                     WINCH & HANG                      //
///////////////////////////////////////////////////////////
    
    if(mechJoyStick.getRawButton(10)){                           //WINCH UP   (HOLD)
      hangClass.setWinchUp();
    }

    else if(mechJoyStick.getRawButton(9)){                      //SEQUENCE UP
      hangClass.setSequenceUp();
    }

    else if(mechJoyStick.getRawButton(11)){                     //SEQUENCE DOWN
      hangClass.setSequenceDown();
    }

    else{
      hangClass.setStop();
    }
///////////////////////////////////////////////////////////
//                      SHOOTER                          //
///////////////////////////////////////////////////////////

    if(mechJoyStick.getRawButton(2)){
      shooterClass.setShoot();
    }

    else if(mechJoyStick.getRawButton(4)){
      shooterClass.setPass();
    }

    else if(mechJoyStick.getRawButton(3)){
      shooterClass.setFarShot();
    }
    else{
      shooterClass.setStop();
    }


///////////////////////////////////////////////////////////
//                      LAZY SUSAN                       //
///////////////////////////////////////////////////////////


    if(mechJoyStick.getRawButton(7)){                        //FIND LEFT   ADD CHECK FOR INTAKE, MOVE IT UP FIRST
      if(intakePiston.get() == Value.kReverse){
      ballManagementClass.setExtend();                    //bring down
      }
      shooterClass.setTrackLeft();                           //track left
    }

    else if(mechJoyStick.getRawButton(8)){                  //FIND RIGHT   ADD CHECK FOR INTAKE, MOVE IT UP FIRST
      if(intakePiston.get() == Value.kReverse){
      ballManagementClass.setExtend();                    //bring down   
      }                            
      shooterClass.setTrackRight();
    }

    else if(mechJoyStick.getRawButton(6)){                    //RESET SUZAN AMD BRING INTAKE UP IF IT'S DOWN
      if(intakePiston.get() == Value.kForward){
      ballManagementClass.setRetract();                     //bring up
      }
      shooterClass.setResetRight();                           //RESET SUZAN
    }
    
    else{
      shooterClass.setStoped();
    }

    driveClass.tankRun();
    limelightClass.runCode();
  }

shooterClass.run();
hangClass.run();
ballManagementClass.run();
}

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
