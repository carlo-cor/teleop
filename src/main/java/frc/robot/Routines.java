package frc.robot;
                                                    //TEST AT COMP FIRST, MAYBE ADD SHORT INTAKE DELAY IF IT STILL PUSHES FIRST BALL
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
                                                        //SET BALLS IN SYSTEM IN EACH
public class Routines{
    //GYRO DECS
    private AHRS navx;
    private float yaw; 
    private final float ROTATION_SPEED = -0.20f; //SPEED FOR STRAIGHTENING

    private CANEncoder rightEnc;
    private Drive drives;
    private Shifter Shifter;

    //SHOOTER
    private Shooter shooter;
    private Timer shootTime;

    //LIMELIGHT
    private Limelight limelight;

    //MANAGEMENT SYSTEM
    private BallManagement manager;
    private Timer intakePause;

    //SPEEDS
    //45.118565
    //45.332848
    //45.070942
    //CONVERSIONS
    private final double COUNTS_PER_FOOT = 11.2935296;           //CHANGE with new base

    private final double intakeDelay = 0;
    //SHOOTING TIMES
    private final double delay = 0.5;                                  //SHOOTER DELAY
    private final double THREE_BALL_SHOOT = delay + 20;              //CHANGE
    private final double FOUR_BALL_SHOOT = delay + 20;               //CHANGE
    private final double FIVE_BALL_SHOOT = delay + 20;               //CHANGE

    //DISTANCES FOR INITIATION LINE
    private final double CROSS_INITIATION = 3;

    //DISTANCES FOR PORT MIDDLE AUTO
    private final double MIDDLE_SHOOT_SETUP = 10.41;

    //DISTANCES FOR PORT LEFT AUTO 
    private final double LEFT_SHOOT_SETUP = 6;

    //DISTANCES FOR TRENCH RIGHT AUTOS
    private final double TRENCH_PICKUP = 10.0;
    private final double TRENCH_SHOOT_SETUP = 6.5;

    //TURN VALUES FOR PORT LEFT AUTO
    private final float RIGHT_TOWARDS_PORT = 12.889f;

    //TURN VALUES FOR PORT MIDDLE AUTO
    private final float RIGHT_OF_PORT = 2f;

    //AUTO COUNTERS
    int portMiddleCounter = 0;
    int portLeftCounter = 0;
    int trenchRightTwoCounter = 0;
    int trenchRightOneCounter = 0;
    int portMiddleTrackCounter = 0;
    int initiationLineCounter = 0;
    
    //CLASS DECLARATIONS
    private auton runState;

    //CONSTRUCTOR
    public Routines(CANEncoder rightEncoder, Drive drive, Shifter shifter, AHRS gyro, Limelight newLimelight, BallManagement newManager, Shooter newShooter){        
        rightEnc = rightEncoder;
        drives = drive;
        Shifter = shifter;
        navx = gyro;
        limelight = newLimelight;
        shootTime = new Timer();
        manager = newManager;
        shooter = newShooter;
        intakePause = new Timer();
        setNothing();
    }

    //ENUMERATIONS
    public enum auton{                                                          //ENUMERATIONOS
        NOTHING, INITIATIONLINE, PORTMIDDLETRACK, PORTMIDDLE, PORTLEFT, TRENCHRIGHTTWO, TRENCHRIGHTONE
    }

    public void setNothing(){                                          //SET STATE TO NOTHING
        runState = auton.NOTHING;
    }

    public void setInitiationLine(){                                //SET STATE TO INITIATION LINE
        runState = auton.INITIATIONLINE;
    }

    public void setPortMiddleTrack(){                                       //SET STATE TO PORT MIDDLE
        runState = auton.PORTMIDDLETRACK;
    }

    public void setPortMiddle(){
        runState = auton.PORTMIDDLE;
    }

    public void setPortLeft(){                                         //SET STATE TO PORT LEFT
        runState = auton.PORTLEFT;
    }

    public void setTrenchRightTwo(){                                  //SET STATE TO TRENCH RIGHT INTAKE
        runState = auton.TRENCHRIGHTTWO;         //2 BALLS
    }

    public void setTrenchRightOne(){                                      //SET STATE TO TRENCH RIGHT SHOOT
        runState = auton.TRENCHRIGHTONE;          //1 BALL
    }

    private void startShooterTime(){                           //START SHOOT TIMER
        shootTime.reset();
        shootTime.start();   
    }

    private void timedShoot(double delay, double time){         //TIMED SHOOT FOR 3 BALLS
        if(shootTime.get() > time){
            shooter.setStop();
        }
        else{
            shooter.setShoot();
            if(shootTime.get() > delay){
                manager.setThreeBallFeed();
            }
        }
    }

    private void timedShootPickup(double delay, double time){   //TIMED SHOOT FOR 4 / 5 BALLS
        if(shootTime.get() > time){
            shooter.setStop();
        }
        else{
            shooter.setShoot();
            if(shootTime.get() > delay){
                manager.setFeedManagement();
            }
        }
    }



    private double feetToCounts(double feet){                       //FEET TO COUNTS
        return(feet * COUNTS_PER_FOOT);
    }

    public void resetCounters(){                                    //RESET SWITCH COUNTERS
        portMiddleCounter = 0;
        portLeftCounter = 0;
        trenchRightTwoCounter = 0;
        trenchRightOneCounter = 0;
        portMiddleTrackCounter = 0;
        initiationLineCounter = 0;
    }

    private void resetEncoders(){                                    //RESET DRIVE ENCODER
        rightEnc.setPosition(0);
        shooter.encReset();
    }

    private void doNothing(){}                                       //DEFAULT, DO NOTHING AUTONOMOUS

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    private void initiationLine(){                                                   //CROSS INITIATION LINE
    yaw = navx.getYaw();
    float rotation = yaw > 0.0f ? ROTATION_SPEED : -ROTATION_SPEED;                 // STRAIGHTENS ROBOT 

    switch(initiationLineCounter){
    case 0:
    Shifter.shiftPower();
    resetEncoders();
    navx.reset();
    initiationLineCounter++;
    break;

    case 1:
    if(Math.abs(rightEnc.getPosition()) >= feetToCounts(CROSS_INITIATION)){         //MOVE BACK
        drives.arcadeRun(0, 0);
    }
    else{
        drives.arcadeRun(rotation, 0.8);
    }
    break;

   }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    private void portLeftShot(){                                                         //LINED UP WITH LOADING BAY
    yaw = navx.getYaw();
    float rotation = yaw > 0.0f ? ROTATION_SPEED : -ROTATION_SPEED;                     // STRAIGHTENS ROBOT 
    
    switch(portLeftCounter){
        case 0:
        Shifter.shiftPower();                                                           //RESET ENCODERS
        resetEncoders();                                                                //RESET NAVX
        navx.reset();
        limelight.setTracking();
        portLeftCounter++;
        break;

        case 1:
        if(Math.abs(rightEnc.getPosition()) >= feetToCounts(LEFT_SHOOT_SETUP * 3/4)){    //MOVE BACK RAMP FAST
            drives.arcadeRun(0,0);
            portLeftCounter++;
        }

        else{
            drives.arcadeRun(rotation, 0.7);
        }
        break;    

        case 2:
        shooter.setShoot();                                                             //REV SHOOTER
        if(Math.abs(rightEnc.getPosition()) >= feetToCounts(LEFT_SHOOT_SETUP)){         //MOVE BACK RAMP SLOW       
            drives.arcadeRun(0,0);
            portLeftCounter++;
        }

        else{
            drives.arcadeRun(rotation, 0.5);
        }
        break;  

        case 3:
        if(Math.abs(navx.getYaw()) >= RIGHT_TOWARDS_PORT){                              //TURN RIGHT OF PORT
            drives.arcadeRun(0,0);
            portLeftCounter++;
        }
        else{
            drives.arcadeRun(0.6, 0);
        }
        break;

        case 4:                                                                         //TRACK PORT
        manager.setExtend();
        if(limelight.targetSeen()){
            shooter.setTrackLeft();
            portLeftCounter++;
        }
        else{
            shooter.setStop();
        }
        break;

        case 5:
            startShooterTime();
            portLeftCounter++;
        break;

        case 6:                                                                     //SHOOT 3 CELLS INTO POWER PORT
            timedShoot(delay, THREE_BALL_SHOOT);
        break;
    }
    }

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

private void portMiddleTrackShot(){                                                      //LINED UP WITH PORT
    yaw = navx.getYaw();
    float rotation = yaw > 0.0f ? ROTATION_SPEED : -ROTATION_SPEED;                 //STRAIGHTEN ROBOT

    switch(portMiddleTrackCounter){                                                     //SHIFT TO POWER
    case 0:                                                                         //RESET ENCODERS
        Shifter.shiftPower();
        resetEncoders();
        navx.reset();
        limelight.setTracking();
        portMiddleTrackCounter++;
        break;

    case 1:                                                                                      //MOVE BACK RAMP FAST
        if(Math.abs(rightEnc.getPosition()) >= feetToCounts(MIDDLE_SHOOT_SETUP * 3/4)){
            drives.arcadeRun(0, 0);
            portMiddleTrackCounter++;
        }
        else{
            drives.arcadeRun(rotation, 0.7);
        }
        break;
    case 2:              
        shooter.setShoot();                                                                       //MOVE BACK RAMP SLOW
        if(Math.abs(rightEnc.getPosition()) >= feetToCounts(MIDDLE_SHOOT_SETUP)){ 
            drives.arcadeRun(0, 0);
            portMiddleTrackCounter++;
}
        else{
            drives.arcadeRun(rotation, 0.5);
        }
        break;
      
    case 3:
        if(Math.abs(navx.getYaw()) >= RIGHT_OF_PORT){                                                //TURN RIGHT OF PORT
            drives.setLeftSpeed(0);
            drives.setRightSpeed(0);
            portMiddleTrackCounter++;
        }
        else{
            drives.arcadeRun(0.3, 0);
        }
        break;

    case 4:
        manager.setExtend();                                                                            //TRACK POWER PORT
         if(limelight.targetSeen()){
            shooter.setTrackLeft();
            portMiddleTrackCounter++;
        }
        else{
            shooter.setStop();
        }
        break;

    case 5:
        startShooterTime();
        portMiddleTrackCounter++;
        break;

    case 6:                                                                                          
        manager.setFeedManagement();                                                                    //SHOOT 3 BALLS
        timedShoot(delay, THREE_BALL_SHOOT);
        break;
        
    }
  }

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    private void portMiddleShot(){
        yaw = navx.getYaw();
    float rotation = yaw > 0.0f ? ROTATION_SPEED : -ROTATION_SPEED;                 //STRAIGHTEN ROBOT

    switch(portMiddleCounter){                                                     //SHIFT TO POWER
    case 0:                                                                         //RESET ENCODERS
        Shifter.shiftPower();
        resetEncoders();
        navx.reset();
        portMiddleCounter++;
        break;

    case 1:                                                                                      //MOVE BACK RAMP FAST
        if(Math.abs(rightEnc.getPosition()) >= feetToCounts(MIDDLE_SHOOT_SETUP * 3/4)){
            drives.arcadeRun(0, 0);
            portMiddleCounter++;
        }
        else{
            drives.arcadeRun(rotation, 0.7);
        }
        break;
    case 2:              
        shooter.setShoot();                                                                       //MOVE BACK RAMP SLOW
        if(Math.abs(rightEnc.getPosition()) >= feetToCounts(MIDDLE_SHOOT_SETUP)){ 
            drives.arcadeRun(0, 0);
            portMiddleCounter++;
}
        else{
            drives.arcadeRun(rotation, 0.5);
        }
        break;

    case 3:
        manager.setExtend();  
        portMiddleCounter++;                                                                          //PUT INTAKE DOWN
        break;

    case 4:
        startShooterTime();
        portMiddleCounter++;
        break;

    case 5:                                                                                          
        manager.setFeedManagement();                                                                    //SHOOT 3 BALLS
        timedShoot(delay, THREE_BALL_SHOOT);
        break;
        
    }
    }
  


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////       

    private void portTrenchRightTwo(){                                                  //LINED UP WITH TRENCH  (2 BALLS)
        yaw = navx.getYaw();
        float rotation = yaw > 0.0f ? ROTATION_SPEED : -ROTATION_SPEED;                 //STRAIGHTENS ROBOT

        switch(trenchRightTwoCounter){
            case 0:                                                                     //RESET
            Shifter.shiftPower();                                                       //SHIFT POWER
            resetEncoders();    
            navx.reset();
            limelight.setTracking();                                                    //SET TO TRACKING
            trenchRightTwoCounter++;
            break;

            case 1:                                                                     //EXTEND INTAKE
            manager.setExtend();
            trenchRightTwoCounter++;
            break;

            case 2:                                                                     //INTAKE DELAY (0 NOW)
            intakePause.reset();
            intakePause.start();
            trenchRightTwoCounter++;
            break;

            case 3:
            if(intakePause.get() > intakeDelay){
                trenchRightTwoCounter++;
            }
            break;

            case 4: 
            if(Math.abs(rightEnc.getPosition()) >= feetToCounts(TRENCH_PICKUP * 6/11)){    //MOVE BACK AND INTAKE 2 CELLS RAMP FAST
                drives.arcadeRun(0, 0); // dont need
                trenchRightTwoCounter++;
            }
            else{                
                manager.setIntakeManagement();
                drives.arcadeRun(rotation, 0.7);                

            }
            break;

            case 5:
            shooter.setShoot();                                                           //REV SHOOTER
            if(Math.abs(rightEnc.getPosition()) >= feetToCounts(TRENCH_PICKUP)){          //MOVE BACK AND INTAKE 2 CELLS RAMP SLOW
                drives.arcadeRun(0, 0);
                trenchRightTwoCounter++;
            }
            else{
                manager.setIntakeManagement();
                drives.arcadeRun(rotation, 0.5);
            }
            break;

            case 6: 
                shooter.setTrackLeft();                                                  //TRACK TARGER
                trenchRightTwoCounter++;
            break;

            case 7:
            startShooterTime();
            trenchRightTwoCounter++;
            break;
            
            case 8:                                                                     //SHOOT 5 CELLS INTO POWER PORT
                timedShootPickup(delay, FIVE_BALL_SHOOT);
            break;
        }
    }

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    private void portTrenchRightOne(){                                        //LINED UP WITH ALLIANCE TRENCH     (1 BALL)
        yaw = navx.getYaw();
        float rotation = yaw > 0.0f ? ROTATION_SPEED : -ROTATION_SPEED;                  //STRAIGHTENS ROBOT

        switch(trenchRightOneCounter){
            case 0:                                                                     //RESET
            Shifter.shiftPower();        
            resetEncoders();    
            navx.reset();
            limelight.setTracking();
            trenchRightOneCounter++;
            break;

            case 1:                                                                         //INTAKE 1 BALL
            manager.setExtend();
            trenchRightOneCounter++;
            break;

            case 2:
            intakePause.reset();
            intakePause.start();
            trenchRightOneCounter++;
            break;

            case 3:
            if(intakePause.get() > intakeDelay){
                trenchRightOneCounter++;
            }
            break;

            case 4:
            shooter.setShoot();
            if(Math.abs(rightEnc.getPosition()) >= feetToCounts(TRENCH_SHOOT_SETUP * 3/4)){
                drives.arcadeRun(0,0);
                trenchRightOneCounter++;
            }
            else{
                manager.setIntakeManagement();
                drives.arcadeRun(rotation, 0.7);

            }
            break;

            case 5:                                                                         //INTAKE 1 BALL
            if(Math.abs(rightEnc.getPosition()) >= feetToCounts(TRENCH_SHOOT_SETUP)){
                drives.arcadeRun(0, 0);
                trenchRightOneCounter++;
            }
            else{
                manager.setIntakeManagement();
                drives.arcadeRun(rotation, 0.5);
            }
            break;

            case 6:                                                     //FIND POWER PORT
                shooter.setTrackLeft();
                trenchRightOneCounter++;
            break;

            case 7:
                startShooterTime();
                trenchRightOneCounter++;
                break;

            case 8:                                                                     //SHOOT 3 CELLS INTO POWER PORT
                timedShootPickup(delay, FOUR_BALL_SHOOT);
            break;
        }
    }        

    public void run(){
        SmartDashboard.putNumber("Right Encoder:", Math.abs(rightEnc.getPosition()));
        SmartDashboard.putString("State:", runState.name());
        SmartDashboard.putNumber("Yaw:", yaw);
        SmartDashboard.putNumber("Port Middle Case:", portMiddleCounter);
        SmartDashboard.putNumber("Port Middle Track Case:", portMiddleTrackCounter);
        SmartDashboard.putNumber("Initiation Line Case", initiationLineCounter);
        SmartDashboard.putNumber("Trench Right Two Case:", trenchRightTwoCounter);
        SmartDashboard.putNumber("Trench Right One Case:", trenchRightOneCounter);
        switch(runState){

            case NOTHING:
            doNothing();
            break;

            case INITIATIONLINE:
            initiationLine();
            break;

            case PORTMIDDLETRACK:
            portMiddleTrackShot();
            break;

            case PORTMIDDLE:
            portMiddleShot();
            break;

            case PORTLEFT:
            portLeftShot();
            break;
            
            case TRENCHRIGHTTWO:
            portTrenchRightTwo();
            break;

            case TRENCHRIGHTONE:
            portTrenchRightOne();
            break;

            default:
            doNothing();
            break;

        }
        shooter.run();
        manager.run();
        limelight.runCode();
    }
}




