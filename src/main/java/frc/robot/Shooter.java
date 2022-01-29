package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SpeedController;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.Encoder;
import com.revrobotics.CANEncoder;

import edu.wpi.first.wpilibj.DigitalInput;

public class Shooter{

    private Limelight limelight;

    private SpeedController turnMotor;//bag motor
    private SpeedController feederMotor;//bag motor
    private SpeedController shooterMotor1;//neo motor
    private SpeedController shooterMotor2;//neo motor

    private TalonEncoder turnEnc;//talon srx encoder
    private CANEncoder shooterEnc;//talon srx encoder

    private DigitalInput rightLimit;//hall effect
    private DigitalInput leftLimit;// hall effect

    private double turnLeftSpeed = 0.35;
    private double turnRightSpeed = -0.35;
    private double turnLeftSpeedSlow = 0.15;
    private double turnRightSpeedSlow = -0.15;
    private double turnRightSpeedSlowHome = -0.17;
    private double feederSpeed = 0.75; //0.7
    private double feederFarSpeed = 0.79; //0.75
    private double shooterSpeed = 0.77;//shots from 16 - 26 feet, infront to back of trench was .74
    private double passSpeed = 0.4;//used to shoot the ball low to teamates or across the field without it going out of bounds
    private double shooterFarSpeed = 0.83;//shoots from 34 feet, behind control panel .9, .8

    private double shooterCoastSpeed = 0.3;

    private int leftPos = 800;//soft limit for suzan left turn
    private int rightPos = 400;//soft limit for suzan right turn
    private int inRange = 30;//range for the suzan facing the target
    private int outRange = 200;//range for suzan closing in on target

    private double ratio = 69;//11 ticks per degree, 1913 counts for 20 degrees 

    /////////////////////////////////////////////
    //                                         //
    //               CONSTRUCTOR               //
    //                                         //
    /////////////////////////////////////////////

    public Shooter(SpeedController newTurnMotor, SpeedController newFeederMotor, SpeedController newShooterMotor1, SpeedController newShooterMotor2, TalonEncoder newTurnEnc, CANEncoder newShooterEnc, DigitalInput newRightLimit, DigitalInput newLeftLimit, Limelight newLimelight){
        turnMotor = newTurnMotor;
        feederMotor = newFeederMotor;
        shooterMotor1 = newShooterMotor1;
        shooterMotor2 = newShooterMotor2;

        turnEnc = newTurnEnc; 
        shooterEnc = newShooterEnc;

        rightLimit = newRightLimit;
        leftLimit = newLeftLimit;

        limelight = newLimelight;
    }
    
    /////////////////////////////////////////////
    //                                         //
    //               ENUMERATION               //
    //                                         //
    /////////////////////////////////////////////

    public enum shootState{
        STOP, SHOOT, PASS, FARSHOT, TESTING
    }

    shootState Mode = shootState.STOP;

    public void setStop(){//state that stops the shooter and feeder
        Mode = shootState.STOP;
    }

    public void setShoot(){//state that turns feeder and shooter wheels to score 22 feet away
        Mode = shootState.SHOOT;
    }

    public void setPass(){//state that turns feeder and shooter wheels to pass the ball low
        Mode = shootState.PASS;
    }

    public void setFarShot(){//state that turns feeder and shooter wheels to score 34 feet away
        Mode = shootState.FARSHOT;
    }

    public void setTesting(){//state used to test the motors manually for the shooter
        Mode = shootState.TESTING;
    }

    public enum suzanState{
        STOPPED, TRACKLEFT, TRACKRIGHT, RESETRIGHT, TEST
    }

    suzanState Form = suzanState.STOPPED;

    public void setStoped(){//state that stops just the suzan motor
        Form = suzanState.STOPPED;
    }

    public void setTrackLeft(){//state that turn suzan left until it finds target
        Form = suzanState.TRACKLEFT;
    }

    public void setTrackRight(){//state that turn suzan right until it finds target
        Form = suzanState.TRACKRIGHT;
    }

    public void setResetRight(){//state that resets the suzan to encoder position 0
        Form = suzanState.RESETRIGHT;
    }

    public void setTest(){
        Form = suzanState.TEST;
    }

    /////////////////////////////////////////////
    //                                         //
    //                   CHECKS                //
    //                                         //
    /////////////////////////////////////////////

    private boolean beforeLeftLimit(){//checks if the turn motor is before left boundray
        return -turnEnc.get() >= leftPos;
    }

    private boolean beforeRightLimit(){//checks if the turn motor is before right boundary
        return -turnEnc.get() >= rightPos; 
    }

    private boolean notAtLeftLimit(){//checks if the turn motor is touching left limit switch
        return leftLimit.get();
    }

    private boolean notAtRightLimit(){//checks if the turn motor is touching right limit swtich
        return rightLimit.get();
    }

    private double getCurrentOffset(){//not a check, converts degrees to encoder counts
        return limelight.getHorizontalOffSet() * ratio;
    }

    private boolean beforeTarget(){
        return Math.abs(getCurrentOffset()) < outRange;
    }

    public boolean checkTarget(){//checks if suzan lined up with upper port for shooting
        return Math.abs(getCurrentOffset()) < inRange;
    }

    private boolean checkTurnLeft(){//checks if suzan right of target
        return getCurrentOffset() < 0;
    }

    private boolean checkTurnRight(){//checks if suzan is left of target
        return getCurrentOffset() > 0;
    }

    /////////////////////////////////////////////
    //                                         //
    //                  METHODS                //
    //                                         //
    /////////////////////////////////////////////

    public void driveTurn(double speed){//drives turn motor manually
        turnMotor.set(speed);
    }

    public void driveFeeder(double speed){//drives feeder motor manually
        feederMotor.set(speed);
    }

    public void driveShooter(double speed){//drives shooter motors manually
        shooterMotor1.set(-speed);
        shooterMotor2.set(speed);
    }

    public void feedForward(){
        feederMotor.set(feederSpeed);
    }

    public void feedBackward(){
        feederMotor.set(-feederSpeed);
    }

    public void testing(){

    }

    public void encReset(){//resets all encoders
        turnEnc.reset(); 
        shooterEnc.setPosition(0); 
    }

    private void stop(){//stops shooter and feeder motors
        feederMotor.set(0);
        shooterMotor1.set(0);
        shooterMotor2.set(0);
    }

    private void suzanStop(){//stops lazy suzan motor
        turnMotor.set(0);
    }

    private void turnLeft(){//moves lazy susan left
        turnMotor.set(turnLeftSpeed);
    }

    private void turnRight(){//moves lazy susan right
        turnMotor.set(turnRightSpeed);
    }

    private void turnLeftSlow(){//moves lazy susan left slowly
        turnMotor.set(turnLeftSpeedSlow);
    }

    private void turnRightSlow(){//moves lazy susan right slowly
        turnMotor.set(turnRightSpeedSlow);
    }

    private void turnRightSlowHome(){//moves lazy susan right slowly
        turnMotor.set(turnRightSpeedSlowHome);
    }

    private void track(){//tracks the vison tape on the upper port and turns the lazy suzan to center with it

        if(limelight.targetSeen()){

            if(checkTarget()){//if suzan is alligned, don't move
                turnMotor.set(0);
            }

            else if(checkTurnLeft()){
                if(notAtLeftLimit()){//if limit switch is not being touched
                    if(beforeLeftLimit() || beforeTarget()){//if before left positiion or almost in target range, slow down
                        turnLeftSlow();
                    }

                    else{
                      turnLeft();
                    }
                }
                else{
                    turnMotor.set(0);
                }
            }

            else if(checkTurnRight()){
                if(notAtRightLimit()){//if limit swtich is not being touched
                    if(beforeRightLimit() || beforeTarget()){//if before right position or almost in target range, slow down
                        turnRightSlow();
                    }

                    else{
                        turnRight();
                    }
                }
                else{
                    turnMotor.set(0);
                }
            }

            else{
                turnMotor.set(0);//stop suzan from moving
            }
        }
    }

    private void trackTargetLeft(){//turns left until it finds the target or hits limit
        if(limelight.targetSeen()){
            track();
        }

        else{
            if(notAtLeftLimit()){//if limit switch is not being touched
                if(beforeLeftLimit()){//if before left positiion, slow down
                    turnLeftSlow();
                }
    
                else{//otherwise, turn left
                    turnLeft();
                }
            }
                
            else{
                turnMotor.set(0);
            }
        }
    }

    private void trackTargetRight(){//turns right until it finds the target or hits limit
        if(limelight.targetSeen()){
            track();
        }

        else{
            if(notAtRightLimit()){//if limit switch is not being touched
                if(beforeRightLimit()){//if before left positiion, stop
                    turnRightSlow();
                }
    
                else{//otherwise, turn right
                    turnRight();
                }
            }
                
            else{
                turnMotor.set(0);
            }
        }
    }

    private void suzanRight(){ 
        if(notAtRightLimit()){//if limit swtich is not being touched
            if(beforeRightLimit()){//if before or almost in right position, slow down
                turnRightSlowHome();
            }

            else{
                turnRight();
            }
        }
        else{
            turnMotor.set(0);
        }
    }

    private void shoot(){//move the shooter and feeder wheel to shoot from 16 - 26 feet away
        shooterMotor1.set(-shooterSpeed);
        shooterMotor2.set(shooterSpeed);
        feederMotor.set(feederSpeed);
    }

    private void pass(){//move the shooter and feeder wheel to pass the ball across the field without it going out of bounds
        shooterMotor1.set(-passSpeed);
        shooterMotor2.set(passSpeed);
        feederMotor.set(feederSpeed);
    }

    private void farShot(){//moves shooter and feeder wheels to shoot from 34 feet away
        shooterMotor1.set(-shooterFarSpeed);
        shooterMotor2.set(shooterFarSpeed);
        feederMotor.set(feederFarSpeed);
    }

    public void displayShooter(){
        SmartDashboard.putNumber("Susan Enc: ", -turnEnc.get());
        SmartDashboard.putNumber("Shooter Enc: ", shooterEnc.getPosition());
        SmartDashboard.putBoolean("Susan Left Enc Limit:", beforeLeftLimit());
        SmartDashboard.putBoolean("Susan Right Enc Limit:", beforeRightLimit());
        SmartDashboard.putBoolean("Susan Left Halo Limit:", notAtLeftLimit());
        SmartDashboard.putBoolean("Susan Right Halo Limit:", notAtRightLimit());
        SmartDashboard.putBoolean("Susan Inner Target:", checkTarget());
        SmartDashboard.putBoolean("Susan Outer Target:", beforeTarget());
        SmartDashboard.putBoolean("Susan Check Left:", checkTurnLeft());
        SmartDashboard.putBoolean("Susan Check Right:", checkTurnRight());
        SmartDashboard.putString("Shooter State:", Mode.toString());
        SmartDashboard.putString("Suzan Form:", Form.toString());
        SmartDashboard.putNumber("Suzan Speed:", turnMotor.get());
    }

    /////////////////////////////////////////////
    //                                         //
    //                  RUN                    //                              
    //                                         //
    /////////////////////////////////////////////

    public void run(){
        switch(Mode){

            case STOP:
            stop();
            break;

            case SHOOT:
            shoot();
            break;

            case PASS:
            pass();
            break;

            case FARSHOT:
            farShot();
            break;

            case TESTING:
            testing();
            break;
        }

        switch(Form){

            case STOPPED:
            suzanStop();
            break;

            case TRACKLEFT:
            trackTargetLeft();
            break;

            case TRACKRIGHT:
            trackTargetRight();
            break;

            case RESETRIGHT:
            suzanRight();
            break;

            case TEST:
            testing();
            break;
        }
        displayShooter();
    }
}