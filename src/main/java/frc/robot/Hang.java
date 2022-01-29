package frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Hang {
    // Elevator needs to come down when the arm comes down
    private SpeedController elevatorLift;
    private SpeedController winch;
    private DoubleSolenoid armSolenoid;
    private TalonEncoder elevatorEncoder;
    private TalonEncoder winchEncoder;
    private Timer timer;


    private double elevatorUpSpeed = 0.9; // temp value
    private double elevatorDownSpeed = -0.9; // temp value
    private double winchSpeed = -1.0; 
 

    private double elevatorUpValue = 10400; 
    private double elevatorDownValue = 700; 
    private double winchDeployValue = -2000; 

    private int hangUpCounter;
    private int pullUpCounter;



    public Hang(SpeedController newElevatorLift, SpeedController newWinch, DoubleSolenoid newArmSolenoid, TalonEncoder newElevatorEncoder, TalonEncoder newWinchEncoder) {
        elevatorLift = newElevatorLift;
        winch = newWinch; 
        armSolenoid = newArmSolenoid;
        elevatorEncoder = newElevatorEncoder;
        winchEncoder = newWinchEncoder;
        hangUpCounter = 0;
        pullUpCounter = 0;
        timer = new Timer();
    }



    public void elevatorTest(double speedXValue) {
        elevatorLift.set(speedXValue);
            
        }
    
    public void winchTest(double speedXValue) {
        winch.set(speedXValue);
        }


    /*////////////////////////////////////////////////////////////////////////////////
	/ 																				 /
	/								ENUMERATIONS								     /
	/																				 /
    */////////////////////////////////////////////////////////////////////////////////
    


    private enum State { // the states of the mechanism
    TESTING, SEQUENCEUP, SEQUENCEDOWN, STOP, WINCHUP
    }



    /*////////////////////////////////////////////////////////////////////////////////
	/ 																				 /
	/								    STATES								         /
	/																				 /
    */////////////////////////////////////////////////////////////////////////////////
    


    public State hangState = State.TESTING;

    public void setTesting() { // Sets to nothing
        hangState = State.TESTING;
    }

    public void setSequenceUp() { // Sets the sequence for the hanging mechanism
        hangUpCounter = 0; //resets the counter
        hangState = State.SEQUENCEUP;
    }

    public void setSequenceDown() { // Sets the sequence to bring the robot up
        pullUpCounter = 0;  //resets the counter
        hangState = State.SEQUENCEDOWN;
    }

    public void setStop() { //sets the state to stop
        if(hangState == State.WINCHUP || hangState == State.TESTING) {
        hangState = State.STOP;
        }
    }

    public void setWinchUp() { //sets the state to winchup
        hangState = State.WINCHUP;
    }

    
    



    /*////////////////////////////////////////////////////////////////////////////////
	/ 																				 /
	/								   BOOLEANS								         /
	/																				 /
    */////////////////////////////////////////////////////////////////////////////////
    


    private boolean elevatorIsUp() { // Boolean check to see if the encoder value is > than the up value
        return elevatorEncoder.get() >= elevatorUpValue;
    }

    private boolean elevatorIsDown() { // Booleans check to see if the encoder value is < than the down value
        return elevatorEncoder.get() <= elevatorDownValue;
    }

    private boolean armSolenoidIsUp() { // Booleans to see if the arm solenoid is up
        return timer.get() > 3; //might change
    }

    private boolean armSolenoidIsDown() { // Booleans to see if the arm solenoid is down
        return timer.get() > 3; //might change
    }

    private boolean winchIsDeployed() { // Checks the encoder to see if its up
        return winchEncoder.get() <= winchDeployValue; 
    }



    /*////////////////////////////////////////////////////////////////////////////////
	/ 																				 /
	/								     METHODS							         /
	/																				 /
    */////////////////////////////////////////////////////////////////////////////////
    


    public void resetElevatorEncoder() { // Resets the encoder
        elevatorEncoder.reset();
        winchEncoder.reset();
    }

    private void liftElevator() { // Lifts the elevator up
        if (elevatorIsUp()) {
            elevatorLift.set(0);
            setTesting();
        } else {
            elevatorLift.set(elevatorUpSpeed);
        }
    }

    private void dropElevator() { // Drops the elevator
        if (elevatorIsDown()) {
            elevatorLift.set(0);
            setTesting();
        } else {
            elevatorLift.set(elevatorDownSpeed);
        }
    }

    public void rotateArmUp() { // Rotates the arm so that its upward
        armSolenoid.set(Value.kForward);
    }

    public void rotateArmDown() { // Rotates the arm down
        armSolenoid.set(Value.kReverse);
    }

    private void winchUp() { //Sets the winch speed
        winch.set(winchSpeed);
    }

    private void winchStop() { //stops the winch and the elevator
        winch.set(0.0);
        elevatorLift.set(0.0);
    }

    private void winchDeploy() { //The method to bring the winch up
        if(winchIsDeployed()){
            winchStop();
            setTesting();
        }
        else{
            winchUp();
        }
    }



    /*////////////////////////////////////////////////////////////////////////////////
	/ 																				 /
	/								  SEQUENCES								         /
    /																				 /
    */////////////////////////////////////////////////////////////////////////////////
    


    private void sequenceUp() { // The sequence to bring out the hanging mechanism
        switch (hangUpCounter) { // the counter
        case 0:
            timer.start();
            hangUpCounter++;
             break;
        case 1:
            if (armSolenoidIsUp()) { // if the arm is up go to the next case
                timer.stop();
                hangUpCounter++;
                
            } else { // else rotate the arm up
                rotateArmUp();
            }
            break;
        case 2:
            if (elevatorIsUp()) { // if the elevator is up, break
                elevatorLift.set(0);
                hangUpCounter++;
            } else { // else lift the elevator up and then break
                liftElevator();
            }
            break;
        case 3:
            if(winchIsDeployed()) {
                winch.set(0);
                hangUpCounter++;
            } else {
                winchDeploy();
            } 
            break;  
        case 4:
            setTesting();
            timer.reset();
            break;
        }
    }

    private void sequenceDown() { // Retracts all the mechanisms to hang
        switch (pullUpCounter) {
        case 0:
            if (elevatorIsDown()) { // if the elevator is down go to the next statement
                elevatorLift.set(0);
                pullUpCounter++;
            } else { // else drop the elevator
                dropElevator();
            }

            break;
        case 1:
            timer.start();
            pullUpCounter++;
        break;
        case 2:
            if (armSolenoidIsDown()) { // if the arm is down go to the next statement
                pullUpCounter++;
                timer.stop();
            } else { // else bring the arm down
                rotateArmDown();
            }
            break;
        case 3:
            setTesting();
            timer.reset();
            break;
            
        }
    }



    /*////////////////////////////////////////////////////////////////////////////////
	/ 																				 /
	/								  DASHBOARD								         /
	/																				 /
	*/////////////////////////////////////////////////////////////////////////////////
    private void displayDashboard(){
        SmartDashboard.putNumber("ElevatorEncoder", elevatorEncoder.get());
        SmartDashboard.putString("HangState", hangState.toString());
        SmartDashboard.putNumber("Hang Up Counter", hangUpCounter);
        SmartDashboard.putNumber("Pull Up Counter", pullUpCounter);
        SmartDashboard.putNumber("WinchEncoder", winchEncoder.get());
        SmartDashboard.putString("WinchState", hangState.toString());

    }


    /*////////////////////////////////////////////////////////////////////////////////
	/ 																				 /
	/								     RUN								         /
	/																				 /
    */////////////////////////////////////////////////////////////////////////////////
    


    public void run() { // Different states of the code
        switch (hangState) {
        case TESTING: // The robot is doing nothing
            break;
        case SEQUENCEUP: // Runs the sequence of hanging mechanism so that its up
            sequenceUp();
            break;
        case SEQUENCEDOWN: // Runs the sequence backward to hang
            sequenceDown();
            break; 
        case STOP: // Stops the winch
            winchStop();
            break;
        case WINCHUP: // Runs the winch
            winchUp();
            break;
        }
        //check if the current state is winchup is running the winch , then allow it to stop
        displayDashboard();
    }
}