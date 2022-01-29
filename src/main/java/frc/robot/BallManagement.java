package frc.robot;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class BallManagement {
    // INTAKE VARIABLES//
    private SpeedController intakeMotor; // SPEEDCONTROLLER FOR INTAKE
    private DoubleSolenoid piston; // PISTON
    private double intakeSpeed = 0.60; // SPEED FOR INTAKE
    private DigitalInput intakeSensor;  // SENSOR ON THE INTAKE

    // CONVEYOR BELT VARIABLES //
    private SpeedController conveyorMotor; // SPEEDCONTROLLER FOR CONVEYOR
    private double conveyorSpeed = 0.75; // REGULAR SPEED FOR THE CONVEYOR BELT // 0.8
    private DigitalInput conveyorSensor; // SENSOR AT THE END OF THE CONVEYOR (SENSOR FOR PICKING UP POWER CELLS)

    // TRANSITIONAL WHEEL VARIABLES //
    private SpeedController transitionWheelMotor; // MOTOR TO TURN THE TRANSITION WHEEL
    private double turnSpeedValue = 0.55; // SPEED OF TRANSITION WHEEL
    private DigitalInput dropDownSensor; // SENSOR JUST RIGHT BEFORE THE DROP DOWN ZONE

    // TIMER VARIABLES //
    private Timer conveyorTime; // TIME FOR TURNING THE TRANSITION WHEEL DURING PICKUP
    private Timer feedTime; // TIME FOR TURNING THE TRANSITION WHEEL DURING FEEDING
    private Timer rotatingTime; // TIME TO ROTATE THE CONVEYOR FOR STORING THE 4TH AND 5TH BALLS
    private static double intervalBetweenFeeding = 0.6; // TIME BETWEEN FEEDING // 0.55
    private static double intervalForThreeBalls = 1; 
    private static double intervalBetweenRotating = 0.0; // TIME BETWEEN EACH ROTATION
    private static double intervalForConveyor = 0.80; // TIME FOR MOVING THE CONVEYOR FOR THE 4TH AND 5TH BALLS

    // CHECK VARAIBLES //
    private boolean lastIntakeSensorValue; // RETURNS THE LAST VALUE FOR THE CONVEYOR SENSOR
    private boolean lastPickUpSensorValue; // RETURNS THE LAST VALUE FOR THE PICKUP SENSOR
    private boolean lastDropDownSensorValue; // RETURNS THE LAST VALUE FOR THE DROP DOWN SENSOR

    // CHECKS PORTIONS //
    private boolean ballHasEnteredPickUp; // BALL HAS ENTERED THE START OF THE TRANSITION WHEEL
    private boolean ballHasLeftPickUp; // BALL HAS LEFT THE PICK UP SENSOR
    private boolean ballHasEnteredIntake; // BALL HAS ENTERED THE CONVEYOR
    private boolean ballHasLeftIntake; // BALL HAS LEFT THE CONVEYOR
    private boolean ballHasEnteredDropDown; // BALL HAS ENTERED THE DROP DOWN SENSOR
    private boolean ballHasLeftDropDown; // BALL HAS LEFT THE DROP DOWN SENSOR

    public BallManagement(SpeedController newIntake, SpeedController newConveyor, SpeedController newTransitionWheel,
            DoubleSolenoid newPiston, DigitalInput newIntakeSensor, DigitalInput newConveyorSensor,
            DigitalInput newDropDownSensor, Timer newConveyorTime, Timer newRotatingTime, Timer newFeedTime) {
        intakeMotor = newIntake;
        conveyorMotor = newConveyor;
        transitionWheelMotor = newTransitionWheel;
        piston = newPiston;
        intakeSensor = newIntakeSensor;
        conveyorSensor = newConveyorSensor;
        dropDownSensor = newDropDownSensor;
        rotatingTime = newRotatingTime;
        conveyorTime = newConveyorTime;
        feedTime = newFeedTime;
        feedTime.reset();
        feedTime.start();
    }

    // ENUMERATIONS //
    public enum ballManagementState {
        TESTING, STOP, UNJAM, INTAKEMANAGEMENT, THREEBALLFEED, FEEDMANAGEMENT, LOADOVERRIDE , EXTEND, RETRACT, PRELOAD ,OUTPUT // States of the Ball Management System
    }

    ballManagementState State = ballManagementState.STOP;

    public void setTesting(){   // Tests the methods 
        State = ballManagementState.TESTING;
    }

    public void setStop() { // Stops the Motors
        State = ballManagementState.STOP;
    }

    public void setUnjam(){ // UnJams the balls in the conveyor and intake
        State = ballManagementState.UNJAM;
    }

    public void setIntakeManagement() { // Runs the Intake part of the Ball Management System
        State = ballManagementState.INTAKEMANAGEMENT;
    }

    public void setThreeBallFeed(){
        State = ballManagementState.THREEBALLFEED;
    }

    public void setFeedManagement() { // Runs the Shooting part of the Ball Management System
        State = ballManagementState.FEEDMANAGEMENT;
    }

    public void setOutPut() { // Reverse all the motors to Output
        State = ballManagementState.OUTPUT;
    }

    public void setExtend() { // Extends the intake out
        State = ballManagementState.EXTEND;
    }

    public void setRetract() { // Retracts the intake back into position
        State = ballManagementState.RETRACT;
    }

    public void setPreLoad(){   // Runs Pre-load for Autonomous
        State = ballManagementState.PRELOAD;
    }

    public void setLoadOverride(){    // Runs all the power cells into the Drop Down
        State = ballManagementState.LOADOVERRIDE;
    }

    // CHECKS //
    private boolean currentIntakeSensorValue() { // Checks if the Conveyor Sensor is being pressed
        return intakeSensor.get();
    }

    private boolean currentConveyorSensorValue() { // Checks if the Pick Up Sensor is being pressed
        return conveyorSensor.get();
    }

    private boolean currentDropDownSensorValue() { // Checks if the Drop Down Sensor is being pressed
        return dropDownSensor.get();
    }

    // METHOD: INTAKE //
    public void driveIntake(double speed){
        intakeMotor.set(speed);
    }

    public void moveIntake() { // Moves the Intake
        intakeMotor.set(intakeSpeed);
    }

    private void reverseIntake() { // Reverse the Intake
        intakeMotor.set(-intakeSpeed);
    }

    private void stopIntake() { // Stops the Intake
        intakeMotor.set(0);
    }

    private void retract() { // Retracts the Intake into starting position
        piston.set(DoubleSolenoid.Value.kReverse);
    }

    private void extend() { // Extends the Intake
        piston.set(DoubleSolenoid.Value.kForward);
    }

    // METHOD: CONVEYOR //
    public void driveConveyor(double speed){
        conveyorMotor.set(speed);
    }
    private void moveConveyor() { // Moves the Conveyor
        conveyorMotor.set(conveyorSpeed);
    }

    private void reverseConveyor() { // Reverse the Conveyor
        conveyorMotor.set(-conveyorSpeed);
    }

    private void stopConveyor() { // Stops the Conveyor
        conveyorMotor.set(0);
    }

    // METHOD: TRANSITIONAL WHEEL //
    public void driveTransitionWheel(double speed){
        transitionWheelMotor.set(speed);
    }

    private void moveTransitionalWheel() { // Moves the Transitional Wheel
        transitionWheelMotor.set(-turnSpeedValue);
    }

    private void reverseTransitionWheel() { // Reverse Transitional Wheel 
        transitionWheelMotor.set(turnSpeedValue);
    }

    private void stopTransitionalWheel() { // Stops the Transitioanl Wheel
        transitionWheelMotor.set(0);
    }

    // STOP ALL MOTORS //
    private void stopAllMotors() { // Stops the Intake, Conveyor, and Transitional Wheel
        stopIntake();
        stopConveyor();
        stopTransitionalWheel();
    }

    // RUN ALL MOTORS //
    private void runAllMotors() { // Runs the Intake, Conveyor, and Transitional Wheel
        moveIntake();
        moveConveyor();
        moveTransitionalWheel();
    }

    private void outPut() { // ONLY FOR TESTING: Reverse all motors
        reverseIntake();
        reverseConveyor();
        reverseTransitionWheel();
    }

    public void turnOffIntakeMode() { // TURNS OFF THE INTAKE PART OF THE BALL MANAGEMENT SYSTEM
        lastIntakeSensorValue = false;
        lastPickUpSensorValue = false;
        lastDropDownSensorValue = false;
        conveyorTime.reset();
        conveyorTime.stop();
        rotatingTime.reset();
        rotatingTime.stop();
    }

    // BALL MANAGEMENT SYSTEM //
    public void preLoad(){  // RUNS PRE-LOAD FOR AUTONOMOUS
        moveIntake();
        if(currentIntakeSensorValue()){
            conveyorTime.reset();
            conveyorTime.start();
            moveConveyor();
        }

        else if(!currentIntakeSensorValue() && conveyorTime.get() > intervalForConveyor){
            conveyorTime.stop();
            conveyorTime.reset();
            stopConveyor();
        }
    }

    private void unjamIntake(){     // UNJAM ANY BALLS IN THE CONVEYOR AND INTAKE
        reverseConveyor();
        reverseIntake();
    }

    private void runIntakeManagement(){ // RUNS BALL MANAGEMENT SYSTEM FOR THE INTAKE
        moveIntake();
        if(currentDropDownSensorValue()){   // WHEN DROP DOWN SENSOR IS PRESSED STOP ALL MOTORS
            stopAllMotors();
            return;
        }
        
        if(currentIntakeSensorValue()){
                conveyorTime.start();
                moveConveyor();
                System.out.println("ON INTAKE: MOVE CONVEYOR");
                if(currentConveyorSensorValue()){
                    rotatingTime.start();
                    moveTransitionalWheel(); 
                    System.out.println("ON CONVEYOR: MOVE WHEEL");
                }

                else if(!currentConveyorSensorValue() && rotatingTime.get() > intervalBetweenRotating){
                    rotatingTime.stop();
                    rotatingTime.reset();
                    stopTransitionalWheel();
                    System.out.println("OFF CONVEYOR: STOP WHEEL");
                }
        }

        else if(!currentIntakeSensorValue() && conveyorTime.get() > intervalForConveyor){
            conveyorTime.stop();
            conveyorTime.reset();
            stopConveyor();
            System.out.println("OFF INTAKE: STOP CONVEYOR");
              
            rotatingTime.stop();
            rotatingTime.reset();
            stopTransitionalWheel();
            System.out.println("OFF INTAKE: STOP WHEEL");

        }
    }

    private void loadLessThan5Balls(){  // Prep up the balls by the drop down point before shooting  
        if(currentDropDownSensorValue()){
            stopAllMotors();
        }

        else{
            moveConveyor();
            moveTransitionalWheel();
        }
    }

    public void runThreeBallFeedingManagement() {    // Feeding for three ball auto 
        if(ballHasLeftDropDown){                     // Creates delay between feeding only when needed
            feedTime.reset();               
            feedTime.start();
        }
        
        if(feedTime.get() > intervalForThreeBalls){
            runAllMotors();
        }

        else if(ballHasEnteredDropDown){
            stopAllMotors();
        }
    }

    public void runFeedingManagement(){             // Feeding method for teleop
        if(ballHasLeftDropDown){                    // Creates delay between feeding when needed
            feedTime.reset();               
            feedTime.start();
        }
        
        if(feedTime.get() > intervalBetweenFeeding){
            runAllMotors();
        }

        else if(ballHasEnteredDropDown){
            stopAllMotors();
        }
    }

    // METHOD: DISPLAY //
    public void display() { // Display time, case, current, and last value of the start and end sensor for the ball management system
        SmartDashboard.putNumber("CONVEYOR DELAY: ", conveyorTime.get());
        SmartDashboard.putNumber("ROTATING DELAY: ", rotatingTime.get());
        SmartDashboard.putNumber("FEED DELAY: ", feedTime.get());
        SmartDashboard.putBoolean("BALL HAS ENTERED", ballHasEnteredDropDown);
        SmartDashboard.putBoolean("BALL HAS LEFT", ballHasLeftDropDown);
        SmartDashboard.putBoolean("CURRENT VALUE OF INTAKE SENSOR: ", intakeSensor.get());
        SmartDashboard.putBoolean("CURRENT VALUE OF CONVEYOR SENSOR: ", conveyorSensor.get());
        SmartDashboard.putBoolean("CURRENT VALUE OF DROPDOWN SENSOR: ", dropDownSensor.get());
        SmartDashboard.putString("BALL MANAGEMENT SYSTEM STATE: ", State.toString());

        if(conveyorTime.get() != intervalForConveyor && conveyorTime.get() != 0){
            SmartDashboard.putString("INTAKE SEQUENCE", "DON'T INTAKE!");    
        }
        else if(conveyorTime.get() == 0){
            SmartDashboard.putString("INTAKE SEQUENCE", "YOU CAN INTAKE!");
        }
    }

    // RUNS STATES //
    public void run() {
        ballHasLeftDropDown = !currentDropDownSensorValue() && lastDropDownSensorValue;
        ballHasEnteredDropDown = currentDropDownSensorValue() && !lastDropDownSensorValue;
        display();

        switch (State) {

        case TESTING:       // Runs Test
            break;

        case STOP: // Stops all of the motors
            stopAllMotors();
            turnOffIntakeMode();
            break;

        case PRELOAD:   // Runs Pre-load 
            preLoad();
            break;

        case UNJAM:     // Runs Unjam
            unjamIntake();
            break;

        case INTAKEMANAGEMENT: // Runs the Intake Management 
            runIntakeManagement();
            break;

        case THREEBALLFEED:
            runThreeBallFeedingManagement();
            break;

        case FEEDMANAGEMENT: // Runs the Feed Management
            runFeedingManagement();
            break;

        case LOADOVERRIDE:  // Runs all of the power cells into the Drop Down   
            loadLessThan5Balls();
            break;

        case EXTEND: // Runs to Extend the Inake
            extend();
            break;

        case RETRACT: // Runs to Retract the Intake
            retract();
            break;

        case OUTPUT: // Runs the Output Balls out of the System
            outPut();
            break;

        }
        lastDropDownSensorValue = currentDropDownSensorValue();     // Returns the last value 
    }
}