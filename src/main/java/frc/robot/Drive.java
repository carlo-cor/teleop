package frc.robot;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class Drive{

    // DRIVE VARIABLES //
    private SpeedControllerGroup leftSide;  // Left side of the drive 
    private SpeedControllerGroup rightSide; // Right side of the drive 
    private DifferentialDrive diffDrive;    // Both right and left drive 
    
    private double dLSpeed; // speed of the left side of the drive //dXSpeed
    private double dRSpeed; // speed of the right side of the drive //dYSpeed

    public Drive(SpeedController rFront, SpeedController rBack, SpeedController lFront, SpeedController lBack){
        leftSide = new SpeedControllerGroup(lFront, lBack);
        rightSide = new SpeedControllerGroup(rFront, rBack);
        diffDrive = new DifferentialDrive(leftSide, rightSide);
        dLSpeed = 0; 
        dRSpeed = 0;
    }
    
    // SET SPEEDS //
    public double setLeftSpeed(double dLeftSpeed){    // sets the speed for the left side // X Speed
        dLSpeed = dLeftSpeed;
        return dLeftSpeed;
    }

    public double setRightSpeed(double dRightSpeed){  // sets the speed for the right side // Y Speed
        dRSpeed = dRightSpeed;
        return dRightSpeed;
    }
    
    // DEADZONE //
    private double deadZone(double dJoystick){   // Deadzone method 
        if(Math.abs(dJoystick) < 0.2){
            return 0;
        } else{
            return dJoystick;
        }
    }

    // RUN //
    public void arcadeRun(double xSpeed, double ySpeed){ 
        diffDrive.arcadeDrive(deadZone(ySpeed), deadZone(-xSpeed));  
    }

    public void tankRun(){
        diffDrive.tankDrive(deadZone(dLSpeed), deadZone(dRSpeed));  // Tank drive
    }
}