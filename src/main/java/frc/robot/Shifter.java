package frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;

public class Shifter {
    // SHIFTER VARIABLE //
    private DoubleSolenoid solenoidOne; // Controlling the speed for the drive

    public Shifter(DoubleSolenoid solenoid){
        solenoidOne = solenoid;
    }

    // METHODS //
    public void shiftSpeed(){   // Shifts the speed fast 
        solenoidOne.set(DoubleSolenoid.Value.kReverse);
    }

    public void shiftPower(){   // Shifts the speed to slow
        solenoidOne.set(DoubleSolenoid.Value.kForward);
    }

    public void run(){  // Empty
        
    }
}