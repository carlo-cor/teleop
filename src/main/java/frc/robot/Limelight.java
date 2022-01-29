package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Limelight{
    NetworkTable table = null;
    NetworkTableEntry camModeEntry = null;
    NetworkTableEntry ledModeEntry = null;
    NetworkTableEntry xOffsetEntry = null;
    NetworkTableEntry yOffsetEntry = null;
    NetworkTableEntry cameraEntry = null;

    
    public Limelight(){
        table = null;
        camModeEntry = null;
        ledModeEntry = null;
        yOffsetEntry = null;
        xOffsetEntry = null;
        cameraEntry = null;
    }
    public enum State{          //The two states the limelight will be in
        TRACKING, DRIVING
    }
    public State camState = State.DRIVING; //The limelight starts in driving mode

    public void setDriving(){               //set the state of the limelight to driving
        camState = State.DRIVING;
    }
    public void setTracking(){              //set the state of the limelight to tracking 
        camState = State.TRACKING;
        
    }
    private NetworkTable getTable(){ // Checks to see if the table has been created or not, if not it returns null
        if(table != null){
            return table;
        }

        table = NetworkTableInstance.getDefault().getTable("limelight"); //Stores the networktable to variable
        return table;
    }

    private NetworkTableEntry getCamEntry(){ //sees if the camMode has been created or not, if not return as null
        NetworkTable localTable = getTable();
        if(localTable == null){
            return null;
        }

        camModeEntry = localTable.getEntry("camMode"); // Stores the camMode to a variable
        return camModeEntry;
    }

    private NetworkTableEntry getLedEntry(){ // Checks if the LedMode is created or not, if not return as null
        NetworkTable localTable = getTable();
        if(localTable == null){
            return null;
        }

        ledModeEntry = localTable.getEntry("ledMode"); // Stores ledMode to a variable
        return ledModeEntry;
    }

      
    public NetworkTableEntry getHorizontalEntry(){ //Checks if its null, if not assigns it as the entry
        NetworkTable localHorizontal = getTable();
        if(localHorizontal == null){
            return null;
        }
        yOffsetEntry = localHorizontal.getEntry("tx");
        return yOffsetEntry;
    }   
    
    public NetworkTableEntry getVerticalEntry(){ //Checks if its null, if not assigns it as the entry
        NetworkTable localTable = getTable();
        if(localTable == null){
            return null;
        }
        xOffsetEntry = localTable.getEntry("ty");
        return xOffsetEntry;
    }

    private NetworkTableEntry getSeenEntry(){
        NetworkTable localTable = getTable();
        if(localTable == null){
            return null;
        }
        cameraEntry = localTable.getEntry("tv");
        return cameraEntry;
    }
    


    private int trackMode(){ // If camMode and the ledMode is MADE then set them to tracking mode
        if(getCamEntry() != null && getLedEntry() != null){
            getCamEntry().setNumber(0);
            getLedEntry().setNumber(3);
            return 0;
        }
        else                    //Code knows that the table does not exist
        {
            return -1;
        }
    }

    private int driverMode(){ //If the camMode and ledMode is MADE then set them to driver mode
        if(getCamEntry() != null && getLedEntry() != null){
            getCamEntry().setNumber(1);
            getLedEntry().setNumber(1);
            return 0;
        }
        else{                  //Code knows that the table does not exist
            return -1;
        }
    }
     
    


    public double getHorizontalOffSet(){ // get the horizontal degrees
        return getHorizontalEntry().getDouble(0);
    }
    
    public double getVerticalOffset(){  //get the vertical degrees
        return getVerticalEntry().getDouble(0);
    }
    
    public boolean targetSeen(){
        return getSeenEntry().getDouble(0) == 1;
    }

    public void displayDashboard(){     //display the offset 
        SmartDashboard.putNumber("HorizontalOffset", getHorizontalOffSet());
        SmartDashboard.putNumber("VerticalOffset", getVerticalOffset());
    }
    public void runCode(){
        switch (camState){
            case DRIVING:
            driverMode();
            break;
            case TRACKING:
            trackMode();
            break;
        }
    }

   




}