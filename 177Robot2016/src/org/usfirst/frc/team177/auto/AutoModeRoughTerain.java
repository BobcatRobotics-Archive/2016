/*
 * Basic Automode to test frame work
 */
package org.usfirst.frc.team177.auto;

import org.usfirst.frc.team177.robot.*;
import edu.wpi.first.wpilibj.DoubleSolenoid;

public class AutoModeRoughTerain extends AutoMode {
    
	enum AutoStates {
		DriveForward,
		Stop
	};
	
    private AutoStates state = AutoStates.DriveForward;
    //State Machine Auto
    long lastDriveForwardEventTime = 0;
    double driveForwardDelay = 5000;

    public AutoModeRoughTerain(Robot robot, double driveForwardTime) {
        super(robot);
        System.out.println("AutoModeDriveForward Constructor");
        driveForwardDelay = driveForwardTime;
    }    
    
    public void autoInit() {    	
    	state = AutoStates.DriveForward;   
    	lastDriveForwardEventTime = 0;
    }

    public void autoPeriodic() {
    	switch(state)
    	{
    		case DriveForward:
    			if(lastDriveForwardEventTime == 0) { 
    				lastDriveForwardEventTime = System.currentTimeMillis();
    			}
    			robot.drive.tankDrive(-1,-1);
    			if(System.currentTimeMillis() - lastDriveForwardEventTime > driveForwardDelay) {
    				robot.drive.tankDrive(0,0);
    				lastDriveForwardEventTime = 0;
    				state = AutoStates.Stop;
    			}
    			break;
    		case Stop:
    		default:
    			robot.drive.tankDrive(0,0);
    			break;
		}
    }
    	
    public String getName() {
        return "DriveForwardTransferUp";
    }

	@Override
	public String GetColumNames() {
		return "state";
	}


	@Override
	public String log() {
		return String.format("%s", state.toString());
	}
}
