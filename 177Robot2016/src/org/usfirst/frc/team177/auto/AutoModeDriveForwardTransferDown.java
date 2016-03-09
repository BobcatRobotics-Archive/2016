/*
 * Basic Automode to test frame work
 */
package org.usfirst.frc.team177.auto;

import org.usfirst.frc.team177.robot.*;
import edu.wpi.first.wpilibj.DoubleSolenoid;

public class AutoModeDriveForwardTransferDown extends AutoMode {
    
	enum AutoStates {
		PutPickupDown,
		DriveForward,
		Stop
	};
	
    private AutoStates state = AutoStates.DriveForward;
    //State Machine Auto
    long lastDriveForwardEventTime = 0;
    double pickupDownDelay = 2000;
    double driveForwardDelay = 4000;

    public AutoModeDriveForwardTransferDown(Robot robot) {
        super(robot);
        System.out.println("AutoModeDriveForward Constructor");
    }    
    
    public void autoInit() {    	
    	state = AutoStates.PutPickupDown;    	
    }

    public void autoPeriodic() {
    	switch(state)
    	{
    		case PutPickupDown:	
    			if(lastDriveForwardEventTime == 0) { 
    				lastDriveForwardEventTime = System.currentTimeMillis();
    			}
    			robot.transferPneumatic.set(DoubleSolenoid.Value.kReverse);
    			if(System.currentTimeMillis() - lastDriveForwardEventTime > pickupDownDelay) {
    				robot.drive.tankDrive(0,0);
    				lastDriveForwardEventTime = 0;
    				state = AutoStates.DriveForward;
    			}
    			break;
    		case DriveForward:
    			if(lastDriveForwardEventTime == 0) { 
    				lastDriveForwardEventTime = System.currentTimeMillis();
    			}
    			robot.transferPneumatic.set(DoubleSolenoid.Value.kReverse);
    			robot.drive.tankDrive(-0.75,-0.75);
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
        return "DriveForwardTransferDown";
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
