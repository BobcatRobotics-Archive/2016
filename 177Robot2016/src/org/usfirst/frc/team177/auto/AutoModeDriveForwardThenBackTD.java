/*
 * Basic Automode to test frame work
 */
package org.usfirst.frc.team177.auto;

import org.usfirst.frc.team177.robot.*;
import edu.wpi.first.wpilibj.DoubleSolenoid;

public class AutoModeDriveForwardThenBackTD extends AutoMode {
    
	enum AutoStates {
		PutPickupDown,
		DriveForward,
		Pause,
		DriveBackward,
		Stop
	};
	
    private AutoStates state = AutoStates.DriveForward;
    //State Machine Auto
    long lastDriveForwardEventTime = 0;
    double pickupDownDelay = 2000;
    double driveForwardDelay = 3500;
    double pauseDelay = 2000;
    double driveBackwardDelay = 4000;

    public AutoModeDriveForwardThenBackTD(Robot robot) {
        super(robot);
        System.out.println("AutoModeDriveForward The back Constructor");
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
    			robot.transferPneumatic.set(DoubleSolenoid.Value.kForward);
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
    			robot.transferPneumatic.set(DoubleSolenoid.Value.kForward);
    			robot.drive.tankDrive(-0.75,-0.75);
    			if(System.currentTimeMillis() - lastDriveForwardEventTime > driveForwardDelay) {
    				robot.drive.tankDrive(0,0);
    				lastDriveForwardEventTime = 0;
    				state = AutoStates.Pause;
    			}
    			break;
    		case Pause:
    			if(lastDriveForwardEventTime == 0) { 
    				lastDriveForwardEventTime = System.currentTimeMillis();
    			}		
    			if(System.currentTimeMillis() - lastDriveForwardEventTime > pauseDelay) {
    				robot.drive.tankDrive(0,0);
    				lastDriveForwardEventTime = 0;
    				state = AutoStates.DriveBackward;
    			}
    			break;
    		case DriveBackward:
    			if(lastDriveForwardEventTime == 0) { 
    				lastDriveForwardEventTime = System.currentTimeMillis();
    			}
    			robot.transferPneumatic.set(DoubleSolenoid.Value.kForward);
    			robot.drive.tankDrive(0.75,0.75);
    			if(System.currentTimeMillis() - lastDriveForwardEventTime > driveBackwardDelay) {
    				robot.drive.tankDrive(0,0);
    				lastDriveForwardEventTime = 0;
    				state = AutoStates.Pause;
    			}
    			break;
    		case Stop:
    		default:
    			robot.drive.tankDrive(0,0);
    			break;
		}
    }
    	
    public String getName() {
        return "AutoModeDriveForwardThenBackTD";
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
