/*
 * Basic Automode to test frame work
 */
package org.usfirst.frc.team177.auto;

import org.usfirst.frc.team177.robot.*;

import edu.wpi.first.wpilibj.DoubleSolenoid;

public class AutoModeDriveToTransferDown extends AutoMode {
    
	enum AutoStates {
		DriveForward,
		Stop
	};
	
    private AutoStates state = AutoStates.DriveForward;
    //State Machine Auto
    long lastDriveForwardEventTime = 0;
    double maxDriveForward = 10000;

    public AutoModeDriveToTransferDown(Robot robot) {
        super(robot);
        System.out.println("AutoModeDriveForward Constructor");
    }    
    
    public void autoInit() {    	
    	state = AutoStates.DriveForward;    	
    }

    public void autoPeriodic() {
    	switch(state)
    	{
    		case DriveForward:
    			if(lastDriveForwardEventTime == 0) { 
    				lastDriveForwardEventTime = System.currentTimeMillis();
    			}
    			robot.transferPneumatic.set(DoubleSolenoid.Value.kForward);
    			
    			if(DriveTo(130, 0, -0.75) || System.currentTimeMillis() - lastDriveForwardEventTime > maxDriveForward) {
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
