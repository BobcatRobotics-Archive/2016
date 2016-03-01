/*
 * Basic Automode to test frame work
 */
package org.usfirst.frc.team177.auto;

import org.usfirst.frc.team177.robot.*;
import edu.wpi.first.wpilibj.DoubleSolenoid;

public class AutoModeDriveForwardTurnAndFire extends AutoMode {
    
	enum AutoStates {
		PutPickupDown,
		DriveForward,
		Pause,
		Turn,
		PauseAgain,
		Fire,
		Stop
	};
	
    private AutoStates state = AutoStates.DriveForward;
    //State Machine Auto
    long lastDriveForwardEventTime = 0;
    double pickupDownDelay = 2000;
    double driveForwardDelay = 4500;
    double pauseDelay = 500;
    double turnHeading = 45;
    double turnDelay = 3000;
    
    boolean fireNow = false;

    public AutoModeDriveForwardTurnAndFire(Robot robot) {
        super(robot);
        System.out.println("AutoModeDriveForward Constructor");
    }    
    
    public void autoInit() {    	
    	state = AutoStates.PutPickupDown; 
    	fireNow = false;
    	lastDriveForwardEventTime = 0;
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
    				state = AutoStates.Turn;
    			}
    			break;	
    		case Turn:
    			if(lastDriveForwardEventTime == 0) { 
    				lastDriveForwardEventTime = System.currentTimeMillis();
    			}
    			robot.transferPneumatic.set(DoubleSolenoid.Value.kForward);
    			robot.drive.tankDrive(-0.75,0.75);
    			if((robot.locator.GetHeading() > turnHeading && robot.locator.GetHeading() < 180) || System.currentTimeMillis() - lastDriveForwardEventTime > turnDelay) {
    				robot.drive.tankDrive(0,0);
    				lastDriveForwardEventTime = 0;
    				state = AutoStates.Fire;
    			}
    			break;
    		case Fire:	
    			fireNow = true;
    			state = AutoStates.Stop;
    			break;    			
    		case Stop:
    		default:
    			robot.drive.tankDrive(0,0);
    			fireNow = false;
    			break;
		}
    	robot.catapult.loop(fireNow);
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
