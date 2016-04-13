
package org.usfirst.frc.team177.auto;

import org.usfirst.frc.team177.robot.*;
import org.usfirst.frc.team177.robot.Catapult.catapultStates;

import edu.wpi.first.wpilibj.DoubleSolenoid;

public class AutoModePortCullis extends AutoMode {
    
	enum AutoStates {
		PutPickupDown,
		DriveForward,
		Turn,
		PauseForAim,
		Aim,
		Fire,
		Stop
	};
	
    private AutoStates state = AutoStates.DriveForward;
    //State Machine Auto
    long lastDriveForwardEventTime = 0;
    
    double pickupDownDelay = 1000;
    double driveForwardDelay = 3000;
    double pauseForAimDelay = 1000;
    
    double driveForwardSpeed = -0.75;
        
    boolean fireNow = false;
    boolean aimNow = false;
    
    private static final double[] turnAngles = {30, 10, -10, -20}; //guesses
	private static final int turnTimeout = 3000;
	Robot.Turns turn;	
    double turnAngle = 0;

    public AutoModePortCullis(Robot robot, Robot.Turns turn) {
        super(robot);
        this.turn = turn;
        if(turn != Robot.Turns.NoTurn)
		{
			turnAngle = turnAngles[turn.getIndex()];
		}      
        System.out.println("AutoModePortCullis Constructor");
    }    
    
    public void autoInit() {    	
    	state = AutoStates.PutPickupDown; 
    	fireNow = false;
    	lastDriveForwardEventTime = 0;
    	robot.locator.Reset();
    	robot.shiftPneumatic.set(true); //high gear
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
    			robot.drive.tankDrive(driveForwardSpeed, driveForwardSpeed);
    			if(System.currentTimeMillis() - lastDriveForwardEventTime > driveForwardDelay) {
    				robot.drive.tankDrive(0,0);
    				lastDriveForwardEventTime = 0;
    				if (turn == Robot.Turns.NoTurn)
    				{
    					state = AutoStates.PauseForAim;
    				}
    				else
    				{
    					state = AutoStates.Turn;
    				}
    			}
    			break;    		
    			
    		case Turn:
    			if(lastDriveForwardEventTime == 0) { 
    				lastDriveForwardEventTime = System.currentTimeMillis();
    			}
    			
    			if (turnAngle > 0) 
    			{
    				// turn right
	    			robot.drive.tankDrive(-0.75,0.75);
	    			if(robot.locator.GetHeading() > turnAngle || System.currentTimeMillis() - lastDriveForwardEventTime > turnTimeout) {
	    				robot.drive.tankDrive(0,0);
	    				lastDriveForwardEventTime = 0;
	    				state = AutoStates.PauseForAim;
	    			}
    			}
    			else
    			{
    				//turn left
    				robot.drive.tankDrive(0.75,-0.75);
	    			if(robot.locator.GetHeading() < (360 + turnAngle) || System.currentTimeMillis() - lastDriveForwardEventTime > turnTimeout) {
	    				robot.drive.tankDrive(0,0);
	    				lastDriveForwardEventTime = 0;
	    				state = AutoStates.PauseForAim;
	    			}
    			}
    			break;
    			
    		case PauseForAim:
    			if(lastDriveForwardEventTime == 0) { 
    				lastDriveForwardEventTime = System.currentTimeMillis();
    			}		
    			if(System.currentTimeMillis() - lastDriveForwardEventTime > pauseForAimDelay) {
    				robot.drive.tankDrive(0,0);
    				lastDriveForwardEventTime = 0;
    				state = AutoStates.Aim;
    			}
    			break;
    			
    		case Aim:
    			aimNow = true;
    			state = AutoStates.Fire;
    			break;
    			
    		case Fire:	
    			aimNow = false;
    			if(robot.catapult.getState() == catapultStates.ReadyToFire)
    			{
    				fireNow = true;
    				state = AutoStates.Stop;
    			}
    			break; 
    			
    		case Stop:
    		default:
    			robot.drive.tankDrive(0,0);
    			fireNow = false;
    			break;
		}
    	robot.catapult.loop(fireNow, aimNow);
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
