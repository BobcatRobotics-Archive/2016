package org.usfirst.frc.team177.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Catapult {

	
	private Solenoid latchPneumatic; 
	private DoubleSolenoid pusherPneumatic;
	
	Robot robot; //reference to main implementation

	
	/**Enums**/
    public enum catapultStates {
    	NoBall,
    	Pickup,
    	BallsIn,
    	PreparingToFire,
    	ReadyToFire,
    	Aiming
    };
    
    //State Machine Shooter
    private catapultStates catapultState = catapultStates.PreparingToFire;
    private static final double stateDelay = 1000; //ms
    private static final double aimTimeout = 5000; //ms   
    private static final double aimThreshold = 0.5; //+/- 1 degrees == on target
    private static final double turnSpeed = 0.75;
    private static final double bearingFudgeFactor = -1;  //negative is left
    		
    private long lastShooterEventTime = 0;
    private double targetHeading = 0;
    
    // definitions to improve code readability
    //latch pneumatic states
    private static final boolean LATCHED = false;
    private static final boolean UNLATCHED = true;
    //pusher states
    private static final DoubleSolenoid.Value EXTENDED = DoubleSolenoid.Value.kReverse;
    private static final DoubleSolenoid.Value RETRACTED = DoubleSolenoid.Value.kForward;
    	
    Catapult(Robot robot, Solenoid latchPneumatic, DoubleSolenoid pusherPneumatic)
    {
    	this.robot = robot;
    	this.latchPneumatic = latchPneumatic;
    	this.pusherPneumatic = pusherPneumatic;   	
    }
    
    //wrapper function to avoid having to rework a bunch of existing code
    public void loop(boolean fire)
    {
    	loop(fire, false);
    }
    
    public void loop(boolean fire, boolean aimThenFire)
    {
    	//Firing State Machine
		switch (catapultState)
		{
		case NoBall:  //fire
			if(lastShooterEventTime == 0) { 
				lastShooterEventTime = System.currentTimeMillis();
			}
			latchPneumatic.set(UNLATCHED);
			pusherPneumatic.set(RETRACTED);
			if(System.currentTimeMillis() - lastShooterEventTime > stateDelay) {
				catapultState = catapultStates.Pickup;
				lastShooterEventTime = 0;
			}
			break;
		case Pickup:
			if(lastShooterEventTime == 0) { 
				lastShooterEventTime = System.currentTimeMillis();
			}
			
			latchPneumatic.set(UNLATCHED); 
			pusherPneumatic.set(EXTENDED);
			
			if((System.currentTimeMillis() - lastShooterEventTime > stateDelay)){
				catapultState = catapultStates.BallsIn;
				lastShooterEventTime = 0;
			}			
			break;
		case BallsIn:
			if(lastShooterEventTime == 0) { 
				lastShooterEventTime = System.currentTimeMillis();
			}
			latchPneumatic.set(LATCHED);
			pusherPneumatic.set(EXTENDED);
			if(System.currentTimeMillis() - lastShooterEventTime > stateDelay) {
				catapultState = catapultStates.PreparingToFire;
				lastShooterEventTime = 0;
			}
			break;
		case PreparingToFire:
			if(lastShooterEventTime == 0) { 
				lastShooterEventTime = System.currentTimeMillis();
			}
			latchPneumatic.set(LATCHED);
			pusherPneumatic.set(RETRACTED);
			if(System.currentTimeMillis() - lastShooterEventTime > stateDelay) {
				catapultState = catapultStates.ReadyToFire;
				lastShooterEventTime = 0;
			}
			break;
		case ReadyToFire:
			if(fire) {
				catapultState = catapultStates.NoBall;
			}
			else if(aimThenFire)
			{
				catapultState = catapultStates.Aiming;
			}						
			break;
		case Aiming:
			if(lastShooterEventTime == 0) { 
				//just entered state, figure out where we're turning to
				double bearing = robot.vision.getBearing();
				if(bearing == robot.vision.BAD_BEARING)
				{
					//no target!
					//todo fire anyway or give up?
					//going with give up for now
					catapultState = catapultStates.ReadyToFire;
					SmartDashboard.putString("Aim Sequence", "No Target!");
				}					
				else
				{
					double heading = robot.locator.GetHeading();
					targetHeading = heading+bearing+bearingFudgeFactor;
					//wrap to 0 - 360 range
					while (targetHeading < 0) { targetHeading += 360.0; }					
					while (targetHeading >= 360) { targetHeading -= 360.0; }					
				}								
				
				
				lastShooterEventTime = System.currentTimeMillis();		
				
				robot.shiftPneumatic.set(true); //high gear
			}
			
			double targetBearing = targetHeading - robot.locator.GetHeading();
			if(targetBearing > 180) targetBearing -= 360;
			if(targetBearing < -180) targetBearing += 360;			
			
			SmartDashboard.putString("Aim Sequence", "Target Bearing " + targetBearing);
			
			//determine turn direction			
			if(targetBearing < 0) {
				//turn left
				robot.drive.tankDrive(turnSpeed,-1.0*turnSpeed);
			} else {		
				//turn right
				robot.drive.tankDrive(-1.0*turnSpeed,turnSpeed);
			}
						
			if(Math.abs(targetBearing) < aimThreshold || System.currentTimeMillis() - lastShooterEventTime > aimTimeout) {
				if(Math.abs(targetBearing) < aimThreshold)
				{
					SmartDashboard.putString("Aim Sequence", "Aim Successful");
					//Fire!
					//Change this to auto fire.
					//catapultState = catapultStates.NoBall;
					catapultState = catapultStates.ReadyToFire;
					
				}
				else
				{
					SmartDashboard.putString("Aim Sequence", "Aim Timeout");
					//Fire! -- May want to change this
					//catapultState = catapultStates.NoBall;
					catapultState = catapultStates.ReadyToFire;
				}
								
				robot.drive.tankDrive(0,0); //stop turning				
				lastShooterEventTime = 0;
			}			
		
			break;
		default:
			break;
		}
    }
    
    public void setState(catapultStates newState)
    {
    	catapultState = newState;
    }

	public catapultStates getState() {
		return catapultState;
	}
}
