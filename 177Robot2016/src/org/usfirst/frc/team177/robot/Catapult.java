package org.usfirst.frc.team177.robot;

import org.usfirst.frc.team177.lib.BasicPIDController;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Catapult {

	private static final boolean usePIDForAim = true;
	private static final boolean useLoopOnAim = false;
	private static final int loopOnAimMaxLoop = 2;
	private int loopOnAimLoopCount;
	
	private BasicPIDController SteerPID;
    private static double SteerP 	= 0.5;    //Proportial gain for Steering System
    private static double SteerI 	= 1;  //Integral gain for Steering System
    private static double SteerD 	= 0.00;   //Derivative gain for Steering System
    private static double SteerMax 	= 0.75;   	  //Max Saturation value for control
    private static double SteerMin 	= -0.75;     //Min Saturation value for control
    
    private long lastRanSteerPID;
	
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
    private static final double aimTimeout = 2500; //ms   
    private static final double aimThreshold = 0.25; //+/- 0.25 degrees == on target
    private static final double turnSpeed = 0.75;
    private static final double bearingFudgeFactor = -1;  //negative is left
    		
    private long lastShooterEventTime = 0;
    private double targetHeading = 0;
    private boolean cancelAim = false;
    
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
    	
    	SteerPID = new BasicPIDController(SteerP,SteerI,SteerD);
        SteerPID.setOutputRange(SteerMin, SteerMax);
    }
    
    //wrapper function to avoid having to rework a bunch of existing code
    public void loop(boolean fire)
    {
    	loop(fire, false);
    }
    
    public void cancelAim()
    {
    	cancelAim = true;
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
				loopOnAimLoopCount = 0;
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
				
				if(usePIDForAim) {
					SteerPID.reset();
					lastRanSteerPID = System.currentTimeMillis();
				}
				
				lastShooterEventTime = System.currentTimeMillis();		
				
				robot.shiftPneumatic.set(true); //high gear
				
				cancelAim = false;
			}
			
			double targetBearing = targetHeading - robot.locator.GetHeading();
			if(targetBearing > 180) targetBearing -= 360;
			if(targetBearing < -180) targetBearing += 360;			
			
			SmartDashboard.putString("Aim Sequence", "Target Bearing " + targetBearing);
			
			if(usePIDForAim)
			{
				//Calculate time step
		        long now = System.currentTimeMillis();
		        double dT = (now - lastRanSteerPID)/1000.0; //dT in seconds
		        lastRanSteerPID = now;
				double steer = SteerPID.calculate(targetBearing, dT);
				System.out.println("PID: " + targetBearing + " " + steer);
				robot.drive.tankDrive(-steer, steer);
			}
			else
			{
				//determine turn direction			
				if(targetBearing < 0) {
					//turn left
					robot.drive.tankDrive(turnSpeed,-1.0*turnSpeed);
				} else {		
					//turn right
					robot.drive.tankDrive(-1.0*turnSpeed,turnSpeed);
				}
			}
						
			if(cancelAim || Math.abs(targetBearing) < aimThreshold || System.currentTimeMillis() - lastShooterEventTime > aimTimeout) {
				if(Math.abs(targetBearing) < aimThreshold)
				{
					if(useLoopOnAim && ++loopOnAimLoopCount < loopOnAimMaxLoop)
					{
						//take another picture to confirm...
						double bearing = robot.vision.getBearing();
						if (Math.abs(bearing) < aimThreshold)
						{
							//Aim is good!
							SmartDashboard.putString("Aim Sequence", "Aim Confirmed");
							//Change this to auto fire.
							//catapultState = catapultStates.NoBall;
							catapultState = catapultStates.ReadyToFire;
						}
						else
						{
							//retrying Aim
							SmartDashboard.putString("Aim Sequence", "Retrying Aim");						//
							catapultState = catapultStates.Aiming;
						}						
					} 
					else
					{
						SmartDashboard.putString("Aim Sequence", "Aim Successful");
						//Fire!
						//Change this to auto fire.
						//catapultState = catapultStates.NoBall;
						catapultState = catapultStates.ReadyToFire;
					}
					
				}
				else
				{					
					SmartDashboard.putString("Aim Sequence", "Aim Timeout");
					//Don't Fire
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
