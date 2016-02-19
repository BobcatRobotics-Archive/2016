package org.usfirst.frc.team177.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;

public class Catapult {

	
	private Solenoid latchPneumatic; 
	private DoubleSolenoid pusherPneumatic;
	private DigitalInput catapultRetractedLimitSwitchA;
	private DigitalInput catapultRetractedLimitSwitchB;
	
	/**Enums**/
    public enum catapultStates {
    	NoBall,
    	Pickup,
    	BallsIn,
    	PreparingToFire,
    	ReadyToFire
    };
    
    //State Machine Shooter
    private catapultStates catapultState = catapultStates.NoBall;
    private static final double stateDelay = 1000; //ms   
    private long lastShooterEventTime = 0;
    
    // definitions to improve code readability
    //latch pneumatic states
    private static final boolean LATCHED = false;
    private static final boolean UNLATCHED = true;
    //pusher states
    private static final DoubleSolenoid.Value EXTENDED = DoubleSolenoid.Value.kReverse;
    private static final DoubleSolenoid.Value RETRACTED = DoubleSolenoid.Value.kForward;
    	
    Catapult(Solenoid latchPneumatic, DoubleSolenoid pusherPneumatic, DigitalInput catapultRetractedLimitSwitchA, DigitalInput catapultRetractedLimitSwitchB)
    {
    	this.latchPneumatic = latchPneumatic;
    	this.pusherPneumatic = pusherPneumatic;
       	this.catapultRetractedLimitSwitchA = catapultRetractedLimitSwitchA;
       	this.catapultRetractedLimitSwitchB = catapultRetractedLimitSwitchB;
    }
    public void loop(boolean fire)
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
			
			if(catapultRetractedLimitSwitchA.get() || catapultRetractedLimitSwitchB.get() 
				|| (System.currentTimeMillis() - lastShooterEventTime > stateDelay)){
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
			break;
		default:
			break;
		}
    }
    
    
    public void setState(catapultStates newState)
    {
    	catapultState = newState;
    }
    
    
}
