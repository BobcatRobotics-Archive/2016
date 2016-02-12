
package org.usfirst.frc.team177.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
@SuppressWarnings("unused")
public class Robot extends IterativeRobot {
    final String defaultAuto = "Default";
    final String customAuto = "My Auto";
    String autoSelected;
    SendableChooser chooser;
	
    
    /**Motor constants**/
	private static final int MotorDriveRL = 3;//Rear Left 888
	private static final int MotorDriveFL = 2; //Front Left 888
	private static final int MotorDriveRR = 1; //Rear Right 888
	private static final int MotorDriveFR = 0; //Front Right 888
	
	private static final int MotorRollerTop = 4; //Top Roller 888
	private static final int MotorRollerSide = 5; //Side Roller 888
	
	/**Initialize Victors**/
	Victor rearLeftMotor = new Victor(MotorDriveRL);
	Victor frontLeftMotor = new Victor(MotorDriveFL);
	    
	Victor rearRightMotor = new Victor(MotorDriveRR);
	Victor frontRightMotor = new Victor(MotorDriveFR); 
	
	Victor rollerTopMotor = new Victor(MotorRollerTop);
	Victor rollerSideMotor = new Victor(MotorRollerSide);
	
	/**Joysticks**/    
	Joystick leftStick = new Joystick(0);
	Joystick rightStick = new Joystick(1);
	Joystick operatorStick = new Joystick(2);
	Joystick switchPanel = new Joystick(3);
	
	public RobotDrive drive = new RobotDrive(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor);
	
	/** Joystick Constants **/ //Magic Numbers found in Joystick.class
    private static final int axisY = 1;
    
    /** Solenoids **/
	public Solenoid uselessPneumatic = new Solenoid(4);
	//SAFETY: At the end of the match both the latch and the pusher should be out
	public Solenoid latchPneumatic = new Solenoid(1); //false = out
	public Solenoid pusherPneumatic = new Solenoid(2); //false = out
	public Solenoid transferPneumatic = new Solenoid(3); //false = out
	public Solenoid shiftPneumatic = new Solenoid(0);
	
    /** Digital Input **/
    //DigitalInput ballIRSwitch = new DigitalInput(); //RIP IR, died 2/11/16 at the hands of Ulf's SuperAwesome piece of Lexan
    DigitalInput readyToFireLimitSwitchA = new DigitalInput(2);
    DigitalInput readyToFireLimitSwitchB = new DigitalInput(3);
    DigitalInput leftDriveEncoder = new DigitalInput(4);
    //Pin 5 is power for the leftDriveEncoder
    DigitalInput rightDriveEncoder = new DigitalInput(6);
    //Pin 7 is power for the rightDriveEncoder
    
    /**Enums**/
    enum catapultStates {
    	NoBall,
    	Pickup,
    	BallsIn,
    	PreparingToFire,
    	ReadyToFire
    };
    enum pickupStates {
    	BallAcquired,
    	TransferUp,
    	DropBall,
    	TransferDown
    };
    
    //State Machine Shooter
    catapultStates catapultState = catapultStates.NoBall;
    double afterFiringDelay = 3000; //ms
    double latchOutDelay = 3000; //ms
    double pusherInDelay = 3000; //ms
    long lastEventTime = 0;
    
    //State Machine Pickup
    pickupStates pickupState = pickupStates.BallAcquired;
    
    
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
        chooser = new SendableChooser();
        chooser.addDefault("Default Auto", defaultAuto);
        chooser.addObject("My Auto", customAuto);
        SmartDashboard.putData("Auto choices", chooser);
        transferPneumatic.set(true);
    }
    
	/**
	 * This autonomous (along with the chooser code above) shows how to select between different autonomous modes
	 * using the dashboard. The sendable chooser code works with the Java SmartDashboard. If you prefer the LabVIEW
	 * Dashboard, remove all of the chooser code and uncomment the getString line to get the auto name from the text box
	 * below the Gyro
	 *
	 * You can add additional auto modes by adding additional comparisons to the switch structure below with additional strings.
	 * If using the SendableChooser make sure to add them to the chooser code above as well.
	 */
    
    public void autonomousInit() {
    	autoSelected = (String) chooser.getSelected();
//		autoSelected = SmartDashboard.getString("Auto Selector", defaultAuto);
		System.out.println("Auto selected: " + autoSelected);
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {
    	switch(autoSelected) {
    	case customAuto:
        //Put custom auto code here   
            break;
    	case defaultAuto:
    	default:
    	//Put default auto code here
            break;
    	}
    }
    public void teleopInit() {
    	catapultStates catapultState = catapultStates.Pickup;
    	pickupStates pickupState = pickupStates.BallAcquired;
    }
    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
    	double left = leftStick.getRawAxis(axisY);
		double right = rightStick.getRawAxis(axisY);
		shiftPneumatic.set(rightStick.getRawButton(3));
		drive.tankDrive(left, right);
		if (ballIRSwitch.get()) {  //allows driver control as long as the IR switch is not triggered.
			rollerTopMotor.set(operatorStick.getRawAxis(3));
			rollerSideMotor.set(operatorStick.getRawAxis(3) / 2);  //Scaling for the side motors
		}
		uselessPneumatic.set(operatorStick.getRawButton(1));
		/**if (ballIRSwitch.get()) { //TESTING ONLY
			uselessPneumatic.set(true); //TESTING ONLY
		} else { //TESTING ONLY
			uselessPneumatic.set(false); //TESTING ONLY
		} //TESTING ONLY
		**/
		//Firing State Machine
		switch (catapultState)
		{
		case NoBall:
			if(lastEventTime == 0) { 
				lastEventTime = System.currentTimeMillis();
			}
			latchPneumatic.set(true); //in
			pusherPneumatic.set(true); //in
			if(System.currentTimeMillis() - lastEventTime > latchOutDelay) {
				catapultState = catapultStates.Pickup;
				lastEventTime = 0;
			}
			break;
		case Pickup:
			latchPneumatic.set(true); //in
			pusherPneumatic.set(false); //out
			if(readyToFireLimitSwitchA.get() || readyToFireLimitSwitchB.get()){
				catapultState = catapultStates.BallsIn;
			}
			break;
		case BallsIn:
			if(lastEventTime == 0) { 
				lastEventTime = System.currentTimeMillis();
			}
			latchPneumatic.set(false); //out
			pusherPneumatic.set(false); //out
			if(System.currentTimeMillis() - lastEventTime > latchOutDelay) {
				catapultState = catapultStates.PreparingToFire;
				lastEventTime = 0;
			}
			break;
		case PreparingToFire:
			if(lastEventTime == 0) { 
				lastEventTime = System.currentTimeMillis();
			}
			latchPneumatic.set(false); //out
			pusherPneumatic.set(true); //in
			if(System.currentTimeMillis() - lastEventTime > latchOutDelay) {
				catapultState = catapultStates.ReadyToFire;
				lastEventTime = 0;
			}
			break;
		case ReadyToFire:
			catapultState = catapultStates.NoBall;
			break;
		default:
			break;
	    }

		//Pickup State Machine
		switch(pickupState)
		{
		case BallAcquired:
			if(operatorStick.getRawButton(2)) {
				pickupState = pickupStates.TransferUp;
			}
			break;
		case TransferUp:
			transferPneumatic.set(false); //out
			pickupState = pickupStates.DropBall;
			break;
		case DropBall:
			rollerTopMotor.set(operatorStick.getRawAxis(3));
			rollerSideMotor.set(operatorStick.getRawAxis(3) / 2);  //Scaling for the side motors
			if(operatorStick.getRawButton(2)) {
				pickupState = pickupStates.TransferDown;
			}
			break;
		case TransferDown:
			transferPneumatic.set(true); //in
			pickupState = pickupStates.BallAcquired;
			break;
		default:
			break;
		}

		//MISSILE SWITCH OVERRIDE
		if(switchPanel.getRawButton(4) && operatorStick.getRawButton(1)) { //This is done so that if the missile switch is fired the driver can fire.  Even if it is a terrible,terrible idea
			catapultState = catapultStates.NoBall;
		}
		if(switchPanel.getRawButton(4) && operatorStick.getRawButton(2)) { //This is done so that if the missile switch is fired the driver can transfer.  Even if it is a terrible,terrible idea
			transferPneumatic.set(!transferPneumatic.get());
		}
		if(switchPanel.getRawButton(4)) {  //allows driver control as long as the IR switch is not triggered or if the missile switch is triggered.
			rollerTopMotor.set(operatorStick.getRawAxis(3));
			rollerSideMotor.set(operatorStick.getRawAxis(3) / 2); //Scaling for the side motors
		}
    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
    
    }
    
}
