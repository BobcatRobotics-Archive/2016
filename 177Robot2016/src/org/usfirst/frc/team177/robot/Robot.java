
package org.usfirst.frc.team177.robot;

import org.usfirst.frc.team177.auto.*;
import org.usfirst.frc.team177.lib.Locator;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team177.robot.Catapult.catapultStates;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
    final String doNothing = "Do Nothing";
    final String driveForwardTransferDown = "Drive Forward Transfer Down";
    final String driveForwardTransferUp = "Drive Forward Transfer Up";
    String autoSelected;
    SendableChooser chooser;
	    
    /**Motor constants**/
	private static final int MotorDriveRL = 3;//Rear Left 888
	private static final int MotorDriveFL = 2; //Front Left 888
	private static final int MotorDriveRR = 1; //Rear Right 888
	private static final int MotorDriveFR = 0; //Front Right 888
	
	private static final int MotorRollerTop = 4; //Top Roller 888
	private static final int MotorRollerSide = 5; //Side Roller 888
	
	private static final int MotorClimb = 6; //Climb Motor 888
	/**Initialize Victors**/
	Victor rearLeftMotor = new Victor(MotorDriveRL);
	Victor frontLeftMotor = new Victor(MotorDriveFL);
	    
	Victor rearRightMotor = new Victor(MotorDriveRR);
	Victor frontRightMotor = new Victor(MotorDriveFR); 
	
	Victor rollerTopMotor = new Victor(MotorRollerTop);
	Victor rollerSideMotor = new Victor(MotorRollerSide);
	
	Victor climbMotor = new Victor(MotorClimb);
	
	/**Joysticks**/    
	Joystick leftStick = new Joystick(0);
	Joystick rightStick = new Joystick(1);
	Joystick operatorStick = new Joystick(2);
	Joystick switchPanel = new Joystick(3);
	
	public RobotDrive drive = new RobotDrive(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor);
	
	/** Joystick Constants **/ //Magic Numbers found in Joystick.class
    private static final int axisY = 1;
    
    /** Solenoids **/
	//SAFETY: At the end of the match both the latch and the pusher should be out
	public Solenoid latchPneumatic = new Solenoid(1); //false = out
	public DoubleSolenoid pusherPneumatic = new DoubleSolenoid(4,5); //false = out
	public Solenoid transferPneumatic = new Solenoid(2); //false = out
	public Solenoid shiftPneumatic = new Solenoid(0);
	
    /** Digital Input **/
    //DigitalInput ballIRSwitch = new DigitalInput(); //RIP IR, died 2/11/16 at the hands of Ulf's SuperAwesome piece of Lexan 
    
    private static final int leftDriveEncoderA = 4;
    private static final int leftDriveEncoderB = 5;
    private static final int rightDriveEncoderA = 6;
    private static final int rightDriveEncoderB = 7;
    
    /** Analog Inputs **/
    private static final int GyroAnalogInput = 0;   
    
    
    enum pickupStates {
    	BallAcquired,
    	TransferUp,
    	DropBall,
    	TransferDown
    };
    
    //catapult;
    Catapult catapult;
    
    
    //State Machine Pickup
    pickupStates pickupState = pickupStates.BallAcquired;
   
    
    //Controller Mapping
    //Controller
    private static final int ButtonTransfer = 8;
    private static final int ButtonSideRollers = 7;
    //Right Joystick
    private static final int ButtonShift = 3;
    //Left Joystick
    
    
    /* Navigation functions */
    public Locator locator = new Locator(GyroAnalogInput, leftDriveEncoderA, leftDriveEncoderB,
    		rightDriveEncoderA, rightDriveEncoderB);
    
    /* Automode Variables */
    String autoMode = "";
    double autoDelay = 0;    
    AutoMode auto;
    long autoStartTime;
    
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
        chooser = new SendableChooser();
        chooser.addDefault("Do Nothing", doNothing);
        chooser.addObject("Drive Forward(LowBar)", driveForwardTransferDown);
        chooser.addObject("Drive Forward(Obstacle)", driveForwardTransferUp);
        SmartDashboard.putData("Auto choices", chooser);
        transferPneumatic.set(true);
        
        catapult = new Catapult(latchPneumatic, pusherPneumatic);
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
		if(auto != null) 
		{
            auto.autoInit();
        }						
		autoStartTime = System.currentTimeMillis();
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {
    	if(auto != null && (System.currentTimeMillis() - autoStartTime > autoDelay)) 
		{
            auto.autoPeriodic();        
        } else 
		{
            drive.tankDrive(0, 0);
        }    	
    	
    	switch(autoSelected) {
    	case driveForwardTransferDown:
    		auto = new AutoModeDriveForwardTransferDown(this);
            break;
    	case driveForwardTransferUp:
    		auto = new AutoModeDriveForwardTransferUp(this);
    		break;
    	case doNothing:
    	default:
    		//Do Nothing
            break;
    	}
    }
    
    public void disabledPeriodic() 
	{	
    	autoSelected = (String) chooser.getSelected();
		autoSelected = SmartDashboard.getString("Auto Selector", doNothing);		
		
		if(!autoSelected.equals(autoMode))
		{
			switch(autoSelected) {
	    		case driveForwardTransferDown:
	    			auto = new AutoModeDriveForwardTransferDown(this);
	    			break;
	    		case driveForwardTransferUp:
	    			auto = new AutoModeDriveForwardTransferUp(this);
	    			break;
	    		case doNothing:
	        	default:
	        		//Do Nothing
	        		auto = null;
	                break;        	
			}
			autoMode = autoSelected;
		}
		
		autoDelay = (switchPanel.getX() + 1.0f)*10.0f;  //-1 to 1 gives you a range 0 - 20
		SmartDashboard.putNumber("Auto Delay", autoDelay);
		SmartDashboard.putString("Auto Mode", autoMode);
	}
    
    @SuppressWarnings("unused")
	public void teleopInit() {
    	catapult.setState(catapultStates.BallsIn);
    	pickupStates pickupState = pickupStates.BallAcquired;
    }
    
    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
    	//Driving
    	double left = leftStick.getRawAxis(axisY);
		double right = rightStick.getRawAxis(axisY);
		drive.tankDrive(left, right);
		shiftPneumatic.set(rightStick.getRawButton(ButtonShift));
		
		SmartDashboard.putString("test", "yay");
		
    	transferPneumatic.set(operatorStick.getRawButton(ButtonSideRollers));
		if(operatorStick.getRawButton(ButtonTransfer)) {
			rollerSideMotor.set(1);
		}
		else {
			rollerSideMotor.set(0);
		}
		rollerTopMotor.set(operatorStick.getRawAxis(1) * -1); //Left Stick, y axis
	
		if(switchPanel.getRawButton(1))
		{
			latchPneumatic.set(switchPanel.getRawButton(2));
			pusherPneumatic.set(switchPanel.getRawButton(3)?DoubleSolenoid.Value.kForward:DoubleSolenoid.Value.kReverse);
			catapult.setState(catapultStates.BallsIn);			
		}
		else
		{
			catapult.loop(operatorStick.getRawButton(1));
		}
		
		
		//Pickup State Machine
		/**switch(pickupState)
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
		}**/

		//MISSILE SWITCH OVERRIDE
		if(switchPanel.getRawButton(4) && operatorStick.getRawButton(1)) { //This is done so that if the missile switch is fired the driver can fire.  Even if it is a terrible,terrible idea
			catapult.setState(catapultStates.NoBall);
		}		
    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
    
    }
    
}
