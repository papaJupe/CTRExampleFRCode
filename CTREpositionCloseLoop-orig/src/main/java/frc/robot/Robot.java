/* sourceType:  CTRE vendor example illustrating using their Smart PID control software
*   built in to their smart motor controllers, SRX and SPX, Relative and Absolute encoders
* sourceStatus: not tested, appears to be usable as is for positioning one motor's rotation to a
* position set by joystick angle using one button press, and continuous position control by joystick
* while holding a different button. Uses (requires) both Relative (Quad) and Absolute encoder inputs
* to the motor controller. 
* useContext: one brushed motor, CAN bus connected CTRE motor controller, remote absolute
* position encoder wired to Talon, standard FRC control envir. RIO, Driver Station et.al. 
*/


/**
 * Description of this example:
 * PositionClosedLoop example demonstrates closed-loop servo-like position control
 * Tested with Logitech F350 USB Gamepad inserted into Driver Station
 * 
 * Be sure to select the correct feedback sensor using configSelectedFeedbackSensor() below.
 * Use Percent Output Mode (Holding A and using Left Joystick) to confirm talon is driving 
 * forward (Green LED on Talon/Victor) when the position sensor is moving in the positive 
 * direction. If this is not the case, invert the boolean input in setSensorPhase().
 * 
 * Controls:
 * Button 1: When pressed, start and run Position Closed Loop on Talon/Victor motor controller
 * Button 2: When held, start and run motor in Percent Output mode
 * Left Joystick Y-Axis:
 * 	+ Position Closed Loop: S signals Talon forward and reverse [-10, 10] rotations
 * 	+ Percent Ouput: Throttle Talon forward and reverse
 * 
 * Gains for Position Closed Loop may need to be adjusted in Constants.java
 * 
 * Supported motor controllers for this Version:
 * - Talon SRX: 4.00  ? if Falcon's built-in encoders are included
 * - Victor SPX: 4.00
 * - Pigeon IMU: 4.00
 * - CANifier: 4.00
 */

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Joystick;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.*;

public class Robot extends TimedRobot {
    /** Hardware */
	WPI_TalonSRX _talon = new WPI_TalonSRX(2);
	Joystick _joy = new Joystick(0);
	
    /** Used to create string thoughout loop */
	StringBuilder _sb = new StringBuilder();
	int _loops = 0;
	
    /** Track button state for single press event */
	boolean _lastButton1 = false;

	/** Save the target position to servo to */
	double targetPositionRotations;


	public void robotInit() {
		/* Factory Default all hardware to prevent unexpected behaviour */
		_talon.configFactoryDefault();
		
		/* Config Quad sensor used for Primary PID and sensor direction */
        _talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 
                                            Constants.kPIDLoopIdx,
				                            Constants.kTimeoutMs);

		/* Ensure sensor is positive when output is positive */
		_talon.setSensorPhase(Constants.kSensorPhase);

		/**
		 * Set based on what direction you want forward/positive to be.
		 * This does not affect sensor phase which governs polarity inversion
             * (true), or not inverted (false) between sensor and motor direction.
		 */ 
		_talon.setInverted(Constants.kMotorInvert);

		/* Config the peak and nominal (i.e. 0) output,  1 means full 12V */
		_talon.configNominalOutputForward(0, Constants.kTimeoutMs);
		_talon.configNominalOutputReverse(0, Constants.kTimeoutMs);
		_talon.configPeakOutputForward(1, Constants.kTimeoutMs);
		_talon.configPeakOutputReverse(-1, Constants.kTimeoutMs);

		/**
		 * Config the allowable closed-loop error, Closed-Loop output will be
		 * neutral within this range. See Table in Section 17.2.1 for native
		 * units per rotation.
		 */
		_talon.configAllowableClosedloopError(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);

		/* Config Position Closed Loop gains in slot0, typically kF stays zero. */
		_talon.config_kF(Constants.kPIDLoopIdx, Constants.kGains.kF, Constants.kTimeoutMs);
		_talon.config_kP(Constants.kPIDLoopIdx, Constants.kGains.kP, Constants.kTimeoutMs);
		_talon.config_kI(Constants.kPIDLoopIdx, Constants.kGains.kI, Constants.kTimeoutMs);
		_talon.config_kD(Constants.kPIDLoopIdx, Constants.kGains.kD, Constants.kTimeoutMs);

		/**
		 * read the 360 degree position of the MagEncoder's absolute
		 * position, and initially set the relative sensor to match.
		 */
		int absolutePosition = _talon.getSensorCollection().getPulseWidthPosition();

		/* Mask out overflows, keep bottom 12 bits */
		absolutePosition &= 0xFFF;       
             /* each char 0-F in this binary format appears to hold 4 bits (16 values)*/

		if (Constants.kSensorPhase) { absolutePosition *= -1; }
		if (Constants.kMotorInvert) { absolutePosition *= -1; }
		
		/* Set the quadrature (relative) sensor to match absolute encod value */
		_talon.setSelectedSensorPosition(absolutePosition, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
    } // end rI
    
	void commonLoop() { method to be called in tP (teleOp periodic)
		/* Gamepad input processing -- button press assigns a value to the button* variable */
		double leftYstick = _joy.getY();
		boolean button1 = _joy.getRawButton(1);	// X-Button
		boolean button2 = _joy.getRawButton(2);	// A-Button

		/* Get Talon/Victor's current output percentage */
		double motorOutput = _talon.getMotorOutputPercent();

		/* Deadband gamepad */
		if (Math.abs(leftYstick) < 0.10) {
			/* Within 10% of zero */
			leftYstick = 0;
		}// end if 

		/* Prepare line to print value being output to motor*/
		_sb.append("\tout:");
		/* Cast to int to remove decimal places */
		_sb.append((int) (motorOutput * 100));
		_sb.append("%");	// Percent

		_sb.append("\tpos:");
		_sb.append(_talon.getSelectedSensorPosition(0));
		_sb.append("u"); 	// Native units

		/**
		 * When button 1 is pressed, perform Position Closed Loop to selected position
		 * indicated by Joystick position x10, [-10, 10] rotations
		 */
		if (!_lastButton1 && button1) { // detects a new button press
			/* Position Closed Loop method here only requires one activation
                and appears to have an internal isFinished method to stop when Pos reached */

			/* 10 Rotations * 4096 unit/rev in either direction */
			targetPositionRotations = leftYstick * 10.0 * 4096;
			_talon.set(ControlMode.Position, targetPositionRotations);
		} // end if

		/* When button 2 is held, just straight drive */
		if (button2) {  // active only when held
			/* Percent Output mode*/

			_talon.set(ControlMode.PercentOutput, leftYstick);
		}

		/* If Talon is in position closed-loop, print this info */
		if (_talon.getControlMode() == ControlMode.Position) {
			/* append more signals to print when in Pos. mode */
			_sb.append("\terr:");  // this will print "err:[int]u"
			_sb.append(_talon.getClosedLoopError(0));
			_sb.append("u");	// Native Units, probably an int

			_sb.append("\ttrg:");
			_sb.append(targetPositionRotations);
			_sb.append("u");	/// Native Units
		}

		/**
		 * Print every ten loops, printing too much too fast is bad
		 * for performance and operator sanity.
		 */
		if (++_loops >= 10) {
			_loops = 0;
			System.out.println(_sb.toString());
		}

		/* Reset (empty) existing string for next loop */
		_sb.setLength(0);
		
		/* Save button state to detect a new press */
		_lastButton1 = button1;
    } // end cL
    
	/**
	 * This function is called periodically during operator control, i.e. when
	* operator has Enabled robot and is using gamepad for control
	 */
	public void teleopPeriodic() {
		commonLoop();
	}  // end tP

} // end robot.j class
