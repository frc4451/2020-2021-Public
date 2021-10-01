
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.ControlMode;

import static frc.robot.Constants.*;

import static frc.robot.Debug.*;
import static frc.robot.RobotContainer.*;

public class IntakeSub extends SubsystemBase {

	//memory for the game time the intake retracts at
	private final double intakeMotorOffDelay = 0.5; // in seconds
	private double retractMemory = -intakeMotorOffDelay;
	private boolean intakeState = false; // true is extended
	private boolean toggleMode = false; // for manual intake toggles

	private final VictorSPX kIntakeVictor;
	private final DoubleSolenoid kIntakeSolenoid;

	/**
	 * Creates a new IntakeSub.
	 */
	public IntakeSub(VictorSPX kIVictor, DoubleSolenoid kISolenoid) {
		debug("intake subsystem initializing: " + this.toString());

		this.kIntakeVictor = kIVictor;
		this.kIntakeSolenoid = kISolenoid;

		kIntakeVictor.configFactoryDefault();
		kIntakeVictor.setInverted(true);

		kIntakeSolenoid.set(Value.kReverse);
	
	}

	@Override
	public void periodic() {
	 // This method will be called once per scheduler run
		if (
				(intakeState && !toggleMode)
			|| getTimeSince(retractMemory)
				< intakeMotorOffDelay
		) {
			kIntakeVictor.set(
				ControlMode.PercentOutput,
				Config.kDefaultIntakeFF
			);
		} else {
			kIntakeVictor.set(ControlMode.PercentOutput, 0);
		}
	}

	public void enableInit() {

	}

	public void runIntake() {

		if (!(kOperator.getBumper(GenericHID.Hand.kLeft)
				|| kOperator.getBumper(GenericHID.Hand.kRight)
				|| kOperator.getTriggerAxis(GenericHID.Hand.kRight)
				>= Config.kDefaultDeadband)) {
			/*
			 * Intake extension and retraction with Driver right trigger
			 * (and priming of tower)
			 * If the Drivers right trigger, extend the intake
			 * (and run agitator and tower unless the tower limit switch triggers.)
			 */
			if (kDriver.getTriggerAxis(GenericHID.Hand.kRight)
					>= Config.kDefaultDeadband) {
				toggleMode = false;
				intakeExtend();
			}
			else {
				if (!toggleMode) {
					intakeRetract();
				}

				if (kDriver.getBackButton()) { // the left one
					toggleMode = true;
					intakeRetract();
				}
				if (kDriver.getStartButton()) { // the right one
					toggleMode = true;
					intakeExtend();
				}


			}
		}

	}

	/**
	 * Extends intake and turns intake motor on.
	 */
	public void intakeExtend() {
		kIntakeSolenoid.set(Value.kForward);
		intakeState = true;
	}

	/**
	 * Retracts intake and turns intake motor off.
	 */
	public void intakeRetract() {
		kIntakeSolenoid.set(Value.kReverse);
		if (intakeState) {
			retractMemory = getTime();
		}
		intakeState = false;
	}
}
