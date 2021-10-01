
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMath;

import static frc.robot.RobotContainer.*;
import static frc.robot.RobotMath.*;
import static frc.robot.RobotMath.getTime;
import static frc.robot.RobotMath.getTimeSince;

public class DigestiveSub extends SubsystemBase {

	private final VictorSPX kAgitatorVictor;
	private final TalonSRX kEggBeaterTalon;
	private final TalonSRX kTowerTalon;

	private final double queueDelay = 0.25; // in seconds
	private double queueMemory = -queueDelay;
	private boolean queueStatus = false;

	/**
	 * Creates a new DigestiveSub.
	 */
	public DigestiveSub(VictorSPX kAVictor, TalonSRX kTTalon, TalonSRX kEBTalon) {

		this.kAgitatorVictor = kAVictor;
		this.kTowerTalon = kTTalon;
		this.kEggBeaterTalon = kEBTalon;

		kAgitatorVictor.configFactoryDefault();
		kTowerTalon.configFactoryDefault();

		kAgitatorVictor.setInverted(true);
		kTowerTalon.setInverted(true);

	}


	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}

	public void enableInit() {

	}

	public void runDigestive() {

		if (kOperator.getBumper(GenericHID.Hand.kLeft) // if operator doing stuff with digestive such as shooting
				|| kOperator.getBumper(GenericHID.Hand.kRight) // reverse
				|| kOperator.getTriggerAxis(GenericHID.Hand.kRight) //
				>= Constants.kDefaultDeadband) {

			// If Left Operator bumper, run prime,
			if (kOperator.getBumper(GenericHID.Hand.kLeft)) { // but if operator right trigger,
				primeOn();
			}

			if (kOperator.getTriggerAxis(GenericHID.Hand.kRight) // when firing, feed on
					>= RobotMath.kDefaultDeadband) {
				feedOn();
			}

			if (kOperator.getBumper(GenericHID.Hand.kRight)) { // button above fire to reverse tower
				reverseOn();
			}
		}
		else { // if operator not doing anything:
			/*
			 * Intake extension and retraction with Driver right trigger
			 * and priming of tower
			 * If the Drivers right trigger, extend the intake
			 * and run agitator and tower unless the tower limit switch triggers.
			 */
			if (kDriver.getTriggerAxis(GenericHID.Hand.kRight)
					>= Constants.kDefaultDeadband) {
				primeOn();
			}
			else {
				feedOff();
			}
		}

	}

	public void primeOn() {
		// .get() returns true if no ball is blocking it.
		if (kTopBeamBreak.get()) { // continue unless limit switch triggered
			if (!kBottomBeamBreak.get()) { // if bottom sees a ball, turn on
				queueStatus = true;
				kTowerTalon.set(
						ControlMode.PercentOutput,
						Constants.kDefaultTowerPrimeFF
				);
//				kAgitatorVictor.set(
//						ControlMode.PercentOutput,
//						Constants.kDefaultAgitatorFF
//				); // turn priming on
			} else {
				if (queueStatus) { // if first time off, set memory
					queueStatus = false;
					queueMemory = getTime();
				} else if (getTimeSince(queueMemory) > queueDelay) {
					// else, if more than queueDelay seconds after, turn off
					feedOff(); // turn priming and feeding off
				}

			}
			kAgitatorVictor.set(
					ControlMode.PercentOutput,
					Constants.kDefaultAgitatorFF
			);
			kEggBeaterTalon.set(
					ControlMode.PercentOutput,
					-Constants.kDefaultEggBeaterFF
			);
		} else { // if top limit switch triggers, forget everything and turn off
			feedOff();
		}

	}

	public void feedOn() {
		kTowerTalon.set(
				ControlMode.PercentOutput,
				Constants.kDefaultTowerFeedFF
		);
		kAgitatorVictor.set(
				ControlMode.PercentOutput,
				Constants.kDefaultAgitatorFF
		);
		kEggBeaterTalon.set(
				ControlMode.PercentOutput,
				-Constants.kDefaultEggBeaterFF
		);
	}

	private void reverseOn() {
		kTowerTalon.set(
			ControlMode.PercentOutput,
			-Constants.kDefaultTowerPrimeFF
		);
	}

	public void feedOff() {
		kAgitatorVictor.set(ControlMode.PercentOutput, 0);
		kTowerTalon.set(ControlMode.PercentOutput, 0);
		kEggBeaterTalon.set(ControlMode.PercentOutput, 0);
	}
}
