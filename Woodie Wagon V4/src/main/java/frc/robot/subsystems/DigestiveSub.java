
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Config;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.ControlMode;

import static frc.robot.Constants.*;
import static frc.robot.Debug.*;
import static frc.robot.RobotContainer.*;

public class DigestiveSub extends SubsystemBase {

	private final VictorSPX kAgitatorVictor;
	private final TalonSRX kTowerTalon;
	private final TalonSRX kEggBeaterTalon;

	private final double queueDelay = 0.25; // in seconds
	private double queueMemory = -queueDelay;
	private boolean queueStatus = false;

	/**
	 * Creates a new DigestiveSub.
	 */
	public DigestiveSub(VictorSPX kAVictor, TalonSRX kTTalon, TalonSRX kEBTalon) {
		debug("digestive subsystem initializing: " + this.toString());

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

//		debug("Beam Break A: " + kTowerBeamBreakA.get());
//		debug("Beam Break B: " + kTowerBeamBreakB.get());
		debug("Beam Break A: " + kTopBeamBreak.get(), "debug");
		debug("Beam Break B: " + kBottomBeamBreak.get(), "debug");

		// if operator doing stuff with digestive
		if (
				kOperator.getBumper(GenericHID.Hand.kLeft) // if priming
				|| kOperator.getBumper(GenericHID.Hand.kRight) // or if reverse
				|| kOperator.getTriggerAxis(GenericHID.Hand.kRight) >= Config.kDefaultDeadband // or if shooting
		) {

			// run prime if Left Operator bumper
			if (kOperator.getBumper(GenericHID.Hand.kLeft)) {
				primeOn();
			}

			// when firing, feed on. (definition of firing)
			if (kOperator.getTriggerAxis(GenericHID.Hand.kRight)
					>= Config.kDefaultDeadband) {
				feedOn();
			}

			// button above fire to reverse tower
			if (kOperator.getBumper(GenericHID.Hand.kRight)) {
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
					>= Config.kDefaultDeadband) {
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
						Config.kDefaultTowerPrimeFF
				);
//				kAgitatorVictor.set(
//						ControlMode.PercentOutput,
//						Config.kDefaultAgitatorFF
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
					Config.kDefaultAgitatorFF
			);
			kEggBeaterTalon.set(
					ControlMode.PercentOutput,
					-Config.kDefaultEggBeaterFF
			);
		} else { // if top limit switch triggers, forget everything and turn off
			feedOff();
		}
	}

	public void feedOn() {
		if (isShooterUpToSpeed()) {
			kTowerTalon.set(
					ControlMode.PercentOutput,
					Config.kDefaultTowerFeedFF
			);
			kAgitatorVictor.set(
					ControlMode.PercentOutput,
					Config.kDefaultAgitatorFF
			);
			kEggBeaterTalon.set(
					ControlMode.PercentOutput,
					-Config.kDefaultEggBeaterFF
			);
		} else {
			primeOn();
		}
	}

	private void reverseOn() {
		kTowerTalon.set(
			ControlMode.PercentOutput,
			-Config.kDefaultTowerPrimeFF
		);
	}

	public void feedOff() {
		kAgitatorVictor.set(ControlMode.PercentOutput, 0);
		kTowerTalon.set(ControlMode.PercentOutput, 0);
		kEggBeaterTalon.set(ControlMode.PercentOutput, 0);
	}
}
