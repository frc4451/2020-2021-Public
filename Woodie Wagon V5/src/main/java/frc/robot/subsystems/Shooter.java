package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;
import static frc.robot.Debug.*;
import static frc.robot.RobotContainer.*;
import static frc.robot.RobotMath.deadBand;
import static frc.robot.RobotMath.square;

public class Shooter extends SubsystemBase {

	private final TalonFX kShooterFlywheel;
	private final TalonSRX kShooterTurret;

	private double shooterSpeedTarget;
	
	public Shooter(TalonFX _kShooterFlywheel, TalonSRX _kShooterTurret) {

		this.kShooterFlywheel = _kShooterFlywheel;
		this.kShooterTurret = _kShooterTurret;

		shooterSpeedTarget = -1;
		
		// Turret Motor configuration
		
		kShooterTurret.configFactoryDefault();

		kShooterTurret.setInverted(true);

		kShooterTurret.configForwardLimitSwitchSource(
				LimitSwitchSource.FeedbackConnector,
				LimitSwitchNormal.NormallyClosed
		);
		kShooterTurret.configReverseLimitSwitchSource(
				LimitSwitchSource.FeedbackConnector,
				LimitSwitchNormal.NormallyClosed
		);

		kShooterTurret.config_kP(
				PIDs.kPIDLoopIdx,
				PIDs.kGainsTurret.kP,
				Config.DEF_TIMEOUT_MS
		);
		kShooterTurret.config_kI(
				PIDs.kPIDLoopIdx,
				PIDs.kGainsTurret.kI,
				Config.DEF_TIMEOUT_MS
		);
		kShooterTurret.config_kD(
				PIDs.kPIDLoopIdx,
				PIDs.kGainsTurret.kD,
				Config.DEF_TIMEOUT_MS
		);
		kShooterTurret.config_kF(
				PIDs.kPIDLoopIdx,
				PIDs.kGainsTurret.kF,
				Config.DEF_TIMEOUT_MS
		);

		// Shooter Motor configuration

		kShooterFlywheel.configFactoryDefault();

		kShooterFlywheel.setInverted(true);

		kShooterFlywheel.configSelectedFeedbackSensor(
				FeedbackDevice.IntegratedSensor,
				PIDs.kPIDLoopIdx,
				Config.DEF_TIMEOUT_MS);

		kShooterFlywheel.setSensorPhase(false);

		kShooterFlywheel.configNominalOutputForward(0, Config.DEF_TIMEOUT_MS);
		kShooterFlywheel.configNominalOutputReverse(0, Config.DEF_TIMEOUT_MS);
		kShooterFlywheel.configPeakOutputForward(1, Config.DEF_TIMEOUT_MS);
		kShooterFlywheel.configPeakOutputReverse(-1, Config.DEF_TIMEOUT_MS);

		kShooterFlywheel.setNeutralMode(NeutralMode.Coast);

		kShooterFlywheel.configClosedloopRamp(0);

		kShooterFlywheel.config_kF(PIDs.kPIDLoopIdx,
				PIDs.kGainsShooter.kF,
				Config.DEF_TIMEOUT_MS
		);
		kShooterFlywheel.config_kP(PIDs.kPIDLoopIdx,
				PIDs.kGainsShooter.kP,
				Config.DEF_TIMEOUT_MS
		);
		kShooterFlywheel.config_kI(PIDs.kPIDLoopIdx,
				PIDs.kGainsShooter.kI,
				Config.DEF_TIMEOUT_MS
		);
		kShooterFlywheel.config_kD(PIDs.kPIDLoopIdx,
				PIDs.kGainsShooter.kD,
				Config.DEF_TIMEOUT_MS
		);
	}

	public void enableInit() {
		resetTurretEncoder();
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}

	/* =================================================================================================================
	 * TURRET METHODS
	 */

	public void runTurret() {

		if (Subsystems.kLimeLight.getPipeline() == LimelightPipeline.NormalAim.val()) {
			if (Subsystems.kLimeLight.hasTarget()) {
				double targetX = Subsystems.kLimeLight.getTargetX();
//				double targetX = kLimelight.getXOffset() + aimOffsetX; // custom offset not being used
				turn(
						targetX/Settings.SCREEN_SIZE_X+Math.copySign(
								Settings.TURRET_TURN_FRICT_CONST,
								targetX
						)
				);
			} else {
				turn(0); // I don't know if this is necessary but this is safe
			}

		}

		if (Subsystems.kLimeLight.getPipeline() == LimelightPipeline.Default.val()) {
			double turretTurn = square(deadBand(
					IO.Operator.rightX(),
					Config.DEF_DEADBAND
			));
			turn(turretTurn);

		}

	}

	private void turn(double value) {
		final double kMaxSpeed = -1.0/2.0;
		turnRaw(value*kMaxSpeed);
	}

	private void turnRaw(double value) {
		kShooterTurret.set(ControlMode.PercentOutput, value);
	}

	/* =================================================================================================================
	 * SHOOTER METHODS
	 */

	public void runShooter() {


		debug("calcSpeed: " + calcSpeed(),"debug");

		// Low medium High configuration
		switch(IO.Operator.pov()) {
			case 0: //North
				setRamp(0);
				spinUp(10400); // long range: actual was 11950,
				break;
			case 90: // East: Auto trench section
				// linear interpolation between front and back of trench for now (ax+b)
				setRamp(0);
				spinUp(calcSpeed()); // mid range: actual was 9777
				break;
			case 270: // West
				setRamp(0);
				spinUp(9650); // mid range: actual was 9777
				break;
			case 180: //South
				setRamp(0);
				spinUp(8750); // short range
				break;
			// mid range again
			case -1: //not pressing
				spinUp(-1);
				break;
			default: //this means something is wrong
				spinUp(0);
				break;
		}

		// Testing for regression

//		if(IO.Operator.uPressed()) {
//			shooterSpeedTarget += 100;
//		}
//
//		if(IO.Operator.dPressed()) {
//			shooterSpeedTarget -= 100;
//		}
//
//		if (shooterSpeedTarget < 0) {
//			shooterSpeedTarget = 0;
//		}
//
//		if (shooterSpeedTarget > 12000) {
//			shooterSpeedTarget = 12000;
//		}
//
//		debug("limelight Y: " + Subsystems.kLimeLight.getTargetY(),"debug");
//		debug("shooter Speed Target: " + shooterSpeedTarget,"debug");

		if (IO.Operator.r()) {
			spinUp(shooterSpeedTarget);
		}


		/* Controller Haptic Feedback */

		// this is being derpy so leave it alone
//		 if (kShooterSub.getRawSpeed()
//		 		>= shooterSpeedTarget*Config.kUpToSpeedThreshold
//		 		&& shooterSpeedTarget != 0) {
//		 	kOperator.setRumble(RumbleType.kLeftRumble, 1);
//		 } else {
//		 	kOperator.setRumble(RumbleType.kLeftRumble, 0);
//		 }

	}

	public double calcSpeed() {

		// linear interpolation between front and back of trench for now (ax+b) TODO: Quadratic

		double a = -230.0613497; // slope
		double b = 9005.828221; // y intercept

		double x = Subsystems.kLimeLight.getTargetY(); // independent variable

		return(a * x + b);
	}

	public void spinUp(double target) {
		if (target == -1) {
			this.kShooterFlywheel.set(ControlMode.PercentOutput, 0);
			this.shooterSpeedTarget = -1;
		} else if (target == -2) { // TODO: -2 SpinUp will be distance and then shooter speed calculation
			// I don't think we will use the capability in auto that much because the distances are predictable.
			// In situations where we have to change auto configurations quickly we might use this.
		} else {
			this.shooterSpeedTarget = target;
			kShooterFlywheel.set(
					ControlMode.Velocity,
					shooterSpeedTarget
			);
		}
	}

	/* =================================================================================================================
	 * BASIC METHODS
	 */

	public double getRawTurretPos() {
		return kShooterTurret.getSelectedSensorPosition();
	}

	public double getRawTurretVel() {
		return kShooterTurret.getSelectedSensorVelocity();
	}

	private void resetTurretEncoder() {
		kShooterTurret.setSelectedSensorPosition(0);
	}

	public double getRawShooterSpeed() {
		return kShooterFlywheel.getSelectedSensorVelocity();
	}

	public double getShooterSpeedTarget() {
		return shooterSpeedTarget;
	}

	public boolean isShooterUpToSpeed() {
		return (getRawShooterSpeed() > shooterSpeedTarget * Settings.SHOOTER_UTS_THRESH);
	}

	public void setRamp(double value) {
		kShooterFlywheel.configClosedloopRamp(value);
	}

}
