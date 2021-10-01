
package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static frc.robot.Constants.getMatchTime;
import static frc.robot.RobotContainer.*;

public class Debug {

	private static final StringBuilder debugMessages = new StringBuilder();

	private static final StringBuilder taggedMessages = new StringBuilder();

	private static StringBuilder tagBuilder = new StringBuilder();

	private static String tags = "";

	private static int counter = 0;
	/**
	 * set printRate to 0 to disable debugging (in a different way to kDebugEnabled)
	 */
	private static int printRate = 100;

	public static void addDebugTags(String input) {
		tagBuilder.append("\r\n").append(input);
	}

	// these are small methods so we don't need to check if debug is enabled... right?

	public static void setDebugTags() {
		tags = tagBuilder.toString();
	}

	/* NETWORK TABLE STUFF FOR DASHBOARD:
	 * (Not being used rn)
	 */

//	private final NetworkTableInstance defaultInstance =
//			NetworkTableInstance.getDefault();
//	private final NetworkTable shuffleMain =
//			defaultInstance.getTable("SmartDashboard");
//	private final NetworkTable shuffleDebug =
//			defaultInstance.getTable("Debug");
//	private final NetworkTable shuffleGraphs =
//			defaultInstance.getTable("Graphs");
//	//use (NetworkTable).getEntry(key).setNumber(value);

	public static void debug(String message) {

		if (Robot.kDebugEnabled) {
			if (counter == 1) {
//				debugMessages.append("\r\n").append(message);
//				debugMessages.append("\n").append(message);
				debugMessages.append(message);
			}
		}

	}

	public static void debug(double message) {

		if (Robot.kDebugEnabled) {
			if (counter == 1) {
//				debugMessages.append("\r\n").append(message);
//				debugMessages.append("\n").append(message);
				debugMessages.append(message);
			}
		}

	}

	public static void debug(boolean message) {

		if (Robot.kDebugEnabled) {
			if (counter == 1) {
//				debugMessages.append("\r\n").append(message);
//				debugMessages.append("\n").append(message);
				debugMessages.append(message);
			}
		}

	}

	// methods using tags

	public static void debug(String message, String tag) {

		if (Robot.kDebugEnabled) {
			if (counter == 1 && tags.contains(tag)) {
//				taggedMessages.append("\r\n").append(tag).append(": ").append(message);
//				taggedMessages.append("\n").append(tag).append(": ").append(message);
				taggedMessages.append(tag).append(": ").append(message);
			}
		}

	}

	public static void debug(double message, String tag) {

		if (Robot.kDebugEnabled) {
			if (counter == 1 && tags.contains(tag)) {
//				taggedMessages.append("\r\n").append(tag).append(": ").append(message);
//				taggedMessages.append("\n").append(tag).append(": ").append(message);
				taggedMessages.append(tag).append(": ").append(message);
			}
		}

	}

	public static void debug(boolean message, String tag) {

		if (Robot.kDebugEnabled) {
			if (counter == 1 && tags.contains(tag)) {
//				taggedMessages.append("\r\n").append(tag).append(": ").append(message);
//				taggedMessages.append("\n").append(tag).append(": ").append(message);
				taggedMessages.append(tag).append(": ").append(message);
			}
		}

	}

	public static void dump() {

		if (Robot.kDebugEnabled) {
			if (counter == 1) {
				System.out.println(debugMessages.toString());
				System.out.println(taggedMessages.toString());
				debugMessages.setLength(0);
				taggedMessages.setLength(0);
			}
			counter++;
			if (counter >= printRate) {
				counter = 0;
			}
		}
	}

	public static void printDashboard() {

		SmartDashboard.putBoolean("Climb Solenoid State", kClimbSub.getClimberBrakeState());

		//System.out.println("current pipeline: " + kLimeLight.getPipeline());

		SmartDashboard.putBoolean(
				"Up To Speed",
				isShooterUpToSpeed()
		);

		SmartDashboard.putNumber("Gyro Yaw", kDriveTrain.getGyroHeading());

		SmartDashboard.putNumber("Match Time", getMatchTime());

		SmartDashboard.putNumber("ShooterSpeed", kShooterSub.getRawSpeed());
		SmartDashboard.putNumber("ShooterTarget", kShooterSub.getShooterSetSpeed());

		SmartDashboard.putNumber("TurretPosition", kTurretSub.getRawEncoder());
		SmartDashboard.putNumber("TurretSpeed", kTurretSub.getRawEncoderSped());

		SmartDashboard.putNumber("ClimbPosition", kClimbSub.getRawPosition());
		SmartDashboard.putNumber("ClimbSpeed", kClimbSub.getRawVelocity());

		SmartDashboard.putNumber("Shooter Modifier", kShooterSub.getShooterSpeedModifier());

		SmartDashboard.putBoolean("LimeLight Has Target", kLimeLight.hasTarget());

		SmartDashboard.putNumber("Pose X", kOdometry.getPoseMeters().getTranslation().getX());
		SmartDashboard.putNumber("Pose Y", kOdometry.getPoseMeters().getTranslation().getY());
		SmartDashboard.putNumber("Pose Theta", kOdometry.getPoseMeters().getRotation().getDegrees());

		SmartDashboard.putNumber("Left Encoder Pos", kDriveTrain.getLeftEncoderPos());
		SmartDashboard.putNumber("Right Encoder Pos", kDriveTrain.getRightEncoderPos());

		SmartDashboard.putBoolean("Boost Mode", kDriveTrain.getBoostMode());

	}

}
