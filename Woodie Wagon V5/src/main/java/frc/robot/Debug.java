
package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static frc.robot.Constants.Config.kDebugEnabled;
import static frc.robot.RobotContainer.*;

public class Debug {

	private static final StringBuilder debugMessages = new StringBuilder();

	private static final StringBuilder taggedMessages = new StringBuilder();

	private static final StringBuilder tagBuilder = new StringBuilder();

	private static String tags = "";

	private static int debugCounter = 0;
	/**
	 * set printRate to 0 to disable debugging (in a different way to kDebugEnabled)
	 */
	private static final int debugPrintRate = 100;

	private static int dashboardCounter = 0;
	private static final int dashboardPrintRate = 10;

	// every iteration is about 0.05 to 0.1 seconds (.5-1 deciseconds, 50-100 milliseconds)

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

		if (kDebugEnabled) {
			if (debugCounter == 1) {
//				debugMessages.append("\r\n").append(message);
//				debugMessages.append("\n").append(message);
				debugMessages.append(message);
			}
		}

	}

	public static void debug(double message) {

		if (kDebugEnabled) {
			if (debugCounter == 1) {
//				debugMessages.append("\r\n").append(message);
//				debugMessages.append("\n").append(message);
				debugMessages.append(message);
			}
		}

	}

	public static void debug(boolean message) {

		if (kDebugEnabled) {
			if (debugCounter == 1) {
//				debugMessages.append("\r\n").append(message);
//				debugMessages.append("\n").append(message);
				debugMessages.append(message);
			}
		}

	}

	// methods using tags

	public static void debug(String message, String tag) {

		if (kDebugEnabled) {
			if (debugCounter == 1 && tags.contains(tag)) {
//				taggedMessages.append("\r\n").append(tag).append(": ").append(message);
				taggedMessages.append("\n").append(tag).append(": ").append(message);
//				taggedMessages.append(tag).append(": ").append(message);
			}
		}

	}

	public static void debug(double message, String tag) {

		if (kDebugEnabled) {
			if (debugCounter == 1 && tags.contains(tag)) {
//				taggedMessages.append("\r\n").append(tag).append(": ").append(message);
//				taggedMessages.append("\n").append(tag).append(": ").append(message);
				taggedMessages.append(tag).append(": ").append(message);
			}
		}

	}

	public static void debug(boolean message, String tag) {

		if (kDebugEnabled) {
			if (debugCounter == 1 && tags.contains(tag)) {
//				taggedMessages.append("\r\n").append(tag).append(": ").append(message);
//				taggedMessages.append("\n").append(tag).append(": ").append(message);
				taggedMessages.append(tag).append(": ").append(message);
			}
		}

	}

	public static void dump() {

		if (kDebugEnabled) {
			if (debugCounter == 1) {
				if(!debugMessages.toString().isBlank() || !taggedMessages.toString().isBlank()) {
					System.out.println(debugMessages.toString());
					System.out.println(taggedMessages.toString());
					debugMessages.setLength(0);
					taggedMessages.setLength(0);
				}
			}
			debugCounter++;
			if (debugCounter >= debugPrintRate) {
				debugCounter = 0;
			}
		}
	}

	public static void printDashboard() {

		if (debugCounter == 1) {



			SmartDashboard.putBoolean("Climb Solenoid State", Subsystems.kClimber.getClimberBrakeState());

			SmartDashboard.putBoolean(
					"Up To Speed",
					Subsystems.kShooterHead.isShooterUpToSpeed()
			);

			SmartDashboard.putNumber("Match Time", RobotMath.getMatchTime());

			SmartDashboard.putNumber("ShooterSpeed", Subsystems.kShooterHead.getRawShooterSpeed()); // Shooter
			SmartDashboard.putNumber("ShooterTarget", Subsystems.kShooterHead.getShooterSpeedTarget()); // Shooter

			SmartDashboard.putNumber("Calculated Speed", Subsystems.kShooterHead.calcSpeed()); // Shooter

//			SmartDashboard.putNumber("TurretPosition", Subsystems.kShooterHead.getRawEncoder()); // Shooter
//			SmartDashboard.putNumber("TurretSpeed", Subsystems.kShooterHead.getRawEncoderSped()); // Shooter
			SmartDashboard.putNumber("TurretPosition", -69420); // Shooter
			SmartDashboard.putNumber("TurretSpeed", -69420); // Shooter

			SmartDashboard.putNumber("ClimbPosition", Subsystems.kClimber.getRawPosition());
			SmartDashboard.putNumber("ClimbSpeed", Subsystems.kClimber.getRawVelocity());

			SmartDashboard.putBoolean("LimeLight Has Target", Subsystems.kLimeLight.hasTarget());

			SmartDashboard.putNumber("Pose X", Subsystems.kDriveTrain.getPose().getTranslation().getX());
			SmartDashboard.putNumber("Pose Y", Subsystems.kDriveTrain.getPose().getTranslation().getY());
			SmartDashboard.putNumber("Pose Theta", Subsystems.kDriveTrain.getPose().getRotation().getDegrees());

			SmartDashboard.putNumber("Gyro Yaw", Subsystems.kDriveTrain.getGyroYawRaw());

			SmartDashboard.putNumber(
					"Internal Gyro Yaw",
					Subsystems.kDriveTrain.getGyroRotation2d().getDegrees()
			);

			if (Robot.activeAuto != null) {
				SmartDashboard.putNumber("Initial X", Robot.activeAuto.getCurrentInitialPose().getX());
				SmartDashboard.putNumber("Initial Y", Robot.activeAuto.getCurrentInitialPose().getY());
				SmartDashboard.putNumber("Initial D", Robot.activeAuto.getCurrentInitialPose().getRotation().getDegrees());

				SmartDashboard.putNumber("Target X", Robot.activeAuto.getCurrentTargetPose().getX());
				SmartDashboard.putNumber("Target Y", Robot.activeAuto.getCurrentTargetPose().getY());
				SmartDashboard.putNumber("Target D", Robot.activeAuto.getCurrentTargetPose().getRotation().getDegrees());

				// calculated track time of 1st 4 stages of the selected trajectory
				SmartDashboard.putNumber("Auto Stage 1 time", Robot.activeAuto.getCalcTrackTime(1));
				SmartDashboard.putNumber("Auto Stage 2 time", Robot.activeAuto.getCalcTrackTime(2));
				SmartDashboard.putNumber("Auto Stage 3 time", Robot.activeAuto.getCalcTrackTime(3));
				SmartDashboard.putNumber("Auto Stage 4 time", Robot.activeAuto.getCalcTrackTime(4));
			}

			SmartDashboard.putNumber("Left Encoder Pos", Subsystems.kDriveTrain.getLeftEncoderPos());
			SmartDashboard.putNumber("Right Encoder Pos", Subsystems.kDriveTrain.getRightEncoderPos());

			SmartDashboard.putString("DEV Auto Chooser", Robot.chooser.getSelected());

			SmartDashboard.putString("Game State", Robot.getGameState());

			SmartDashboard.putBoolean("Bottom Limit Switch", Subsystems.kDigestiveSystem.getBottomLimitSwitch());
			SmartDashboard.putBoolean("Top Limit Switch", Subsystems.kDigestiveSystem.getTopLimitSwitch());

		}
		debugCounter++;
		if (debugCounter >= debugPrintRate) {
			debugCounter = 0;
		}

	}

}
