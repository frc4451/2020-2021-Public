package frc.robot.sensors;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimelightValue;

import static frc.robot.Debug.debug;
import static frc.robot.RobotContainer.kDriver;

public class LimelightSub extends SubsystemBase {

	NetworkTable table;

	private int _pipeline = LimelightValue.Default.val();

	/**
	 * Creates a new LimelightSubsystem.
	 */
	public LimelightSub() {

		table = NetworkTableInstance.getDefault().getTable("limelight");

		setPipeDefault();
		
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}

	public void enableInit() {
		setPipeDefault();
	}

	public void runLimelight() {
		// Markov diagram for following code https://i.ibb.co/4WqdxzZ/image.png

//		if (kDriver.getAButtonPressed()) {
//			if (_pipeline == LimelightValue.Short.val()) {
//				debug("The pipeline is DEFAULT");
//				setPipeDefault();
//			} else {
//				debug("The pipeline is SHORT");
//				setPipeShort();
//			}
//		} else if (kDriver.getYButtonPressed()) {
//			if (_pipeline == LimelightValue.Long.val()) {
//				debug("The pipeline is DEFAULT");
//				setPipeDefault();
//			} else {
//				debug("The pipeline is LONG");
//				setPipeLong();
//			}
//		}
//		if (kDriver.getXButtonPressed()) {
//			debug("The pipeline is DEFAULT");
//			setPipeDefault();
//		}

		setPipeNewAim();
	}

	/**
	 * 
	 * @return Returns if the limelight is currently detecting a target.
	 */
	public boolean hasTarget() {
		return (table.getEntry("tv").getDouble(0) >= 1);
	}

	/**
	 * 
	 * @return Returns the difference (from ~ -28 to 28) between the x position
	 * of the limelights crosshair and the x position of the midpoint of the
	 * target.
	 */
	public double getXOffset() {
		return table.getEntry("tx").getDouble(0);
	}

	/**
	 * 
	 * @return Returns the difference (from ~ -24 to 24) between the y position
	 * of the limelights crosshair and the y position of the midpoint of the
	 * target.
	 */
	public double getYOffset() {
		return table.getEntry("ty").getDouble(0);
	}

	private void setPipeline(int value) {
		table.getEntry("pipeline").setNumber(value);
	}

	public void setPipeShort() {
		this._pipeline = LimelightValue.Short.val();
		this.setPipeline(this._pipeline);
	}

	public void setPipeMedium() {
		this._pipeline = LimelightValue.Medium.val();
		this.setPipeline(this._pipeline);
	}

	public void setPipeLong() {
		this._pipeline = LimelightValue.Long.val();
		this.setPipeline(this._pipeline);
	}

	public void setPipeDefault() {
		this._pipeline = LimelightValue.Default.val();
		this.setPipeline(this._pipeline);
	}

	public void setPipeNewAim() {
		this._pipeline = 9;
		this.setPipeline(this._pipeline);
	}

	public int getPipeline() {
		return this._pipeline;
	}
}
