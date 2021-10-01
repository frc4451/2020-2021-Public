package frc.robot;

import edu.wpi.first.wpilibj.geometry.Pose2d;

/**
 * This class is for representing entire autonomous routines.
 * A track means an entire path of driving during an autonomous mode.
 * A track is made up of 1 or more trajectories that are represented as a ramsete path.
 * This class is made to encapsulate these trajectories along with other configurations for autonomous,
 * and also to facilitate the running of the autonomous routine during the autonomous period.
 */
public class AutoRoutine {

	// instance variables

	private AutoStage[] stages;

	AutoStage activeStage;

	private int stageNumber = -1; // -1 means not active
	public int numberOfStages;

	private boolean isReferenceFrameSetYet = false; // internal variable
	private boolean doContinuousReferenceFrame = false;
	// this is for if we don't want to reset odometry for each new track.
	// (ONLY DO THIS FOR CONTINUOUSLY CONNECTED TRACKS)

	// accessible and signal variables default
	public double driveForward = 0;
	public double driveTurn = 0;
	public boolean intake = false;
	public boolean digest = false;
	public boolean aim = false;
	public double spinUp = -1;
	public boolean shoot = false;
	public Pose2d resetPose = new Pose2d();
	public boolean resetSignal = false;

	// constructor
	public AutoRoutine(boolean _doContinuousReferenceFrame, AutoStage[] _stages) {
		this.doContinuousReferenceFrame = _doContinuousReferenceFrame;
		this.stages = _stages;
		this.numberOfStages = this.stages.length;
		System.out.println("AutoRoutine" + this.toString() + "made with " + this.numberOfStages + "stages");

	}

	public boolean iterate() {

		// variable to return status of iteration; if the routine is continuing to run or not.
		boolean returns = false; // for the time being

		if (this.numberOfStages != 0 && stageNumber != -3) { // conditions that mean to actually iterate



			// iteration variables

			this.resetSignal = false;
			returns = true; // for the time being
			// iterationMode is used as a condensed condition of what stage we are on
			int iterationMode = this.stageNumber; // this is only to start off (before condensing)

			// -1: not yet started on first stage (built in to stageNumber but I have it for the else)
			// -2: on last stage, if this stage ends then routine stops
			// -3: already dead, or out of bounds, stage is less than -1 or more than final stage number (length - 1)

			// checked backwards :p
			if (this.stageNumber >= this.numberOfStages || this.stageNumber < -2) {
				iterationMode = -3; // kill: means stage number is out of bounds fsr.
			} else {
				if (this.stageNumber == this.numberOfStages - 1) {
					iterationMode = -2; // on last stage, if this stage ends then routine stops
				} else if (this.stageNumber == -1) {
					this.stageNumber = 0; // -1++
					iterationMode = -1; // routine hasn't started yet, set stage to 1
					// this ^ line is redundant, but for OCD (-3, -2, and -1)
				}
				// at this point we should be left with a stageNumber that is in list bounds
				activeStage = this.stages[stageNumber];
			}

			switch(iterationMode) {

				case -1: // routine hasn't started yet, initialize first stage
					this.resetPose = activeStage.startStage();

					if (!this.doContinuousReferenceFrame || !this.isReferenceFrameSetYet) {
						this.resetSignal = true;
						// sends a signal to reset the odometry to the first pose of the first track
					}

					if (!activeStage.isTrackNull()) {
						this.isReferenceFrameSetYet = true; // so that future stages will not reset odometry
					}
					break;

				case -2: // on last stage, if this stage ends then routine stops
					if (activeStage.killConditionsCheck() || activeStage.endConditionsCheck()) {
						// (end conditions become kill conditions)

						this.stageNumber = -3; // this kills the process
						returns = false;
						// set all intentions to default
						this.driveForward = 0;
						this.driveTurn = 0;
						this.intake = true;
						this.digest = false;
						this.aim = false;
						this.spinUp = -1;
						this.shoot = false;

					} else { // if no conditions are met, go along with the iteration normally

						// Drivetrain

						if (activeStage.isTrackNull()) {
							this.driveForward = 0;
							this.driveTurn = 0;
						} else {
							// Get the desired pose from the trajectory.
							var desiredPose = activeStage.getTrackPose();

							// Get the reference chassis speeds from the Ramsete controller.
							var refChassisSpeeds = RobotContainer.ramseteCalculate(desiredPose);

							// Set the linear and angular speeds.
							this.driveForward = refChassisSpeeds.vxMetersPerSecond;
							this.driveTurn = refChassisSpeeds.omegaRadiansPerSecond;
						}

						// Intake and digestion
						this.intake = activeStage.intake;
						this.digest = activeStage.digest;

						// Turret and Shooter
						this.aim = activeStage.aim;
						this.spinUp = activeStage.spinUp;
						this.shoot = activeStage.shoot;

					}

					break;

				case -3: // auto kill: means stage number is out of bounds fsr.
					this.stageNumber = -3; // this kills the routine
					returns = false;
					// set all intentions to default
					this.driveForward = 0;
					this.driveTurn = 0;
					this.intake = true;
					this.digest = false;
					this.aim = false;
					this.spinUp = -1;
					this.shoot = false;
					break;

				default: // means we are in a normal stage that is not the last one

					// check for ending conditions and kill switch
					if (activeStage.killConditionsCheck()) {

						this.stageNumber = -3; // this kills the process
						returns = false;
						// set all intentions to default
						this.driveForward = 0;
						this.driveTurn = 0;
						this.intake = true;
						this.digest = false;
						this.aim = false;
						this.spinUp = -1;
						this.shoot = false;

					} else if (activeStage.endConditionsCheck()) {

						AutoStage nextStage = this.stages[stageNumber+1];

						this.resetPose = nextStage.startStage();

						if (!this.doContinuousReferenceFrame || !this.isReferenceFrameSetYet) {
							this.resetSignal = true;
							// sends a signal to reset the odometry to the first pose of the first track
						}

						if (!nextStage.isTrackNull()) {
							this.isReferenceFrameSetYet = true; // so that future stages will not reset odometry
						}

						this.stageNumber++;

					} else { // if no conditions are met, go along with the iteration normally

						// Drivetrain

						if (activeStage.isTrackNull()) {
							this.driveForward = 0;
							this.driveTurn = 0;
						} else {
							// Get the desired pose from the trajectory.
							var desiredPose = activeStage.getTrackPose();

							// Get the reference chassis speeds from the Ramsete controller.
							var refChassisSpeeds = RobotContainer.ramseteCalculate(desiredPose);

							// Set the linear and angular speeds.
							this.driveForward = refChassisSpeeds.vxMetersPerSecond;
							this.driveTurn = refChassisSpeeds.omegaRadiansPerSecond;
						}

						// Intake and digestion
						this.intake = activeStage.intake;
						this.digest = activeStage.digest;

						// Turret and Shooter
						this.aim = activeStage.aim;
						this.spinUp = activeStage.spinUp;
						this.shoot = activeStage.shoot;

					}

					break;
			}

		}
		return(returns);
	}

	public void reset() { // this might not be used
		this.stageNumber = -1;
	}

	/* =================================================================================================================
	 * WRAPPER GET METHODS
	 */

	public double getCalcTrackTime(int _stage) {
		if (_stage < numberOfStages) {
			AutoStage _stageObject = this.stages[_stage];
			if (_stageObject.isTrackNull()) {
				return(-1); // doesnt have a trajectory
			} else {
				return(_stageObject.trackDuration);
			}
		} else {
			return(-2); //not a stage
		}

	}

	public Pose2d getCurrentInitialPose() {
		return(this.resetPose);
	}

	public Pose2d getCurrentTargetPose() {
		return(this.activeStage.getTrackPose().poseMeters);
	}



}

