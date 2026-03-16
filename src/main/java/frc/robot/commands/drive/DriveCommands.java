package frc.robot.commands.drive;

import com.bobcats.lib.container.Container;
import com.bobcats.lib.container.IntegerContainer;
import com.bobcats.lib.subsystem.vision.LibVisionSubsystem;
import com.bobcats.lib.utils.PoseUtils;
import com.bobcats.lib.utils.VisionCalibration;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.RobotContainer;
import frc.robot.constants.OIConstants;
import frc.robot.subsystems.superstructure.Superstructure.ActionState;
import frc.robot.subsystems.swerve.SwerveConstants.DriveConstants;
import frc.robot.subsystems.swerve.SwerveConstants.ModuleConstants;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import java.util.function.DoubleSupplier;

//
// First few methods' credits go to Team 6328.
//

/**
 * A class containing comamnds related to the drivetrain, localization and characterization.
 */
public class DriveCommands {
	// The constants
	private static final double kFFStartDelay = 2.0; // in seconds
	private static final double kFFVoltageRampRate = 0.1; // in V/s

	private static final double kWheelRadRampRate = 0.05; // in rad/s^2
	private static final double kWheelRadMaxVel = 0.25; // in rad/s

	private static final double kSkewTargetLinearSpeed = 2.0; // in m/s
	private static final double kSkewAngularRampRate = 1.5; // in rad/s^2
	private static final double kSkewMaxAngularSpeed = 3.14; // in rad/s
	private static final double kSkewMinSpeed = 0.5; // in m/s
	private static final double kSkewMinAngularSpeed = 0.7; // in rad/s

	private static final int kVisionCalSampleCountPerCam = 15 * 50; // 15 secs, 50Hz
	private static final double kVisionCalStartDelay = 10; // in seconds

	private static final double kWheelCOFSlideThreshold = 0.1; // in m/s
	private static final int kWheelCOFBacktrackSamples = 50 / 4; // About 0.25 seconds, ~12 samples.
	private static final double kWheelCOFStartDelay = 5; // in seconds
	private static final int kWheelCOFMaxPitchBufferSize = 100; // About 2 seconds

	public static double LatestShootingAngularVelocity = 0;

	/** No constructor for static utilities. */
	private DriveCommands() {}

	/**
	 * Computes a linear velocity from the joystick inputs.
	 *
	 * @param x The input along the X axis.
	 * @param y The input along the Y axis.
	 * @return The linear velocity.
	 */
	private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
		// Apply deadband
		double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), OIConstants.kDriverControllerDeadband);
		Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

		// Square magnitude for more precise control at low speeds
		linearMagnitude = Math.pow(linearMagnitude, 2);

		// Return new linear velocity
		return new Pose2d(new Translation2d(), linearDirection)
				.transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
				.getTranslation();
	}

	/**
	 * Returns a field relative drive command using two joysticks on a controller. Uses quadratic
	 * control, where the input is squared for more precise control at lower speeds.
	 *
	 * @param drive         The drive subsystem to control.
	 * @param xSupplier     The supplier for the X axis input.
	 * @param ySupplier     The supplier for the Y axis input.
	 * @param omegaSupplier The supplier for the rotation input.
	 * @return A command that drives the robot based on joystick inputs.
	 */
	public static Command joystickDrive(SwerveSubsystem drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier,
			DoubleSupplier omegaSupplier) {
		return Commands.run(() -> {
			double omegaSupplied = omegaSupplier.getAsDouble();
			double xSupplied = xSupplier.getAsDouble();
			double ySupplied = ySupplier.getAsDouble();

			// Enforce slow mode
			if (drive.isSlowMode()) {
				omegaSupplied *= DriveConstants.kSlowModeRotationalMultiplier;
				xSupplied *= DriveConstants.kSlowModeTranslationalMultiplier;
				ySupplied *= DriveConstants.kSlowModeTranslationalMultiplier;
			}

			// Get linear velocity
			Translation2d linearVelocity = getLinearVelocityFromJoysticks(xSupplied, ySupplied);

			// Apply rotation deadband
			double omega = MathUtil.applyDeadband(omegaSupplied, OIConstants.kDriverControllerDeadband);

			// Square rotation value for more precise control
			omega = Math.copySign(omega * omega, omega);

			// Aim towards the shot target
			var superstructure = RobotContainer.getInstance().superstructure;
			boolean isObjective = superstructure.getActionState() != ActionState.kActionIdle
					&& superstructure.isLatestShooterParamsRecent();
			double omegaOut = isObjective ? LatestShootingAngularVelocity : omega * DriveConstants.kMaxAngularSpeed;

			// Convert to field relative speeds & send command
			ChassisSpeeds speeds = new ChassisSpeeds(linearVelocity.getX() * DriveConstants.kMaxSpeedMetersPerSecond,
					linearVelocity.getY() * DriveConstants.kMaxSpeedMetersPerSecond, omegaOut);
			boolean isFlipped = DriverStation.getAlliance().isPresent()
					&& DriverStation.getAlliance().get() == Alliance.Blue;

			speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds,
					isFlipped ? drive.getRobotRotation().plus(new Rotation2d(Math.PI)) : drive.getRobotRotation());

			drive.runSpeeds(speeds, false);
		}, drive);
	}

	/**
	 * Measures the velocity feedforward constants for the drive motors.
	 *
	 * @param drive The drive subsystem to characterize.
	 * @return The feedforward characterization command.
	 */
	public static Command feedforwardCharacterization(SwerveSubsystem drive) {
		List<Double> velocitySamples = new LinkedList<>();
		List<Double> voltageSamples = new LinkedList<>();
		Timer timer = new Timer();

		return Commands.sequence(
				// Reset data
				Commands.runOnce(() -> { velocitySamples.clear(); voltageSamples.clear(); }),

				// Allow modules to orient
				Commands.run(() -> { drive.runCharacterization(0.0); }, drive).withTimeout(kFFStartDelay),

				// Start timer
				Commands.runOnce(timer::restart),

				// Accelerate and gather data
				Commands.run(() -> {
					double voltage = timer.get() * kFFVoltageRampRate;
					drive.runCharacterization(voltage);
					velocitySamples.add(drive.getFeedforwardCharacterizationVelocity());
					voltageSamples.add(voltage);
				}, drive)

						// When cancelled, calculate and print results
						.finallyDo(() -> {
							int n = velocitySamples.size();
							double sumX = 0.0;
							double sumY = 0.0;
							double sumXY = 0.0;
							double sumX2 = 0.0;
							for (int i = 0; i < n; i++) {
								sumX += velocitySamples.get(i);
								sumY += voltageSamples.get(i);
								sumXY += velocitySamples.get(i) * voltageSamples.get(i);
								sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
							}
							double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
							double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

							NumberFormat formatter = new DecimalFormat("#0.00000");
							System.out.println("********** DRIVE FEEDFORWARD CHARACTERIZATION RESULTS **********");
							System.out.println("\tkS: " + formatter.format(kS));
							System.out.println("\tkV: " + formatter.format(kV));
						}));
	}

	/**
	 * Create a characterization command that commands a constant forward linear speed and ramps
	 * the angular rate (omega) linearly while collecting samples. Fits the collected samples into
	 * a linear model. Logs the test reults.
	 *
	 * <p>
	 * <b>Note</b>: The initial skew correction factor must be set to 0. A non-zero skew factor
	 * should only be used for the validation of a factor, in which case a value closer to 0 is
	 * better.
	 *
	 * @param drive The drive subsystem to characterize.
	 * @return The skew characterization command.
	 */
	public static Command skewCharacterization(SwerveSubsystem drive) {
		List<Double> omegaSamples = new ArrayList<>();
		List<Double> deltaThetaSamples = new ArrayList<>();
		Timer timer = new Timer();

		return Commands.sequence(
				// Reset data
				Commands.runOnce(() -> { omegaSamples.clear(); deltaThetaSamples.clear(); }),

				// Allow modules to orient
				Commands.run(() -> { drive.runCharacterization(0); }, drive).withTimeout(1),

				// Start timer
				Commands.runOnce(timer::restart),

				// Request speeds and sample the measured velocities
				Commands.run(() -> {
					double t = timer.get();

					// Compute commanded omega by linearly ramping from 0
					double commandedOmega = MathUtil.clamp(t * kSkewAngularRampRate, -kSkewMaxAngularSpeed,
							kSkewMaxAngularSpeed);

					// commanded robot-relative chassis speeds: forward x, zero lateral, commanded omega
					ChassisSpeeds commanded = new ChassisSpeeds(kSkewTargetLinearSpeed, // vx
							0.0, // vy
							commandedOmega // omega
					);

					drive.runSpeeds(commanded, true);

					// Sampling //

					// Measured robot-relative speeds
					ChassisSpeeds measuredRobot = drive.getChassisSpeedsFieldRelative();

					double vCmdMag = Math.hypot(commanded.vxMetersPerSecond, commanded.vyMetersPerSecond);

					// Skip noisy samples when commanded translational speed is too small
					if (vCmdMag < kSkewMinSpeed) return;

					double thetaCmd = Math.atan2(commanded.vyMetersPerSecond, commanded.vxMetersPerSecond);
					double thetaMeas = Math.atan2(measuredRobot.vyMetersPerSecond, measuredRobot.vxMetersPerSecond);

					double deltaTheta = MathUtil.inputModulus(thetaMeas - thetaCmd, -Math.PI, Math.PI);

					// Use gyro measured angular velocity
					double omegaRadPerSec = Math.toRadians(drive.getGyro().getInputs().omegaDegreesPerSecond);

					// Skip tiny omega samples
					if (Math.abs(omegaRadPerSec) < kSkewMinAngularSpeed) return;

					omegaSamples.add(omegaRadPerSec);
					deltaThetaSamples.add(deltaTheta);
				}, drive)
						// When the command finishes/cancelled, compute k
						.finallyDo(() -> {
							int n = omegaSamples.size();
							if (n == 0) {
								System.out.println(
										"No valid samples collected for skew identification. Re-run the test.");
								return;
							}

							// Estimate slope
							double sumWD = 0.0, sumW2 = 0.0;
							for (int i = 0; i < n; ++i) {
								double w = omegaSamples.get(i);
								double d = deltaThetaSamples.get(i);
								sumWD += w * d;
								sumW2 += w * w;
							}
							double slopeK = sumWD / sumW2;

							double sumResidualSq = 0.0;
							double residualMin = Double.POSITIVE_INFINITY, residualMax = Double.NEGATIVE_INFINITY;
							double residualMinW = 0.0, residualMaxW = 0.0;
							for (int i = 0; i < n; ++i) {
								double w = omegaSamples.get(i);
								double d = deltaThetaSamples.get(i);
								double residual = d - slopeK * w;
								sumResidualSq += residual * residual;
								if (Math.abs(
										residual) < residualMin) { residualMin = Math.abs(residual); residualMinW = w; }

								if (Math.abs(
										residual) > residualMax) { residualMax = Math.abs(residual); residualMaxW = w; }
							}

							double rmse = Math.sqrt(sumResidualSq / n);

							NumberFormat formatter = new DecimalFormat("#0.00000");
							System.out.println("********** SKEW CHARACTERIZATION RESULTS **********");
							System.out.println("\tk: " + formatter.format(slopeK) + " s");
							System.out.println("\tRMSE: " + formatter.format(Math.toDegrees(rmse)) + " deg");
							System.out.println(
									"\tAbsolute Residuals: [min: " + formatter.format(Math.toDegrees(residualMin))
											+ " deg @ " + formatter.format(Math.toDegrees(residualMinW))
											+ " deg/s, max: " + formatter.format(Math.toDegrees(residualMax))
											+ " deg @ " + formatter.format(Math.toDegrees(residualMaxW)) + " deg/s]");
							System.out.println("\tSamples used: " + n);
						}));
	}

	/**
	 * Identifies the wheel coefficient of friction by slowly raising a ramp until the robot begins
	 * to slide. A tiltable platform covered with the FRC carpet is required for this test.
	 *
	 * <p>
	 * <b>The motors must be set to coast before usage!</b>
	 *
	 * @param drive The swerve subsystem.
	 * @return The wheel COF identification command.
	 */
	public static Command identifyWheelCOF(SwerveSubsystem drive) {
		Container<Boolean> hasSlid = new Container<Boolean>(false);
		LinkedList<Double> pitchSampleBuffer = new LinkedList<>();

		return Commands.sequence(Commands.runOnce(() -> {
			System.out.println("Starting wheel COF identification in " + kWheelCOFStartDelay + " seconds...");
			System.out.println(
					"**** Make sure the robot is initially stationary, the wheels are aligned parallel to the incline, and the motors are set to coast! ****");
			System.out.println("When sampling starts, slowly raise the ramp the robot is on until it begins to slide.");
		}),
				// Allow modules to orient
				Commands.run(() -> { drive.runCharacterization(0.0); }, drive).withTimeout(kWheelCOFStartDelay),

				Commands.runOnce(() -> System.out
						.println("Sampling has begun, please raise the ramp slowly until the robot begins sliding.")),

				// Start gathering data
				Commands.run(() -> {
					double pitch = Math.abs(drive.getGyro().getInputs().pitchDegrees);
					double velocity = Math.hypot(drive.getChassisSpeedsFieldRelative().vxMetersPerSecond,
							drive.getChassisSpeedsFieldRelative().vyMetersPerSecond);
					boolean isSliding = velocity > kWheelCOFSlideThreshold;

					pitchSampleBuffer.add(Math.toRadians(pitch));

					if (pitchSampleBuffer.size() > kWheelCOFMaxPitchBufferSize) pitchSampleBuffer.removeFirst();
					hasSlid.val = isSliding;
				}).until(() -> hasSlid.val).finallyDo(() -> {
					if (pitchSampleBuffer.size() == 0) {
						System.out.println("No valid samples collected for wheel COF identification. Re-run the test.");
						return;
					}
					double sumPitch = 0.0;
					for (int i = Math.max(0,
							pitchSampleBuffer.size() - kWheelCOFBacktrackSamples); i < pitchSampleBuffer.size(); i++)
						sumPitch += pitchSampleBuffer.get(i);
					double avgSlidePitch = sumPitch / Math.max(1, pitchSampleBuffer.size()
							- Math.max(0, pitchSampleBuffer.size() - kWheelCOFBacktrackSamples));

					// Calculate and print results
					NumberFormat formatter = new DecimalFormat("#0.00000");
					System.out.println("********** WHEEL C.O.F. CHARACTERIZATION RESULTS **********");
					System.out.println("\tAverage Slide Angle: " + formatter.format(avgSlidePitch) + " rad / "
							+ formatter.format(Math.toDegrees(avgSlidePitch)) + " deg");
					System.out.println(
							"\tEstimated Friction Coefficient (kS): " + formatter.format(Math.tan(avgSlidePitch)));
				}));
	}

	// Calibrations (with noisy sim):
	// Positional precision: 0.01+ mm precision.
	// Angular precision: 0.0001-0.001+ deg precision.

	/**
	 * Calibrates the specified vision camera's offset by collecting samples for each camera and
	 * averaging them.
	 *
	 * @param drive         The swerve subsystem.
	 * @param camIdx        The index of the camera to calibrate.
	 * @param angularOffset The angular offset to apply to the tag's pose. For example, if a front
	 *                      camera is being calibrated, this should be 180 as the robot faces
	 *                      towards the tag, if a rear camera is being calibrated, this should be 0
	 *                      as the robot faces away from the tag.
	 * @param tagDist       The perpendicular distance from the center of the robot to the tag.
	 * @return The calibration command.
	 */
	public static Command calibrateVisionCameraOffset(SwerveSubsystem drive, int camIdx, Rotation2d angularOffset,
			double tagDist) {
		List<Pose3d> samples = new ArrayList<>();
		LibVisionSubsystem vision = drive.getVisionSubsystem();

		Container<Boolean> hasSampled = new Container<Boolean>(false);
		IntegerContainer currentSample = new IntegerContainer(0);

		return Commands.sequence(Commands.runOnce(() -> {
			System.out.println("Starting vision camera offset calibration in " + kVisionCalStartDelay + " seconds...");
			System.out.println("**** Make sure the robot is stationary and the camera only sees 1 target! ****");
		}), Commands.waitSeconds(kVisionCalStartDelay), Commands.runOnce(drive::stop),
				Commands.runOnce(() -> { System.out.println("Collecting camera samples, this may take a moment..."); }),
				Commands.run(() -> {
					Optional<Pose3d> tagPose = LibVisionSubsystem.kLayout
							.getTagPose(vision.getCameraInputs()[camIdx].latestTargetObservation.tagId());
					if (tagPose.isPresent()) {
						Pose3d offseted = PoseUtils.offsetPerpendicular(tagPose.get(), tagDist);
						offseted = new Pose3d(offseted.getTranslation(),
								offseted.getRotation().plus(new Rotation3d(angularOffset)));
						Pose3d sample = VisionCalibration.computeCameraInRobot(
								new Pose3d(new Translation3d(offseted.getX(), offseted.getY(), 0),
										offseted.getRotation()),
								tagPose.get(), vision.getCameraInputs()[camIdx].latestTargetObservation.camToTarget());
						samples.add(sample);
					}

					if (++currentSample.value >= kVisionCalSampleCountPerCam) {
						System.out.println("Collected " + samples.size() + " samples for Camera #" + (camIdx + 1)
								+ " [Failed samples: " + (kVisionCalSampleCountPerCam - samples.size()) + "]");
						hasSampled.val = true;
					}
				}).until(() -> hasSampled.val)).finallyDo(() -> {
					if (samples.size() == 0) {
						System.out.println(
								"No valid samples collected for Camera #" + (camIdx + 1) + ". Re-run the test.");
						return;
					}
					var result = VisionCalibration.averageCameraInRobotSamples(samples);
					NumberFormat formatter = new DecimalFormat("#0.00000");
					System.out.println("********** CAMERA OFFSET CALIBRATION RESULTS **********");
					System.out.println("Camera #" + (camIdx + 1) + " Offset (m, rad): ");
					System.out.println("\tX: " + formatter.format(result.getX()) + " m");
					System.out.println("\tY: " + formatter.format(result.getY()) + " m");
					System.out.println("\tZ: " + formatter.format(result.getZ()) + " m");
					System.out.println("\tRoll (x-rot): " + formatter.format(result.getRotation().getX()) + " rad / "
							+ formatter.format(Units.radiansToDegrees(result.getRotation().getX())) + " deg");
					System.out.println("\tPitch (y-rot): " + formatter.format(result.getRotation().getY()) + " rad / "
							+ formatter.format(Units.radiansToDegrees(result.getRotation().getY())) + " deg");
					System.out.println("\tYaw (z-rot): " + formatter.format(result.getRotation().getZ()) + " rad / "
							+ formatter.format(Units.radiansToDegrees(result.getRotation().getZ())) + " deg");
				});
	}

	/**
	 * Runs a full set of SysId tests. Runs forward and reverse dynamic and quasistatic tests, 4 in
	 * total.
	 *
	 * @param drive                The swerve subsystem.
	 * @param dynamicTimeLimit     The time limit for the dynamic tests, in seconds.
	 * @param quasistaticTimeLimit The time limit for the quasistatic tests, in seconds.
	 * @return The sequenced SysId commands.
	 */
	public static Command sysIdFullTest(SwerveSubsystem drive, double dynamicTimeLimit, double quasistaticTimeLimit) {
		return drive.sysIdDynamic(Direction.kForward)
				.withTimeout(dynamicTimeLimit)
				.andThen(drive.sysIdDynamic(Direction.kReverse).withTimeout(dynamicTimeLimit))
				.andThen(drive.sysIdQuasistatic(Direction.kForward).withTimeout(quasistaticTimeLimit))
				.andThen(drive.sysIdQuasistatic(Direction.kReverse).withTimeout(quasistaticTimeLimit));
	}

	/**
	 * Precisely measures the robot's wheel radius by spinning.
	 *
	 * @param drive The swerve subsystem.
	 * @return The wheel identification command.
	 */
	public static Command wheelRadiusCharacterization(SwerveSubsystem drive) {
		SlewRateLimiter limiter = new SlewRateLimiter(kWheelRadRampRate);
		WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

		return Commands.parallel(
				// Drive control sequence
				Commands.sequence(
						// Reset acceleration limiter
						Commands.runOnce(() -> limiter.reset(0.0)),

						// Turn in place, accelerating up to full speed
						Commands.run(() -> {
							double speed = limiter.calculate(kWheelRadMaxVel);
							drive.runSpeeds(new ChassisSpeeds(0.0, 0.0, speed), false);
						}, drive)),

				// Measurement sequence
				Commands.sequence(
						// Wait for modules to fully orient before starting measurement
						Commands.waitSeconds(1.0),

						// Record starting measurement
						Commands.runOnce(() -> {
							double[] positions = Arrays.stream(drive.getModulePositions())
									.mapToDouble(k -> k.distanceMeters)
									.toArray();
							for (int i = 0; i < positions.length; i++) {
								positions[i] = 2 * positions[i] / ModuleConstants.kWheelDiameterMeters;
							}
							state.positions = positions;
							state.lastAngle = drive.getRobotRotation();
							state.gyroDelta = 0.0;
						}),

						// Update gyro delta
						Commands.run(() -> {
							var rotation = drive.getRobotRotation();
							state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
							state.lastAngle = rotation;
						})
								// When cancelled, calculate and print results
								.finallyDo(() -> {
									double[] positions = Arrays.stream(drive.getModulePositions())
											.mapToDouble(k -> k.distanceMeters)
											.toArray();
									for (int i = 0; i < positions.length; i++) {
										positions[i] = 2 * positions[i] / ModuleConstants.kWheelDiameterMeters;
									}
									double wheelDelta = 0.0;
									for (int i = 0; i < 4; i++) { wheelDelta += Math
											.abs(positions[i] - state.positions[i]) / 4.0; }
									double wheelRadius = (state.gyroDelta * DriveConstants.kDrivebaseRadius)
											/ wheelDelta;

									NumberFormat formatter = new DecimalFormat("#0.000");
									System.out.println("********** Wheel Radius Characterization Results **********");
									System.out.println("\tWheel Delta: " + formatter.format(wheelDelta) + " rad");
									System.out.println("\tGyro Delta: " + formatter.format(state.gyroDelta) + " rad");
									System.out.println("\tWheel Radius: " + formatter.format(wheelRadius) + " m, "
											+ formatter.format(Units.metersToInches(wheelRadius)) + " in");
								})));
	}

	private static class WheelRadiusCharacterizationState {
		double[] positions = new double[4];
		Rotation2d lastAngle = new Rotation2d();
		double gyroDelta = 0.0;
	}
}
