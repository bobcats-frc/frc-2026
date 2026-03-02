package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import static frc.robot.constants.FieldConstants.path;

import com.bobcats.lib.auto.CustomRoutineBuilder;
import com.bobcats.lib.utils.AllianceUtil;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants.PhysicalParameters;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.superstructure.SuperstructureConstants;
import frc.robot.subsystems.swerve.SwerveConstants.AutoConstants;
import frc.robot.subsystems.swerve.VisionConstants;
import frc.robot.util.TrajectoryUtils2026;
import frc.robot.util.TrajectoryUtils2026.GeneratorParams;
import java.io.IOException;
import java.util.Set;
import java.util.function.BooleanSupplier;
import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * A class for building custom autonomous routines based on dashboard selections.
 */
public class AutonomousBuilder implements CustomRoutineBuilder {

	// Enums //

	/** An enum specifying the autonomous routine type. */
	public enum AutoType {
		kCustom, kPathPlanner;
	}

	/** An enum specifying the starting point of the robot. */
	public enum AutoStartingPoint {
		kMiddle, kRight, kLeft;
	}

	/** An enum specifying an autonomous routine objective. */
	public enum AutoObjective {
		kNone, kLeave, kScoreDepot, kScoreOutpost, kScoreNeutral, kScoreNeutralLoopOver, kScoreNeutralSweepOver,
		kScoreNeutralAdaptive;
	}

	/** An enum specifying the autonomous routine climb side. */
	public enum AutoClimbSide {
		kNone, kRight, kLeft, kPark;
	}

	/**
	 * An enum specifying whether to override the bump crossing side for neutral scoring.
	 */
	public enum AutoBumpCrossingOverride {
		kNone, kForceRight, kForceLeft;
	}

	// Dashboard choosers
	private LoggedDashboardChooser<AutoType> m_autoTypeChooser;
	// NOTE: Only the climb side chooser works with pathplanner routines.
	private LoggedDashboardChooser<Command> m_ppAutoRoutine;

	private LoggedDashboardChooser<AutoStartingPoint> m_startingPointChooser;
	private LoggedDashboardChooser<AutoObjective> m_objective1Chooser;
	private LoggedDashboardChooser<AutoObjective> m_objective2Chooser;
	private LoggedDashboardChooser<AutoBumpCrossingOverride> m_forceNeutralCrossSideOverride;
	private LoggedDashboardChooser<Double> m_adaptiveNeutralTimeBudgetChooser;
	private LoggedDashboardChooser<AutoClimbSide> m_climbSideChooser;
	private LoggedDashboardChooser<Boolean> m_passThroughTowerObj2;
	private LoggedDashboardChooser<Boolean> m_passThroughTowerObj1;
	private LoggedDashboardChooser<Boolean> m_shouldScoreChooser;

	// Incompatibility warnings
	private Alert m_doubleDepotAlert, m_doubleOutpostAlert, m_deadOnTheFieldAlert, m_redundantBumpOverride,
			m_redundantTowerCrossObj2, m_redundantTowerCrossObj1, m_redundantPathPlannerPath, m_noClimb,
			m_noPathSelected, m_invalidLeaveObjective, m_unableToObtainPath;

	// TODO Re-measure with real robot
	// Constants
	private static final double kDepotIdleCooldown = 0.4;
	private static final double kOutpostIdleCooldown = 2.5;

	private static final double kPassAllianceBumpTimer = 1.0;
	private static final double kNeutralIntakeTimer = 1.0;
	private static final double kLeaveTimer = 0.5;

	private static final double kAdaptivePathStopPenalty = 0.2;
	private static final double kAdaptivePathWaypointVelocity = 1.3;

	private static final double kAutoBufferTime = 0.12;

	private static final ChassisSpeeds kLeaveSpeedsFwdBlue = new ChassisSpeeds(-1.5, 0, 0);

	private static final ChassisSpeeds kIntakeNeutralSpeedsFwdBlue = new ChassisSpeeds(3.5, 0, 0);
	private static final ChassisSpeeds kIntakeNeutralSpeedsRevBlue = new ChassisSpeeds(-3.5, 0, 0);

	private static final ChassisSpeeds kPassBumpSpeedsFwdBlue = new ChassisSpeeds(2, 0, 0);
	private static final ChassisSpeeds kPassBumpSpeedsRevBlue = new ChassisSpeeds(-2.5, 0, 0);

	// Pre-pathed trajectories
	private PathPlannerPath m_rightLoopOver, m_leftLoopOver, m_rightSweepOver, m_leftSweepOver, m_towerLR, m_towerRL;

	/** Constructs a new AutonomousSequenceBuilder. */
	public AutonomousBuilder() {
		// Initialize choosers
		m_autoTypeChooser = new LoggedDashboardChooser<>("Autonomous Type");
		m_autoTypeChooser.addDefaultOption("Custom Builder", AutoType.kCustom);
		m_autoTypeChooser.addOption("PathPlanner Preset", AutoType.kPathPlanner);

		m_ppAutoRoutine = new LoggedDashboardChooser<>("Autonomous PathPlanner Routines",
				AutoBuilder.buildAutoChooser());

		m_startingPointChooser = new LoggedDashboardChooser<>("Autonomous Starting Point");
		m_startingPointChooser.addDefaultOption("Middle", AutoStartingPoint.kMiddle);
		m_startingPointChooser.addOption("Right", AutoStartingPoint.kRight);
		m_startingPointChooser.addOption("Left", AutoStartingPoint.kLeft);

		m_objective1Chooser = new LoggedDashboardChooser<>("Autonomous Objective #1");
		m_objective1Chooser.addDefaultOption("None", AutoObjective.kNone);
		m_objective1Chooser.addOption("Leave", AutoObjective.kLeave);
		m_objective1Chooser.addOption("Score Depot (Primary)", AutoObjective.kScoreDepot);
		m_objective1Chooser.addOption("Score Outpost (Primary)", AutoObjective.kScoreOutpost);
		m_objective1Chooser.addOption("Score Neutral (Primary)", AutoObjective.kScoreNeutral);
		m_objective1Chooser.addOption("Score Neutral (Primary - Loop Over)", AutoObjective.kScoreNeutralLoopOver);
		m_objective1Chooser.addOption("Score Neutral (Primary - Sweep Over)", AutoObjective.kScoreNeutralSweepOver);
		m_objective1Chooser.addOption("Score Neutral (Primary - Adaptive)", AutoObjective.kScoreNeutralAdaptive);

		m_objective2Chooser = new LoggedDashboardChooser<>("Autonomous Objective #2");
		m_objective2Chooser.addDefaultOption("None", AutoObjective.kNone);
		m_objective2Chooser.addOption("Score Depot (Secondary)", AutoObjective.kScoreDepot);
		m_objective2Chooser.addOption("Score Outpost (Secondary)", AutoObjective.kScoreOutpost);

		m_forceNeutralCrossSideOverride = new LoggedDashboardChooser<>("Autonomous Override Bump Cross");
		m_forceNeutralCrossSideOverride.addDefaultOption("No Override", AutoBumpCrossingOverride.kNone);
		m_forceNeutralCrossSideOverride.addOption("Force Right Bump", AutoBumpCrossingOverride.kForceRight);
		m_forceNeutralCrossSideOverride.addOption("Force Left Bump", AutoBumpCrossingOverride.kForceLeft);

		m_adaptiveNeutralTimeBudgetChooser = new LoggedDashboardChooser<>("Autonomous Adaptive Neutral Time Budget");
		m_adaptiveNeutralTimeBudgetChooser.addDefaultOption("5 seconds", 5.0);
		m_adaptiveNeutralTimeBudgetChooser.addOption("4 seconds", 4.0);
		m_adaptiveNeutralTimeBudgetChooser.addOption("3 seconds", 3.0);

		m_climbSideChooser = new LoggedDashboardChooser<>("Autonomous Climb Side");
		m_climbSideChooser.addDefaultOption("None", AutoClimbSide.kNone);
		m_climbSideChooser.addOption("Right", AutoClimbSide.kRight);
		m_climbSideChooser.addOption("Left", AutoClimbSide.kLeft);
		m_climbSideChooser.addOption("Park", AutoClimbSide.kPark);

		m_passThroughTowerObj1 = new LoggedDashboardChooser<>("Autonomous Pass Through Tower Objective #1");
		m_passThroughTowerObj1.addDefaultOption("No", false);
		m_passThroughTowerObj1.addOption("Yes", true);

		m_passThroughTowerObj2 = new LoggedDashboardChooser<>("Autonomous Pass Through Tower Objective #2");
		m_passThroughTowerObj2.addDefaultOption("No", false);
		m_passThroughTowerObj2.addOption("Yes", true);

		m_shouldScoreChooser = new LoggedDashboardChooser<>("Autonomous Scoring");
		m_shouldScoreChooser.addDefaultOption("Score", true);
		m_shouldScoreChooser.addOption("Don't Score", false);

		// Load individual paths
		m_unableToObtainPath = new Alert("Unable to obtain an autonomous builder path! Reboot the robot.",
				AlertType.kError);
		try {
			m_rightLoopOver = PathPlannerPath.fromPathFile(path("RightLoopOverEnd"));
			m_leftLoopOver = PathPlannerPath.fromPathFile(path("LeftLoopOverEnd"));
			m_rightSweepOver = PathPlannerPath.fromPathFile(path("RightSweepOverEnd"));
			m_leftSweepOver = PathPlannerPath.fromPathFile(path("LeftSweepOverEnd"));
			m_towerRL = PathPlannerPath.fromPathFile(path("CrossTowerRL"));
			m_towerLR = PathPlannerPath.fromPathFile(path("CrossTowerLR"));
		} catch (FileVersionException | IOException | ParseException e) {
			m_unableToObtainPath.set(true);
			DriverStation.reportWarning("WARNING: AutonomousSequenceBuilder::new, unable to obtain a PathPlanner path",
					false);
			e.printStackTrace();
		}

		// Initialize alerts
		m_doubleDepotAlert = new Alert("Incompatible autonomous! Primary and secondary objectives both set to depot.",
				AlertType.kWarning);
		m_deadOnTheFieldAlert = new Alert("No autonomous selected with custom builder! Robot is dead on the field.",
				AlertType.kWarning);
		m_redundantBumpOverride = new Alert("Redundant bump override given, but no neutral scoring objective selected.",
				AlertType.kWarning);
		m_redundantTowerCrossObj1 = new Alert("Redundant Objective #1 tower cross given with invalid objectives.",
				AlertType.kWarning);
		m_redundantTowerCrossObj2 = new Alert("Redundant Objective #2 tower cross given with invalid objectives.",
				AlertType.kWarning);
		m_redundantPathPlannerPath = new Alert(
				"Redundant PathPlanner path was selected even though the custom builder is being used.",
				AlertType.kWarning);
		m_noClimb = new Alert("No climb was selected, is this intentional?", AlertType.kWarning);
		m_noPathSelected = new Alert("PathPlanner mode active, but no PathPlanner path selected!", AlertType.kWarning);
		m_invalidLeaveObjective = new Alert(
				"Redundant leave objective is set as primary when a secondary or climb objective is present. Set primary to none instead.",
				AlertType.kWarning);
		m_doubleOutpostAlert = new Alert(
				"Incompatible autonomous! Primary and secondary objectives both set to outpost.", AlertType.kWarning);
	}

	/**
	 * Updates the NetworkTable alerts to show possible problems with the autonomous sequence.
	 */
	public void updateNTAlerts() {
		var obj1 = m_objective1Chooser.get();
		var obj2 = m_objective2Chooser.get();
		var autoType = m_autoTypeChooser.get();
		var climbSide = m_climbSideChooser.get();

		// Both objectives set to Depot
		m_doubleDepotAlert.set(obj1 == AutoObjective.kScoreDepot && obj2 == AutoObjective.kScoreDepot);
		// Both objectives set to Outpost
		m_doubleOutpostAlert.set(obj1 == AutoObjective.kScoreOutpost && obj2 == AutoObjective.kScoreOutpost);
		// No valid auto selected, no autonomous action, all set to none
		m_deadOnTheFieldAlert.set(autoType == AutoType.kCustom && obj1 == AutoObjective.kNone
				&& obj2 == AutoObjective.kNone && climbSide == AutoClimbSide.kNone);

		// Whether Objective #1 is a neutral zone objective
		boolean isPrimaryNeutralObj = obj1 == AutoObjective.kScoreNeutral || obj1 == AutoObjective.kScoreNeutralAdaptive
				|| obj1 == AutoObjective.kScoreNeutralLoopOver || obj1 == AutoObjective.kScoreNeutralSweepOver;

		// A bump override provided with no neutral objective
		m_redundantBumpOverride
				.set(!isPrimaryNeutralObj && m_forceNeutralCrossSideOverride.get() != AutoBumpCrossingOverride.kNone);

		// Cross to go from the depot to the outpost or vice versa
		boolean objective2CrossBetweenDepotOutpost = (obj1 == AutoObjective.kScoreDepot
				&& obj2 == AutoObjective.kScoreOutpost)
				|| (obj1 == AutoObjective.kScoreOutpost && obj2 == AutoObjective.kScoreDepot);
		// Cross from right bump to the depot (left), inverted logic if a sweep auto is
		// used as that comes out of the bump opposite to the bump it started
		boolean objective2CrossFromRightBumpToDepot = ((isCrossRightObjective1()
				&& obj1 != AutoObjective.kScoreNeutralSweepOver)
				|| (!isCrossRightObjective1() && obj1 == AutoObjective.kScoreNeutralSweepOver)) && isPrimaryNeutralObj
				&& obj2 == AutoObjective.kScoreDepot;
		// Same as obove, just inverted logic
		boolean objective2CrossFromLeftBumpToOutpost = !((isCrossRightObjective1()
				&& obj1 != AutoObjective.kScoreNeutralSweepOver)
				|| (!isCrossRightObjective1() && obj1 == AutoObjective.kScoreNeutralSweepOver)) && isPrimaryNeutralObj
				&& obj2 == AutoObjective.kScoreOutpost;

		// Passing through the tower with no real purpose for Objective #2
		m_redundantTowerCrossObj2.set(m_passThroughTowerObj2.get() && !(objective2CrossBetweenDepotOutpost
				|| objective2CrossFromRightBumpToDepot || objective2CrossFromLeftBumpToOutpost));

		// In the beginning, crossing from the right starting point to the depot (left)
		boolean objective1CrossRightToLeftValid = m_startingPointChooser.get() == AutoStartingPoint.kRight
				&& obj1 == AutoObjective.kScoreDepot;
		// In the beginning, crossing from the left starting point to the outpost
		// (right)
		boolean objective1CrossLeftToRightValid = m_startingPointChooser.get() == AutoStartingPoint.kLeft
				&& obj1 == AutoObjective.kScoreOutpost;
		// Only allow passing through for opposite side objectives
		m_redundantTowerCrossObj1.set(
				m_passThroughTowerObj1.get() && !(objective1CrossRightToLeftValid || objective1CrossLeftToRightValid));

		// PathPlanner path selected for the custom builder
		m_redundantPathPlannerPath.set(
				autoType == AutoType.kCustom && !m_ppAutoRoutine.getSendableChooser().getSelected().equals("None"));
		// No climb objective given
		m_noClimb.set(climbSide == AutoClimbSide.kNone);
		// No path selected for PathPlanner
		m_noPathSelected.set(
				autoType == AutoType.kPathPlanner && m_ppAutoRoutine.getSendableChooser().getSelected().equals("None"));
		// Redundant leave objective, wastes time
		m_invalidLeaveObjective
				.set(obj1 == AutoObjective.kLeave && (obj2 != AutoObjective.kNone || climbSide != AutoClimbSide.kNone));
	}

	@Override
	public Command getRoutine() {
		var robotContainer = RobotContainer.getInstance();
		var superstructure = robotContainer.superstructure;
		var swerve = robotContainer.swerve;

		var obj1 = m_objective1Chooser.get();
		var obj2 = m_objective2Chooser.get();
		var passTowerObj1 = m_passThroughTowerObj1.get();
		var passTowerObj2 = m_passThroughTowerObj2.get();
		var autoType = m_autoTypeChooser.get();
		var climbSide = m_climbSideChooser.get();

		// Pre-extend arm to save time
		Command climbInitiate = superstructure.climbForceExtend()
				.onlyIf(() -> climbSide != AutoClimbSide.kNone && climbSide != AutoClimbSide.kPark);

		// Return early if using a PP routine
		if (autoType == AutoType.kPathPlanner)
			return Commands.runOnce(() -> superstructure.setIsClimbRightSide(climbSide != AutoClimbSide.kLeft))
					.andThen(climbInitiate)
					// .andThen(() ->
					// CommandScheduler.getInstance().schedule(m_ppAutoRoutine.get()))
					.andThen(m_ppAutoRoutine.get().asProxy())
					.withName(m_ppAutoRoutine.get().getName());

		// Resetting the odometry / starting pose
		Command resetOdomCommand = Commands.runOnce(() -> swerve.resetOdometry(getStartingPoseFlipped()));

		// Setting robot objective state
		Command scoreCommand = m_shouldScoreChooser.get()
				? Commands.runOnce(() -> robotContainer.initAutonScoreFuelCommand()) : Commands.none();

		boolean neutralAlignRightObj1 = isCrossRightObjective1();

		var deferredNeutralAdaptive = new DeferredCommand(() -> {
			var path = TrajectoryUtils2026.generateGridOptimizedPath(getTrajectoryParams(),
					robotContainer.objectDetection.getDetectedInstances());
			if (!path.isPresent()) {
				DriverStation.reportWarning("WARNING: Adaptive auto has failed, no path generated! Using fallback.",
						false);
				Logger.recordOutput("AdaptivaAutoFailedObjects",
						robotContainer.objectDetection.getDetectedInstances().toArray(Pose2d[]::new));
				// Use fallback path instead
				return runIntakeSpeeds(true).andThen(() -> swerve.stop())
						.andThen(runIntakeSpeeds(false))
						.andThen(() -> swerve.stop());
			}

			// Make sure to look in the opposite direction of travel to intake
			PPHolonomicDriveController.overrideRotationFeedback(() -> {
				var speeds = swerve.getChassisSpeedsFieldRelative();
				return swerve.ArbitraryPIDAngular.calculate(swerve.getRobotRotation().getRadians(),
						Math.atan2(speeds.vyMetersPerSecond, speeds.vxMetersPerSecond) + Math.PI);
			});

			return TrajectoryUtils2026.followTrajectoryByPathfind(path.get(), swerve.getRobotRotation(),
					AutoConstants.kPathConstraints, kAdaptivePathWaypointVelocity);
		}, Set.of(swerve));

		// Main objective command
		Command objective1Command = switch (obj1) {
			// Simple leave objective
			case kLeave -> Commands.run(() -> swerve.runSpeeds(getFlippedSpeeds(kLeaveSpeedsFwdBlue), true), swerve)
					.withTimeout(kLeaveTimer)
					.andThen(Commands.runOnce(() -> swerve.stop()));
			// Cross tower if appropriate && intake from the depot
			case kScoreDepot -> depotTrajectory(true, m_passThroughTowerObj1::get);
			// Cross tower if appropriate && intake from the outpost
			case kScoreOutpost -> outpostTrajectory(false, m_passThroughTowerObj1::get);
			// Intake from the neutral zone via a linear horizontal trajectory
			case kScoreNeutral -> AutoBuilder
					// Set up to cross the bump
					.pathfindToPoseFlipped(neutralAlignRightObj1 ? FieldConstants.kRobotPoseRightBlue
							: FieldConstants.kRobotPoseLeftBlue, AutoConstants.kPathConstraints)
					.andThen(Commands.runOnce(() -> robotContainer.cancelAutonScoreFuelCommand()))
					.andThen(passBump(true))
					// Drive straight then back
					.andThen(runIntakeSpeeds(true).alongWith(superstructure.startIntaking()))
					.andThen(() -> swerve.stop())
					.andThen(runIntakeSpeeds(false))
					.andThen(() -> swerve.stop())
					.andThen(Commands.runOnce(() -> superstructure.stopIntake()))
					.andThen(passBump(false))
					.andThen(() -> swerve.stop())
					.andThen(() -> {
						if (!m_shouldScoreChooser.get()) return;
						// superstructure.setObjectiveOriented(false);
						robotContainer.initAutonScoreFuelCommand();
					});
			// Intake from the neutral in a loop shape
			case kScoreNeutralLoopOver -> neutralTrajectory(neutralAlignRightObj1 ? m_rightLoopOver : m_leftLoopOver,
					neutralAlignRightObj1);
			// Sweep across the whole alliance-side neutral zone
			case kScoreNeutralSweepOver -> neutralTrajectory(neutralAlignRightObj1 ? m_rightSweepOver : m_leftSweepOver,
					neutralAlignRightObj1);
			// Adapt according to object detection
			case kScoreNeutralAdaptive -> Commands.runOnce(() -> {
				// Switch to object detection
				robotContainer.objectDetectionCamRear.setPipeline(VisionConstants.kObjectDetectionPipelineId);
				robotContainer.objectDetectionCamRear.setEnabled(true);
			})
					// Set up to pass the bump
					.andThen(AutoBuilder.pathfindToPoseFlipped(
							neutralAlignRightObj1
									? FieldConstants.kRobotPoseRightBlue.rotateAround(
											FieldConstants.kRobotPoseRightBlue.getTranslation(), Rotation2d.kPi)
									: FieldConstants.kRobotPoseLeftBlue.rotateAround(
											FieldConstants.kRobotPoseLeftBlue.getTranslation(), Rotation2d.kPi),
							AutoConstants.kPathConstraints))
					.andThen(Commands.runOnce(() -> robotContainer.cancelAutonScoreFuelCommand()))
					.andThen(passBump(true).andThen(swerve::stop))
					// TODO maybe cache the values when non-zero?
					.andThen(Commands.waitUntil(() -> robotContainer.objectDetection.getDetectedInstances().size() > 0)
							.withTimeout(1))
					// Run the adaptive trajectory
					.andThen(deferredNeutralAdaptive.alongWith(superstructure.startIntaking()))
					// Cross back to the alliance zone
					.andThen(superstructure.stopIntake())
					.andThen(passBump(false).alongWith(Commands.runOnce(() -> {
						// Switch back to localization for teleop
						robotContainer.objectDetectionCamRear.setEnabled(false);
						robotContainer.objectDetectionCamRear.setPipeline(VisionConstants.kLocalizationPipelineId);
					})))
					.andThen(() -> swerve.stop())
					.andThen(() -> {
						if (!m_shouldScoreChooser.get()) return;
						superstructure.setObjectiveOriented(false);
						robotContainer.initAutonScoreFuelCommand();
					})
					.finallyDo((interrupt) -> {
						PPHolonomicDriveController.clearRotationFeedbackOverride();
						// Switch back to localization for teleop if interrupted
						if (!interrupt) return;
						robotContainer.objectDetectionCamRear.setEnabled(false);
						robotContainer.objectDetectionCamRear.setPipeline(VisionConstants.kLocalizationPipelineId);
					});
			case kNone -> Commands.none();
			default -> Commands.none();
		};

		// Whether the primary objective is a neutral zone objective
		boolean isPrimaryNeutral = obj1 == AutoObjective.kScoreNeutral || obj1 == AutoObjective.kScoreNeutralAdaptive
				|| obj1 == AutoObjective.kScoreNeutralLoopOver || obj1 == AutoObjective.kScoreNeutralSweepOver;

		// After going to the neutral zone, would the robot enter through the right
		// bump?
		boolean isRightNeutralReEntry = ((isCrossRightObjective1() && obj1 != AutoObjective.kScoreNeutralSweepOver)
				|| (!isCrossRightObjective1() && obj1 == AutoObjective.kScoreNeutralSweepOver));

		// Secondary objective command
		Command objective2Command = switch (obj2) {
			// Cross tower if appropriate && intake from the depot (either coming from the
			// right bump or outpost)
			case kScoreDepot -> depotTrajectory(true, () -> passTowerObj2
					&& (obj1 == AutoObjective.kScoreOutpost || (isRightNeutralReEntry && isPrimaryNeutral)));
			// Cross tower if appropriate && intake from the outpost (either coming from the
			// left bump or depot)
			case kScoreOutpost -> outpostTrajectory(false, () -> passTowerObj2
					&& (obj1 == AutoObjective.kScoreDepot || (!isRightNeutralReEntry && isPrimaryNeutral)));
			case kNone -> Commands.none();
			default -> Commands.none();
		};

		// Align & climb command, choose between climb, park or neither
		Command climbCommand = Commands.either(
				Commands.runOnce(() -> { superstructure.setIsClimbRightSide(climbSide != AutoClimbSide.kLeft); })
						.andThen(Commands.either(superstructure.alignClimberAndClimbCommand(), Commands.none(),
								() -> climbSide != AutoClimbSide.kNone)),
				robotContainer.getAutoManager()
						.getPathfindCommand(path("AutoPark"), AutoConstants.kPathfindSkipTolerance),
				() -> climbSide != AutoClimbSide.kPark);

		// Piece together each command
		return Commands.waitSeconds(kAutoBufferTime)
				.andThen(resetOdomCommand)
				.andThen(scoreCommand)
				.andThen(climbInitiate)
				.andThen(objective1Command)
				.andThen(objective2Command)
				.andThen(climbCommand)
				.finallyDo(() -> superstructure.setIsClimbRightSide(true))
				.withName("Custom Auto Routine (Starting: " + m_startingPointChooser.get() + " / Objective #1: " + obj1
						+ " / Objective #2: " + obj2 + " / Climb: " + climbSide + " / Score: "
						+ m_shouldScoreChooser.get() + " / Pass Tower Objective #1: " + passTowerObj1
						+ " / Pass Tower Objective #2: " + passTowerObj2 + ")");
	}

	/** Returns the appropriately flipped starting pose. */
	private Pose2d getStartingPoseFlipped() {
		return AllianceUtil.flipWithAlliance(switch (m_startingPointChooser.get()) {
			case kMiddle -> FieldConstants.kRobotPoseMiddleBlue;
			case kRight -> FieldConstants.kRobotPoseRightBlue;
			case kLeft -> FieldConstants.kRobotPoseLeftBlue;
			default -> FieldConstants.kRobotPoseMiddleBlue;
		});
	}

	/** Returns the appropriately flipped chassis speeds. */
	private ChassisSpeeds getFlippedSpeeds(ChassisSpeeds speeds) {
		return AllianceUtil.isRedAlliance() ? FlippingUtil.flipFieldSpeeds(speeds) : speeds;
	}

	/**
	 * Returns whether the right bump should be used if going to the neutral zone for Objective #1.
	 */
	private boolean isCrossRightObjective1() {
		if (m_forceNeutralCrossSideOverride.get() != AutoBumpCrossingOverride.kNone)
			return m_forceNeutralCrossSideOverride.get() == AutoBumpCrossingOverride.kForceRight;

		return m_startingPointChooser.get() != AutoStartingPoint.kLeft;
	}

	/** Returns the command to pass the bump. */
	private Command passBump(boolean isToNeutral) {
		return Commands
				.run(() -> RobotContainer.getInstance().swerve.runSpeeds(isToNeutral
						? getFlippedSpeeds(kPassBumpSpeedsFwdBlue) : getFlippedSpeeds(kPassBumpSpeedsRevBlue), true))
				.withTimeout(kPassAllianceBumpTimer);
	}

	/** Returns the command to intake from the depot. */
	private Command depotTrajectory(boolean crossRightToLeft, BooleanSupplier towerCrossCondition) {
		return crossTower(crossRightToLeft).onlyIf(towerCrossCondition)
				.andThen(RobotContainer.getInstance().superstructure.startIntaking())
				.andThen(Commands.either(
						RobotContainer.getInstance()
								.getAutoManager()
								.getPathfindCommand(path("TowerThenDepot"), AutoConstants.kPathfindSkipTolerance),
						RobotContainer.getInstance()
								.getAutoManager()
								.getPathfindCommand(path("Depot"), AutoConstants.kPathfindSkipTolerance),
						() -> crossRightToLeft && towerCrossCondition.getAsBoolean()))
				.andThen(Commands.waitSeconds(kDepotIdleCooldown))
				.andThen(RobotContainer.getInstance().superstructure.stopIntake());
	}

	/** Returns the command to intake from the outpost. */
	private Command outpostTrajectory(boolean crossRightToLeft, BooleanSupplier towerCrossCondition) {
		return crossTower(crossRightToLeft).onlyIf(towerCrossCondition)
				.andThen(RobotContainer.getInstance()
						.getAutoManager()
						.getPathfindCommand(path("Outpost"), AutoConstants.kPathfindSkipTolerance)
						.alongWith(RobotContainer.getInstance().superstructure.startIntaking()))
				.andThen(Commands.waitSeconds(kOutpostIdleCooldown));
	}

	/**
	 * Returns the command to run the chassis speeds to intake fuel from the neutral zone in a
	 * straight line.
	 */
	private Command runIntakeSpeeds(boolean forward) {
		return Commands.run(() -> RobotContainer.getInstance().swerve.runSpeeds(
				forward ? getFlippedSpeeds(kIntakeNeutralSpeedsFwdBlue) : getFlippedSpeeds(kIntakeNeutralSpeedsRevBlue),
				true)).withTimeout(kNeutralIntakeTimer);
	}

	/** Returns the command to pass through the tower. */
	private Command crossTower(boolean isRightToLeft) {
		return pathfindThenFollow(isRightToLeft ? m_towerRL : m_towerLR);
	}

	/** Returns the command to follow a path. */
	private Command pathfindThenFollow(PathPlannerPath path) {
		return Commands.either(
				// Just try to follow if negligibly close to the starting point
				AutoBuilder.followPath(path),
				// Pathfind first, then follow if too far away
				AutoBuilder.pathfindThenFollowPath(path, AutoConstants.kPathConstraints),
				() -> RobotContainer.getInstance().swerve.getFilteredPose()
						.getTranslation()
						.getDistance(path.getStartingHolonomicPose()
								.orElse(Pose2d.kZero)
								.getTranslation()) <= AutoConstants.kPathfindSkipTolerance);
	}

	/** Returns the adaptive trajectory parameters. */
	private GeneratorParams getTrajectoryParams() {
		return new GeneratorParams(RobotContainer.getInstance().swerve.getFilteredPose(),
				m_adaptiveNeutralTimeBudgetChooser.get(), AutoConstants.kPathConstraints.maxVelocityMPS(),
				AutoConstants.kPathConstraints.maxAccelerationMPSSq(), kAdaptivePathStopPenalty,
				PhysicalParameters.kRobotWidth.in(Meters), SuperstructureConstants.kFuelCapacity,
				RobotContainer.getInstance().swerve.getFilteredPose());
	}

	/** Crosses the bump and follows a trajectory in the neutral zone. */
	private Command neutralTrajectory(PathPlannerPath path, boolean neutralAlignRight) {
		return AutoBuilder
				.pathfindToPoseFlipped(
						neutralAlignRight ? FieldConstants.kRobotPoseRightBlue : FieldConstants.kRobotPoseLeftBlue,
						AutoConstants.kPathConstraints)
				.andThen(Commands.runOnce(() -> RobotContainer.getInstance().cancelAutonScoreFuelCommand()))
				.andThen(passBump(true))
				.andThen(pathfindThenFollow(path))
				.andThen(Commands.runOnce(() -> RobotContainer.getInstance().superstructure.stopIntake()))
				.andThen(passBump(false))
				.andThen(() -> RobotContainer.getInstance().swerve.stop())
				.andThen(() -> {
					if (!m_shouldScoreChooser.get()) return;
					RobotContainer.getInstance().superstructure.setObjectiveOriented(false);
					RobotContainer.getInstance().initAutonScoreFuelCommand();
				});
	}
}
