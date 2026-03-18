package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import static frc.robot.commands.AutonomousBuilderConstants.k24ScoringTimer;
import static frc.robot.commands.AutonomousBuilderConstants.k60ScoringTimer;
import static frc.robot.commands.AutonomousBuilderConstants.k8ScoringTimer;
import static frc.robot.commands.AutonomousBuilderConstants.kAdaptivePathStopPenalty;
import static frc.robot.commands.AutonomousBuilderConstants.kAdaptivePathWaypointVelocity;
import static frc.robot.commands.AutonomousBuilderConstants.kAutoBufferTime;
import static frc.robot.commands.AutonomousBuilderConstants.kDepotIdleCooldown;
import static frc.robot.commands.AutonomousBuilderConstants.kIntakeNeutralSpeedsFwdBlue;
import static frc.robot.commands.AutonomousBuilderConstants.kIntakeNeutralSpeedsRevBlue;
import static frc.robot.commands.AutonomousBuilderConstants.kLeaveSpeedsFwdBlue;
import static frc.robot.commands.AutonomousBuilderConstants.kLeaveTimer;
import static frc.robot.commands.AutonomousBuilderConstants.kNeutralIntakeTimer;
import static frc.robot.commands.AutonomousBuilderConstants.kOutpostIdleCooldown;
import static frc.robot.commands.AutonomousBuilderConstants.kPassBumpSpeedsFwdBlue;
import static frc.robot.commands.AutonomousBuilderConstants.kPassBumpSpeedsRevBlue;
import static frc.robot.commands.AutonomousBuilderConstants.kScoringPoseLeftBlue;
import static frc.robot.commands.AutonomousBuilderConstants.kScoringPoseRightBlue;
import static frc.robot.commands.AutonomousBuilderConstants.kScoringVelocityLimitDistanceThreshold;
import static frc.robot.commands.AutonomousBuilderConstants.kSimPassBumpTimer;
import static frc.robot.constants.FieldConstants.path;

import com.bobcats.lib.auto.CustomRoutineBuilder;
import com.bobcats.lib.utils.AllianceUtil;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
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
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.commands.drive.PassBumpCommand;
import frc.robot.constants.Constants.PhysicalParameters;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.superstructure.SuperstructureConstants;
import frc.robot.subsystems.swerve.SwerveConstants.AutoConstants;
import frc.robot.subsystems.swerve.VisionConstants;
import frc.robot.util.CustomPPHolonomicDriveController;
import frc.robot.util.TrajectoryUtils2026;
import frc.robot.util.TrajectoryUtils2026.GeneratorParams;
import java.io.IOException;
import java.util.List;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
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

	/**
	 * An enum specifying whether to override the bump crossing side for neutral scoring.
	 */
	public enum AutoBumpCrossingOverride {
		kNone, kForceRight, kForceLeft;
	}

	/**
	 * The scoring mode, i.e. after what objectives to score.
	 */
	public enum ScoringMode {
		kNone, kObjective1, kObjective2, kObjective1And2;
	}

	// Dashboard choosers
	private LoggedDashboardChooser<AutoType> m_autoTypeChooser;
	private LoggedDashboardChooser<Command> m_ppAutoRoutine;

	private LoggedDashboardChooser<AutoStartingPoint> m_startingPointChooser;
	private LoggedDashboardChooser<AutoObjective> m_objective1Chooser;
	private LoggedDashboardChooser<AutoObjective> m_objective2Chooser;
	private LoggedDashboardChooser<AutoBumpCrossingOverride> m_forceNeutralCrossSideOverride;
	private LoggedDashboardChooser<Double> m_adaptiveNeutralTimeBudgetChooser;
	private LoggedDashboardChooser<Double> m_objective1EndCooldown;
	private LoggedDashboardChooser<Boolean> m_parkChooser;
	private LoggedDashboardChooser<Boolean> m_passThroughTowerObj2;
	private LoggedDashboardChooser<Boolean> m_passThroughTowerObj1;
	private LoggedDashboardChooser<ScoringMode> m_scoringModeChooser;
	private LoggedDashboardChooser<Boolean> m_scoreFirst8;

	// Incompatibility warnings
	private Alert m_doubleDepotAlert, m_doubleOutpostAlert, m_redundantBumpOverride, m_redundantTowerCrossObj2,
			m_redundantTowerCrossObj1, m_redundantPathPlannerPath, m_noPathSelected, m_invalidLeaveObjective,
			m_unableToObtainPath, m_objective2before1;

	// TODO fallback scoring mode????

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

		m_objective1EndCooldown = new LoggedDashboardChooser<>("Autonomous Objective #1 to #2 Cooldown");
		m_objective1EndCooldown.addDefaultOption("0 seconds", 0.0);
		m_objective1EndCooldown.addOption("1 seconds", 1.0);
		m_objective1EndCooldown.addOption("2 seconds", 2.0);
		m_objective1EndCooldown.addOption("3 seconds", 3.0);
		m_objective1EndCooldown.addOption("4 seconds", 4.0);
		m_objective1EndCooldown.addOption("5 seconds", 5.0);

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

		m_parkChooser = new LoggedDashboardChooser<>("Autonomous Parking");
		m_parkChooser.addDefaultOption("No", false);
		m_parkChooser.addOption("Yes", true);

		m_passThroughTowerObj1 = new LoggedDashboardChooser<>("Autonomous Pass Through Tower Objective #1");
		m_passThroughTowerObj1.addDefaultOption("No", false);
		m_passThroughTowerObj1.addOption("Yes", true);

		m_passThroughTowerObj2 = new LoggedDashboardChooser<>("Autonomous Pass Through Tower Objective #2");
		m_passThroughTowerObj2.addDefaultOption("No", false);
		m_passThroughTowerObj2.addOption("Yes", true);

		m_scoringModeChooser = new LoggedDashboardChooser<>("Autonomous Scoring");
		m_scoringModeChooser.addDefaultOption("After Objective #1 and #2", ScoringMode.kObjective1And2);
		m_scoringModeChooser.addOption("After Objective #1", ScoringMode.kObjective1);
		m_scoringModeChooser.addOption("After Objective #2", ScoringMode.kObjective2);
		m_scoringModeChooser.addOption("None", ScoringMode.kNone);

		m_scoreFirst8 = new LoggedDashboardChooser<>("Autonomous Score First 8 Fuel");
		m_scoreFirst8.addDefaultOption("No", false);
		m_scoreFirst8.addOption("Yes", true);

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
		m_redundantBumpOverride = new Alert("Redundant bump override given, but no neutral scoring objective selected.",
				AlertType.kWarning);
		m_redundantTowerCrossObj1 = new Alert("Redundant Objective #1 tower cross given with invalid objectives.",
				AlertType.kWarning);
		m_redundantTowerCrossObj2 = new Alert("Redundant Objective #2 tower cross given with invalid objectives.",
				AlertType.kWarning);
		m_redundantPathPlannerPath = new Alert(
				"Redundant PathPlanner path was selected even though the custom builder is being used.",
				AlertType.kWarning);
		m_noPathSelected = new Alert("PathPlanner mode active, but no PathPlanner path selected!", AlertType.kWarning);
		m_invalidLeaveObjective = new Alert(
				"Redundant leave objective is set as primary when a secondary or climb objective is present. Set primary to none instead.",
				AlertType.kWarning);
		m_doubleOutpostAlert = new Alert(
				"Incompatible autonomous! Primary and secondary objectives both set to outpost.", AlertType.kWarning);
		m_objective2before1 = new Alert("Objective #2 set before Objective #1, possible issues during autonomous!",
				AlertType.kWarning);
	}

	/**
	 * Updates the NetworkTable alerts to show possible problems with the autonomous sequence.
	 */
	public void updateNTAlerts() {
		var obj1 = m_objective1Chooser.get();
		var obj2 = m_objective2Chooser.get();
		var autoType = m_autoTypeChooser.get();

		// Both objectives set to Depot
		m_doubleDepotAlert.set(obj1 == AutoObjective.kScoreDepot && obj2 == AutoObjective.kScoreDepot);
		// Both objectives set to Outpost
		m_doubleOutpostAlert.set(obj1 == AutoObjective.kScoreOutpost && obj2 == AutoObjective.kScoreOutpost);

		// A bump override provided with no neutral objective
		m_redundantBumpOverride.set(!isPrimaryNeutralObjective()
				&& m_forceNeutralCrossSideOverride.get() != AutoBumpCrossingOverride.kNone);

		// Cross to go from the depot to the outpost or vice versa
		boolean objective2CrossBetweenDepotOutpost = (obj1 == AutoObjective.kScoreDepot
				&& obj2 == AutoObjective.kScoreOutpost)
				|| (obj1 == AutoObjective.kScoreOutpost && obj2 == AutoObjective.kScoreDepot);
		// Cross from right bump to the depot (left), inverted logic if a sweep auto is
		// used as that comes out of the bump opposite to the bump it started
		boolean objective2CrossFromRightBumpToDepot = ((isCrossRightObjective1()
				&& obj1 != AutoObjective.kScoreNeutralSweepOver)
				|| (!isCrossRightObjective1() && obj1 == AutoObjective.kScoreNeutralSweepOver))
				&& isPrimaryNeutralObjective() && obj2 == AutoObjective.kScoreDepot;
		// Same as obove, just inverted logic
		boolean objective2CrossFromLeftBumpToOutpost = !((isCrossRightObjective1()
				&& obj1 != AutoObjective.kScoreNeutralSweepOver)
				|| (!isCrossRightObjective1() && obj1 == AutoObjective.kScoreNeutralSweepOver))
				&& isPrimaryNeutralObjective() && obj2 == AutoObjective.kScoreOutpost;

		// Passing through the tower with no real purpose for Objective #2
		m_redundantTowerCrossObj2.set(m_passThroughTowerObj2.get() && !(objective2CrossBetweenDepotOutpost
				|| objective2CrossFromRightBumpToDepot || objective2CrossFromLeftBumpToOutpost));

		// In the beginning, crossing from the right starting point to the depot (left)
		boolean objective1CrossRightToLeftValid = m_startingPointChooser.get() == AutoStartingPoint.kRight
				&& (obj1 == AutoObjective.kScoreDepot || (isPrimaryNeutralObjective() && !isCrossRightObjective1()));
		// In the beginning, crossing from the left starting point to the outpost
		// (right)
		boolean objective1CrossLeftToRightValid = m_startingPointChooser.get() == AutoStartingPoint.kLeft
				&& (obj1 == AutoObjective.kScoreOutpost || ((isPrimaryNeutralObjective() && isCrossRightObjective1())));
		// Only allow passing through for opposite side objectives
		m_redundantTowerCrossObj1.set(
				m_passThroughTowerObj1.get() && !(objective1CrossRightToLeftValid || objective1CrossLeftToRightValid));

		// PathPlanner path selected for the custom builder
		m_redundantPathPlannerPath.set(
				autoType == AutoType.kCustom && !m_ppAutoRoutine.getSendableChooser().getSelected().equals("None"));
		// No path selected for PathPlanner
		m_noPathSelected.set(
				autoType == AutoType.kPathPlanner && m_ppAutoRoutine.getSendableChooser().getSelected().equals("None"));
		// Redundant leave objective, wastes time
		m_invalidLeaveObjective
				.set(obj1 == AutoObjective.kLeave && (obj2 != AutoObjective.kNone || m_scoreFirst8 != null));

		// Objective 2 is set before objective 1 is set
		m_objective2before1.set(
				m_objective2Chooser.get() != AutoObjective.kNone && m_objective1Chooser.get() == AutoObjective.kNone);
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

		// Return early if using a PP routine
		if (autoType == AutoType.kPathPlanner)
			return m_ppAutoRoutine.get().asProxy().withName(m_ppAutoRoutine.get().getName());

		// Resetting the odometry / starting pose
		Command resetOdomCommand = Commands.runOnce(() -> swerve.resetOdometry(getStartingPoseFlipped()));

		// Setting robot objective state
		boolean neutralAlignRightObj1 = isCrossRightObjective1();

		var deferredNeutralAdaptive = new DeferredCommand(() -> {
			var path = TrajectoryUtils2026.generateGridOptimizedPath(getTrajectoryParams(),
					robotContainer.objectDetection.getDetectedInstances());
			if (!path.isPresent()) {
				DriverStation.reportWarning("WARNING: Adaptive auto has failed, no path generated! Using fallback.",
						false);
				Logger.recordOutput("AdaptiveAutoFailedObjects",
						robotContainer.objectDetection.getDetectedInstances().toArray(Pose2d[]::new));
				// Use fallback path instead
				return runIntakeSpeeds(true).andThen(() -> swerve.stop())
						.andThen(runIntakeSpeeds(false))
						.andThen(() -> swerve.stop());
			}

			// Make sure to look in the opposite direction of travel to intake
			CustomPPHolonomicDriveController.overrideRotationFeedback(() -> {
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
			case kScoreNeutral -> crossTower(
					m_startingPointChooser.get() == AutoStartingPoint.kRight && !isCrossRightObjective1())
							.onlyIf(m_passThroughTowerObj1::get)
							// Set up to cross the bump
							.andThen(
									AutoBuilder.pathfindToPoseFlipped(
											neutralAlignRightObj1 ? FieldConstants.kRobotPoseRightBlue
													: FieldConstants.kRobotPoseLeftBlue,
											AutoConstants.kPathConstraints))
							.andThen(passBump(true))
							// Drive straight then back
							.andThen(runIntakeSpeeds(true).alongWith(superstructure.startIntaking()))
							.andThen(() -> swerve.stop())
							.andThen(runIntakeSpeeds(false))
							.andThen(() -> swerve.stop())
							.andThen(Commands.runOnce(() -> superstructure.stopIntake()))
							.andThen(passBump(false))
							.andThen(() -> swerve.stop());
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
					.andThen(crossTower(
							m_startingPointChooser.get() == AutoStartingPoint.kRight && !isCrossRightObjective1())
									.onlyIf(m_passThroughTowerObj1::get))
					// Set up to pass the bump
					.andThen(AutoBuilder.pathfindToPoseFlipped(
							neutralAlignRightObj1
									? FieldConstants.kRobotPoseRightBlue.rotateAround(
											FieldConstants.kRobotPoseRightBlue.getTranslation(), Rotation2d.kPi)
									: FieldConstants.kRobotPoseLeftBlue.rotateAround(
											FieldConstants.kRobotPoseLeftBlue.getTranslation(), Rotation2d.kPi),
							AutoConstants.kPathConstraints))
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
					.finallyDo((interrupt) -> {
						// CustomPPHolonomicDriveController.clearRotationFeedbackOverride();
						// Switch back to localization for teleop if interrupted
						if (!interrupt) return;
						robotContainer.objectDetectionCamRear.setEnabled(false);
						robotContainer.objectDetectionCamRear.setPipeline(VisionConstants.kLocalizationPipelineId);
					});
			case kNone -> Commands.none();
			default -> Commands.none();
		};

		// After going to the neutral zone, would the robot enter through the right
		// bump?
		boolean isRightNeutralReEntry = ((isCrossRightObjective1() && obj1 != AutoObjective.kScoreNeutralSweepOver)
				|| (!isCrossRightObjective1() && obj1 == AutoObjective.kScoreNeutralSweepOver));

		// Secondary objective command
		Command objective2Command = switch (obj2) {
			// Cross tower if appropriate && intake from the depot (either coming from the
			// right bump or outpost)
			case kScoreDepot -> depotTrajectory(true, () -> passTowerObj2
					&& (obj1 == AutoObjective.kScoreOutpost || (isRightNeutralReEntry && isPrimaryNeutralObjective())));
			// Cross tower if appropriate && intake from the outpost (either coming from the
			// left bump or depot)
			case kScoreOutpost -> outpostTrajectory(false, () -> passTowerObj2
					&& (obj1 == AutoObjective.kScoreDepot || (!isRightNeutralReEntry && isPrimaryNeutralObjective())));
			case kNone -> Commands.none();
			default -> Commands.none();
		};

		// Parking command
		Supplier<Command> parking = () -> robotContainer.getAutoManager()
				.getPathfindCommand(path("AutoPark"), AutoConstants.kPathfindSkipTolerance)
				.onlyIf(m_parkChooser::get);

		// Piece together each command
		return resetOdomCommand.andThen(Commands.waitSeconds(kAutoBufferTime))
				.andThen(Commands.either(
						parking.get()
								.andThen(new DeferredCommand(() -> scoreFuel(99999, getPrescorePose(), true),
										Set.of(RobotContainer.getInstance().swerve)).onlyIf(m_scoreFirst8::get)),
						new DeferredCommand(() -> scoreFuel(k8ScoringTimer, getPrescorePose(), true),
								Set.of(RobotContainer.getInstance().swerve)).onlyIf(m_scoreFirst8::get),
						() -> m_objective1Chooser.get() == AutoObjective.kNone
								&& m_objective2Chooser.get() == AutoObjective.kNone && m_parkChooser.get()))
				.andThen(objective1Command)
				.andThen(scoreAfterObjective(
						m_objective2Chooser.get() != AutoObjective.kNone ? getObjective1ScoreTime() : 99999, true)
								.onlyIf(() -> m_objective1Chooser.get() != AutoObjective.kNone))
				.andThen(Commands.waitSeconds(m_objective1EndCooldown.get()))
				.andThen(objective2Command)
				.andThen(parking.get()
						.onlyIf(() -> (m_objective1Chooser.get() != AutoObjective.kNone
								|| m_objective2Chooser.get() != AutoObjective.kNone) && m_parkChooser.get()))
				.andThen(scoreAfterObjective(99999, false)
						.onlyIf(() -> m_objective2Chooser.get() != AutoObjective.kNone))
				.withName("Custom Auto Routine (Starting: " + m_startingPointChooser.get() + " / Objective #1: " + obj1
						+ " / Objective #2: " + obj2 + " / Score: " + m_scoringModeChooser.get()
						+ " / Pass Tower Objective #1: " + passTowerObj1 + " / Pass Tower Objective #2: "
						+ passTowerObj2 + " / Park: " + m_parkChooser.get() + ")");
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
		if (!Robot.isSimulation()) return new PassBumpCommand(RobotContainer.getInstance().swerve,
				isToNeutral ? kPassBumpSpeedsFwdBlue : kPassBumpSpeedsRevBlue);

		return Commands
				.run(() -> RobotContainer.getInstance().swerve
						.runSpeeds(isToNeutral ? getFlippedSpeeds(new ChassisSpeeds(kPassBumpSpeedsFwdBlue, 0, 0))
								: getFlippedSpeeds(new ChassisSpeeds(kPassBumpSpeedsRevBlue, 0, 0)), true))
				.withTimeout(kSimPassBumpTimer);
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
						.getPathfindCommand(path("Outpost"), AutoConstants.kPathfindSkipTolerance))
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

	/** Score after an objective has ended. */
	private Command scoreAfterObjective(double scoreTime, boolean isObjective1) {
		return new DeferredCommand(() -> scoreFuel(scoreTime, getOptimalScoringPose(isObjective1), false), Set.of())
				.onlyIf(() -> m_scoringModeChooser.get() == ScoringMode.kObjective1And2
						|| (isObjective1 && m_scoringModeChooser.get() == ScoringMode.kObjective1)
						|| (!isObjective1 && m_scoringModeChooser.get() == ScoringMode.kObjective2));
	}

	/** Returns the optimal pose to score the first 8 fuel. */
	private Pose2d getPrescorePose() {
		if (isPrimaryNeutralObjective())
			return isCrossRightObjective1() ? AllianceUtil.flipWithAlliance(kScoringPoseRightBlue)
					: AllianceUtil.flipWithAlliance(kScoringPoseLeftBlue);

		if (m_objective1Chooser.get() == AutoObjective.kNone && m_parkChooser.get()) return null;
		if (m_objective1Chooser.get() == AutoObjective.kScoreDepot && !m_redundantTowerCrossObj1.get())
			return AllianceUtil.flipWithAlliance(kScoringPoseLeftBlue);
		else if (m_objective1Chooser.get() == AutoObjective.kScoreDepot && m_redundantTowerCrossObj1.get())
			return AllianceUtil.flipWithAlliance(kScoringPoseRightBlue);
		if (m_objective1Chooser.get() == AutoObjective.kScoreOutpost && !m_redundantTowerCrossObj1.get())
			return AllianceUtil.flipWithAlliance(kScoringPoseRightBlue);
		else if (m_objective1Chooser.get() == AutoObjective.kScoreOutpost && m_redundantTowerCrossObj1.get())
			return AllianceUtil.flipWithAlliance(kScoringPoseLeftBlue);

		return getNearestScoringPose();
	}

	/** Returns the command to start scoring. */
	private Command scoreFuel(double scoreTime, Pose2d scoringPose, boolean isFirst8) {
		return Commands.runOnce(RobotContainer.getInstance()::initAutonScoreFuelCommand)
				// Align yaw during pathfind
				.andThen(Commands.runOnce(() -> {
					CustomPPHolonomicDriveController
							.overrideRotationFeedback(() -> DriveCommands.LatestShootingAngularVelocity);
					CustomPPHolonomicDriveController.setCompleteOverrideRotationOutput(true);
				}))
				.andThen(new DeferredCommand(() -> AutoBuilder.pathfindToPose(scoringPose,
						getScoringConstraints(RobotContainer.getInstance().swerve.getFilteredPose(), scoringPose,
								isFirst8)),
						Set.of()).onlyIf(() -> scoringPose != null))
				// Rotate to setpoint
				.andThen(Commands
						.run(() -> RobotContainer.getInstance().swerve.runSpeeds(0, 0,
								DriveCommands.LatestShootingAngularVelocity, true))
						.until(() -> RobotContainer.getInstance().swerve.ArbitraryPIDAngular.atSetpoint()
								&& RobotContainer.getInstance().hood.isNearSetpoint()
								&& RobotContainer.getInstance().rollers.isNearSetpoint()))
				.andThen(Commands.print("Reached setpoint"))
				// Wait and make sure to stay at the setpoint
				.andThen(Commands.race(Commands.waitSeconds(scoreTime),
						Commands.run(() -> RobotContainer.getInstance().swerve.runSpeeds(0, 0,
								DriveCommands.LatestShootingAngularVelocity, true))))
				.andThen(Commands.print("Seconds waited: " + scoreTime))
				.finallyDo(() -> {
					DriveCommands.LatestShootingAngularVelocity = 0;
					RobotContainer.getInstance().swerve.ArbitraryPIDAngular
							.reset(RobotContainer.getInstance().swerve.getRobotRotation().getRadians());
					CustomPPHolonomicDriveController.clearRotationFeedbackOverride();
					RobotContainer.getInstance().cancelAutonScoreFuelCommand();
					RobotContainer.getInstance().superstructure.setObjectiveOriented(true);
				});
	}

	/** Returns the appropriate scoring path constraints. */
	private PathConstraints getScoringConstraints(Pose2d robot, Pose2d scoringPose, boolean isFirst8) {
		if (isFirst8) return AutoConstants.kPathConstraints;

		if (robot.getTranslation().getDistance(scoringPose.getTranslation()) < kScoringVelocityLimitDistanceThreshold)
			return AutoConstants.kPathConstraintsScoring;
		else return AutoConstants.kPathConstraints;
	}

	/** Returns the closest scoring pose for the hub. */
	private Pose2d getNearestScoringPose() {
		return RobotContainer.getInstance().swerve.getFilteredPose()
				.nearest(List.of(AllianceUtil.flipWithAlliance(kScoringPoseRightBlue),
						AllianceUtil.flipWithAlliance(kScoringPoseLeftBlue)));
	}

	/** Returns the optimal objective scoring pose for the hub. */
	private Pose2d getOptimalScoringPose(boolean isObjective1) {
		// Final scoring, just nearest scoring point
		if (!isObjective1 && !m_parkChooser.get()) return getNearestScoringPose();
		if (!isObjective1 && m_parkChooser.get()) return null;

		// post-Objective 1, nearest scoring pose to the next objective
		if (m_objective2Chooser.get() == AutoObjective.kNone) return getNearestScoringPose();
		if (m_objective2Chooser.get() == AutoObjective.kScoreDepot && !m_passThroughTowerObj2.get())
			return AllianceUtil.flipWithAlliance(kScoringPoseLeftBlue);
		else if (m_objective2Chooser.get() == AutoObjective.kScoreDepot && m_passThroughTowerObj2.get())
			return AllianceUtil.flipWithAlliance(kScoringPoseRightBlue);
		if (m_objective2Chooser.get() == AutoObjective.kScoreOutpost && !m_passThroughTowerObj2.get())
			return AllianceUtil.flipWithAlliance(kScoringPoseRightBlue);
		else if (m_objective2Chooser.get() == AutoObjective.kScoreOutpost && m_passThroughTowerObj2.get())
			return AllianceUtil.flipWithAlliance(kScoringPoseLeftBlue);

		System.out.println("Invalid scoring! Is#1: " + isObjective1 + ", Obj2: " + m_objective2Chooser.get());

		// Fallback
		return getNearestScoringPose();
	}

	/** Returns the scoring time for objective 1. */
	private double getObjective1ScoreTime() {
		if (getObjective1Fuel() == 24) return k24ScoringTimer;
		if (getObjective1Fuel() == 60) return k60ScoringTimer;
		return k8ScoringTimer;
	}

	/** Returns the approximate fuel scored for objective 1. */
	private double getObjective1Fuel() {
		if (m_objective1Chooser.get() == AutoObjective.kScoreDepot
				|| m_objective1Chooser.get() == AutoObjective.kScoreOutpost)
			return 24;
		if (isPrimaryNeutralObjective()) return 60;

		// Could return 8, but fine to return 0
		return 0;
	}

	private boolean isPrimaryNeutralObjective() {
		var obj1 = m_objective1Chooser.get();
		return obj1 == AutoObjective.kScoreNeutral || obj1 == AutoObjective.kScoreNeutralAdaptive
				|| obj1 == AutoObjective.kScoreNeutralLoopOver || obj1 == AutoObjective.kScoreNeutralSweepOver;
	}

	/** Crosses the bump and follows a trajectory in the neutral zone. */
	private Command neutralTrajectory(PathPlannerPath path, boolean neutralAlignRight) {
		return crossTower(m_startingPointChooser.get() == AutoStartingPoint.kRight && !isCrossRightObjective1())
				.onlyIf(m_passThroughTowerObj1::get)
				.andThen(AutoBuilder.pathfindToPoseFlipped(
						neutralAlignRight ? FieldConstants.kRobotPoseRightBlue : FieldConstants.kRobotPoseLeftBlue,
						AutoConstants.kPathConstraints))
				.andThen(Commands.runOnce(() -> RobotContainer.getInstance().cancelAutonScoreFuelCommand()))
				.andThen(passBump(true))
				.andThen(pathfindThenFollow(path))
				.andThen(Commands.runOnce(() -> RobotContainer.getInstance().superstructure.stopIntake()))
				.andThen(passBump(false))
				.andThen(() -> RobotContainer.getInstance().swerve.stop());
	}
}
