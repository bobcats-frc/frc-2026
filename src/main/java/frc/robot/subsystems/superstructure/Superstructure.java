package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.Meters;
import static frc.robot.constants.FieldConstants.path;
import static frc.robot.subsystems.intake.IntakeConstants.kArmOpenedAngle;
import static frc.robot.subsystems.intake.IntakeConstants.kArmPushFuelAngle;
import static frc.robot.subsystems.shooter.hood.HoodConstants.kExitAngleOffset;
import static frc.robot.subsystems.shooter.hood.HoodConstants.kHoodCalibrationAngle;
import static frc.robot.subsystems.shooter.hood.HoodConstants.kHoodCalibrationYaw;
import static frc.robot.subsystems.shooter.hood.HoodConstants.kHoodCentralPivotRobotRelative;
import static frc.robot.subsystems.superstructure.SuperstructureConstants.kAverageShotTime;
import static frc.robot.subsystems.superstructure.SuperstructureConstants.kChassisStateSwitchDebounce;
import static frc.robot.subsystems.superstructure.SuperstructureConstants.kCorralAlignPath;
import static frc.robot.subsystems.superstructure.SuperstructureConstants.kCorralOuttakeTimerLimit;
import static frc.robot.subsystems.superstructure.SuperstructureConstants.kFlashPeriod;
import static frc.robot.subsystems.superstructure.SuperstructureConstants.kFuelCapacity;
import static frc.robot.subsystems.superstructure.SuperstructureConstants.kFuelProjectile;
import static frc.robot.subsystems.superstructure.SuperstructureConstants.kIntakeArmMaxHorizontalExtension;
import static frc.robot.subsystems.superstructure.SuperstructureConstants.kIntakeFlashOffColor;
import static frc.robot.subsystems.superstructure.SuperstructureConstants.kIntakeFlashRollingColor;
import static frc.robot.subsystems.superstructure.SuperstructureConstants.kIntakeRollerVelocityThreshold;
import static frc.robot.subsystems.superstructure.SuperstructureConstants.kLimitK0Hub;
import static frc.robot.subsystems.superstructure.SuperstructureConstants.kLimitK0Pass;
import static frc.robot.subsystems.superstructure.SuperstructureConstants.kLimitK1Hub;
import static frc.robot.subsystems.superstructure.SuperstructureConstants.kLimitK1Pass;
import static frc.robot.subsystems.superstructure.SuperstructureConstants.kLimitVelocityWhenShootingTeleop;
import static frc.robot.subsystems.superstructure.SuperstructureConstants.kPassFuelAcceptDistance;
import static frc.robot.subsystems.superstructure.SuperstructureConstants.kPassTrajOptSteps;
import static frc.robot.subsystems.superstructure.SuperstructureConstants.kPrefireFlashColor;
import static frc.robot.subsystems.superstructure.SuperstructureConstants.kPrefireFlashColorSecondary;
import static frc.robot.subsystems.superstructure.SuperstructureConstants.kPrefireFlashOffColor;
import static frc.robot.subsystems.superstructure.SuperstructureConstants.kPrefireFlashTimeBuffer;
import static frc.robot.subsystems.superstructure.SuperstructureConstants.kPreloadedFuelAmount;
import static frc.robot.subsystems.superstructure.SuperstructureConstants.kPresetShootingParametersBlue;
import static frc.robot.subsystems.superstructure.SuperstructureConstants.kShooterDescriptor;
import static frc.robot.subsystems.superstructure.SuperstructureConstants.kShooterFlashOffColor;
import static frc.robot.subsystems.superstructure.SuperstructureConstants.kShooterNotReadyFlashColor;
import static frc.robot.subsystems.superstructure.SuperstructureConstants.kShooterReadyFlashColor;
import static frc.robot.subsystems.superstructure.SuperstructureConstants.kShooterStateStaleTimeThreshold;
import static frc.robot.subsystems.superstructure.SuperstructureConstants.kShootingFlightTimeFudgeFactors;
import static frc.robot.subsystems.superstructure.SuperstructureConstants.kSwerveIdleVelocityThreshold;

import com.bobcats.lib.auto.LineTrajectoryUtils;
import com.bobcats.lib.auto.LineTrajectoryUtils.Line;
import com.bobcats.lib.auto.LineTrajectoryUtils.OptimizedLine;
import com.bobcats.lib.container.Vector3;
import com.bobcats.lib.control.shooter.ShooterCalculator;
import com.bobcats.lib.control.shooter.ShooterCalculator.ShooterParameters;
import com.bobcats.lib.control.shooter.data.ShooterProjectile;
import com.bobcats.lib.utils.AllianceUtil;
import com.bobcats.lib.utils.Tracer;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.PhysicalParameters;
import frc.robot.constants.Constants.RobotMode;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.feeder.Feeder;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.rollers.Rollers;
import frc.robot.subsystems.swerve.SwerveConstants.AutoConstants;
import frc.robot.subsystems.swerve.SwerveConstants.DriveConstants;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.util.SeasonUtils;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import lombok.AllArgsConstructor;
import org.dyn4j.geometry.Rectangle;
import org.dyn4j.geometry.Vector2;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.IntakeSimulation.IntakeSide;
import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

// State Notes:
// Robot is assumed to have 3 state categories:
// 1. -> Primary: Primary robot actions: INTAKE, OUTTAKE.
// 2. -> Action: Mostly scoring related. E.g. SHOOT_HUB, PASS_ALLIANCE, etc.
// 3. -> Chassis: Whether the robot is above a certain linear (not angular) velocity threshold
// The state diagram can be found in the project's root directory.
// -> Objective Oriented: Whether the robot is actively trying to either shoot or pass fuel

/**
 * The finite-state-machine superstructure subsystem that manages the overall state of the
 * robot.
 */
public class Superstructure extends SubsystemBase {

	// State Enums //

	/** An enum representing the primary states of the robot. */
	@AllArgsConstructor
	public enum PrimaryState {
		kPrimaryIdle("Idle Primary"), kIntaking("Intaking"), kOuttaking("Outtaking");

		public final String display;
	}

	/** An enum representing the action states of the robot. */
	@AllArgsConstructor
	public enum ActionState {
		kActionIdle("Idle Action"), kScoreFuelHub("Score Hub"), kPassFuelAlliance("Pass Alliance Fuel");

		public final String display;
	}

	/** An enum representing the chassis states of the robot. */
	@AllArgsConstructor
	public enum ChassisState {
		kChassisDrive("Driving"), kChassisIdle("Idle Chassis");

		public final String display;
	}

	// Robot state data
	private PrimaryState m_primaryState = PrimaryState.kPrimaryIdle;
	private ActionState m_actionState = ActionState.kActionIdle;
	private ChassisState m_chassisState = ChassisState.kChassisIdle;

	private boolean m_isObjectiveOriented = false;
	private LoggedDashboardChooser<Boolean> m_isFallbackScoring;

	// For testing and calib purposes, overriding setpoints
	public boolean setpointOverride = false;
	public LoggedDashboardChooser<Boolean> overrideRollerSetpointChooser, overrideHoodSetpointChooser;

	private Debouncer m_chassisStateDebouncer = new Debouncer(kChassisStateSwitchDebounce, DebounceType.kBoth);

	private PathPlannerPath m_corralAlignPath;
	private Alert m_alignPathMissingAlert = new Alert("Unable to obtain an alignment path, please reboot!",
			AlertType.kError);

	// Shooter state data
	private final ShooterCalculator m_shooterCalculator;
	private ShooterParameters m_latestParameters = new ShooterParameters(false,
			Rotation2d.fromDegrees(kHoodCalibrationAngle), Rotation2d.kZero, 0, kHoodCalibrationAngle, 0,
			Translation3d.kZero, Vector3.kZero, 0);
	private List<ShooterProjectile> m_projectiles = new ArrayList<>();
	private double m_lastFuelShotSim = 0;
	private double m_lastShooterUpdate;
	public final SuperstructureVisualizer superstructureVisualizer;
	private boolean m_isFeeding = false;

	private IntakeSimulation m_intakeSim;

	private boolean m_isDashboardShotStateFlashOn = false;
	private int m_shotFlashCycle;

	private boolean m_isPrefireStateFlashOn = false;
	private int m_prefireFlashCycle;

	private boolean m_isDashboardIntakeStateFlashOn = false;
	private int m_intakeFlashCycle;

	private Timer m_intakeOscillationTimer = new Timer();
	private static final double kMaxOscillationTime = 3.0;

	private OptimizedLine m_latestPassLineOpt;

	// Subsystems
	private final Feeder m_feeder;
	private final Hood m_hood;
	private final Rollers m_rollers;
	private final Intake m_intake;
	private final SwerveSubsystem m_swerve;

	/**
	 * Constructs a new Superstructure.
	 *
	 * @param feeder  The feeder subsystem.
	 * @param hood    The hood subsytem.
	 * @param rollers The roller subsytem.
	 * @param intake  The intake subsystem.
	 * @param swerve  The swerve subsystem.
	 */
	@SuppressWarnings("resource")
	public Superstructure(Feeder feeder, Hood hood, Rollers rollers, Intake intake, SwerveSubsystem swerve) {
		m_feeder = feeder;
		m_hood = hood;
		m_rollers = rollers;
		m_intake = intake;
		m_swerve = swerve;

		m_shooterCalculator = new ShooterCalculator(kShooterDescriptor);
		superstructureVisualizer = new SuperstructureVisualizer(m_hood, m_intake);

		m_isFallbackScoring = new LoggedDashboardChooser<>("Fallback Scoring Mode");
		m_isFallbackScoring.addDefaultOption("Disabled", false);
		m_isFallbackScoring.addOption("Enabled", true);

		overrideHoodSetpointChooser = new LoggedDashboardChooser<>("Test Override Hood Setpoints");
		overrideHoodSetpointChooser.addDefaultOption("Disabled", false);
		overrideHoodSetpointChooser.addOption("Enabled", true);

		overrideRollerSetpointChooser = new LoggedDashboardChooser<>("Test Override Roller Setpoints");
		overrideRollerSetpointChooser.addDefaultOption("Disabled", false);
		overrideRollerSetpointChooser.addOption("Enabled", true);

		try {
			m_corralAlignPath = PathPlannerPath.fromPathFile(path(kCorralAlignPath));
		} catch (FileVersionException | IOException | ParseException e) {
			m_alignPathMissingAlert.set(true);
			DriverStation.reportWarning("WARNING: Superstructure::new, unable to obtain corral alignment path", false);
			e.printStackTrace();
		}

		if (Robot.isSimulation()) {
			// Assuming the intake covers the whole hopper extension
			Rectangle rect = new Rectangle(kIntakeArmMaxHorizontalExtension, PhysicalParameters.kRobotWidth.in(Meters));
			rect.translate(new Vector2(
					PhysicalParameters.kRobotLength.in(Meters) / 2 + kIntakeArmMaxHorizontalExtension / 2, 0));

			m_intakeSim = IntakeSimulation.OverTheBumperIntake("Fuel", RobotContainer.getInstance().swerveSim,
					PhysicalParameters.kRobotWidth, Meters.of(kIntakeArmMaxHorizontalExtension), IntakeSide.BACK,
					kFuelCapacity);

			// Preloading fuel
			m_intakeSim.setGamePiecesCount(kPreloadedFuelAmount);
			RobotModeTriggers.autonomous()
					.onTrue(Commands.runOnce(() -> m_intakeSim.setGamePiecesCount(kPreloadedFuelAmount)));

			// Update shot trajectories
			var trajectoryThread = new Notifier(() -> {
				try {
					// Avoid concurrent modification issues
					synchronized (m_projectiles) {
						m_projectiles = m_shooterCalculator.updateVisualizer("FuelProjectiles", m_projectiles);
					}
				} catch (Exception e) {}
			});
			trajectoryThread.setName("FuelTrajectoryThread");
			trajectoryThread.startPeriodic(1.0 / 1000.0);
		}

		setName("Superstructure");
	}

	@Override
	public void periodic() {
		Tracer.start("SuperstructurePeriodic");
		// Chassis State //
		if (m_chassisStateDebouncer.calculate(Math.hypot(m_swerve.getChassisSpeedsFieldRelative().vxMetersPerSecond,
				m_swerve.getChassisSpeedsFieldRelative().vyMetersPerSecond) >= kSwerveIdleVelocityThreshold))
			m_chassisState = ChassisState.kChassisDrive;
		else m_chassisState = ChassisState.kChassisIdle;

		if (Robot.isSimulation()) {
			if (m_primaryState == PrimaryState.kIntaking) m_intakeSim.startIntake();
			else m_intakeSim.stopIntake();

			Logger.recordOutput("Superstructure/SimHopperFuelCount", m_intakeSim.getGamePiecesAmount());
		}

		// Flash dashboard for certain states
		Color shooterFlashColor = kShooterFlashOffColor;
		if (m_isDashboardShotStateFlashOn && m_rollers.isNearSetpoint() && m_hood.isNearSetpoint()
				&& m_swerve.ArbitraryPIDAngular.atSetpoint())
			shooterFlashColor = kShooterReadyFlashColor;
		else if (m_isDashboardShotStateFlashOn && !(m_rollers.isNearSetpoint() && m_hood.isNearSetpoint()
				&& m_swerve.ArbitraryPIDAngular.atSetpoint()))
			shooterFlashColor = kShooterNotReadyFlashColor;
		m_shotFlashCycle++;
		if (m_shotFlashCycle >= (int) (kFlashPeriod / Constants.kLoopPeriodSeconds)) {
			m_isDashboardShotStateFlashOn = !m_isDashboardShotStateFlashOn;
			m_shotFlashCycle = 0;
		}

		// Update shooter state even if we haven't enabled objective mode
		if (!m_isObjectiveOriented) updateShooterSetpointState(getAllianceHubLocation());
		// Whether we should start shooting early in order to optimize shot time
		boolean canPrefire = SeasonUtils.getTimeUntilHubShift() <= m_latestParameters.timeOfFlight()
				+ kShootingFlightTimeFudgeFactors.get(getAllianceHubLocation()
						.getDistance(new Translation3d(m_swerve.getFilteredPose().getTranslation())))
				+ kPrefireFlashTimeBuffer && isLatestShooterParamsRecent() && !SeasonUtils.isAllianceHubActive();
		// Flash dashboard if shooter is ready
		Color prefireFlashColor = kPrefireFlashOffColor;
		if (m_isPrefireStateFlashOn && canPrefire) prefireFlashColor = kPrefireFlashColor;
		else if (!m_isPrefireStateFlashOn && canPrefire) prefireFlashColor = kPrefireFlashColorSecondary;
		m_prefireFlashCycle++;
		if (m_prefireFlashCycle >= (int) (kFlashPeriod / Constants.kLoopPeriodSeconds)) {
			m_isPrefireStateFlashOn = !m_isPrefireStateFlashOn;
			m_prefireFlashCycle = 0;
		}

		Color intakeFlashColor = kShooterFlashOffColor;
		if (m_isDashboardIntakeStateFlashOn && Math.abs(m_intake.getRollerVelocity()) >= kIntakeRollerVelocityThreshold)
			intakeFlashColor = kIntakeFlashRollingColor;
		m_intakeFlashCycle++;
		if (m_intakeFlashCycle >= (int) (kFlashPeriod / Constants.kLoopPeriodSeconds)) {
			m_isDashboardIntakeStateFlashOn = !m_isDashboardIntakeStateFlashOn;
			m_intakeFlashCycle = 0;
		}

		if (DriverStation.isDisabled()) {
			shooterFlashColor = kShooterReadyFlashColor;
			intakeFlashColor = kIntakeFlashOffColor;
			prefireFlashColor = kPrefireFlashOffColor;
		} else if (!m_isObjectiveOriented) { shooterFlashColor = kShooterReadyFlashColor; }

		// Logging data
		Logger.recordOutput("Superstructure/RobotState",
				m_primaryState.display + " / " + m_actionState.display + " / " + m_chassisState.display);
		Logger.recordOutput("Superstructure/IsObjectiveOriented", m_isObjectiveOriented);
		Logger.recordOutput("Superstructure/LatestShooterState", m_latestParameters.toString());
		Logger.recordOutput("Superstructure/LastShooterStateUpdate", m_lastShooterUpdate);
		;
		Logger.recordOutput("Superstructure/IsFeeding", m_isFeeding);
		Logger.recordOutput("Superstructure/ShooterReadyColor", shooterFlashColor);
		Logger.recordOutput("Superstructure/IntakeRollerStatus", intakeFlashColor);
		Logger.recordOutput("Superstructure/PrefireColor", prefireFlashColor);
		Logger.recordOutput("CShooterControl/LatestShootingAngVel", DriveCommands.LatestShootingAngularVelocity);
		Logger.recordOutput("CShooterControl/NearSetpoint", m_swerve.ArbitraryPIDAngular.atSetpoint());
		Logger.recordOutput("CShooterControl/Setpoint", m_swerve.ArbitraryPIDAngular.getSetpoint());

		Tracer.finish("SuperstructurePeriodic");
	}

	// State Getters //

	/**
	 * Returns the robot's primary state.
	 *
	 * @return The robot's primary state.
	 */
	public PrimaryState getPrimaryState() { return m_primaryState; }

	/**
	 * Returns the robot's action state.
	 *
	 * @return The robot's action state.
	 */
	public ActionState getActionState() { return m_actionState; }

	/**
	 * Returns the robot's chassis state.
	 *
	 * @return The robot's chassis state.
	 */
	public ChassisState getChassisState() { return m_chassisState; }

	/**
	 * Returns whether mechanisms are at their respective setpoints.
	 *
	 * @return Whether mechanisms are at their respective setpoints.
	 */
	public boolean areRumbleMechanismsAtSetpoints() {
		return m_intake.isArmNearSetpoint() && m_hood.isNearSetpoint() && m_rollers.isNearSetpoint()
				&& m_swerve.ArbitraryPIDAngular.atSetpoint();
	}

	// Primary State Commands //

	/**
	 * Returns the command to start intaking fuel.
	 *
	 * @return The command to start intaking fuel.
	 */
	public Command startIntaking() {
		return Commands.runOnce(() -> m_primaryState = PrimaryState.kIntaking)
				// .andThen(Commands.runOnce(m_backpack::openBackpack))
				// .andThen(Commands.waitUntil(m_backpack::isNearSetpoint))
				.andThen(Commands.parallel(Commands.runOnce(m_intake::openIntake),
						Commands.runOnce(m_intake::runIntakeRollers)))
				.onlyIf(() -> m_primaryState == PrimaryState.kPrimaryIdle || m_primaryState == PrimaryState.kIntaking)
				.withName("Intake.Run");
	}

	/**
	 * Returns the command to start outtaking fuel.
	 *
	 * @return The command to start outtaking fuel.
	 */
	public Command startOuttaking() {
		return Commands.runOnce(() -> m_primaryState = PrimaryState.kOuttaking)
				// .andThen(Commands.runOnce(m_backpack::openBackpack))
				// .andThen(Commands.waitUntil(m_backpack::isNearSetpoint))
				.andThen(Commands.parallel(Commands.runOnce(m_intake::openIntake),
						Commands.runOnce(m_intake::runOuttakeRollers)))
				.onlyIf(() -> m_primaryState == PrimaryState.kPrimaryIdle || m_primaryState == PrimaryState.kOuttaking)
				.withName("Intake.RunReverse");
	}

	/**
	 * Returns the command to stop intaking/outtaking.
	 *
	 * @return The command to stop intaking/outtaking.
	 */
	public Command stopIntake() {
		return Commands.parallel(Commands.runOnce(m_intake::stopRollers).onlyIf(() -> !m_isFeeding))
				// ,Commands.runOnce(m_intake::closeIntake).onlyIf(() -> closeArm)
				.andThen(Commands.waitUntil(m_intake::isArmNearSetpoint))
				// .andThen(m_backpack::closeBackpack)
				.andThen(Commands.runOnce(() -> m_primaryState = PrimaryState.kPrimaryIdle))
				.onlyIf(() -> m_primaryState == PrimaryState.kIntaking || m_primaryState == PrimaryState.kOuttaking)
				.withName("Intake.Stop");
	}

	// Action State Commands //

	/**
	 * Sets whether or not the robot should focus on a game objective such as scoring or passing.
	 *
	 * @param isObjective Whether the robot should focus on an objective.
	 */
	public void setObjectiveOriented(boolean isObjective) {
		m_isObjectiveOriented = isObjective;
		if (!isObjective) m_actionState = ActionState.kActionIdle;
	}

	/**
	 * Returns whether the robot is objective oriented.
	 *
	 * @return Whether the robot is objective oriented.
	 */
	public boolean isObjectiveOriented() { return m_isObjectiveOriented; }

	/**
	 * Returns whether the robot is using fallback scoring presets.
	 *
	 * @return Whether the robot is using fallback scoring presets.
	 */
	public boolean isFallbackScoring() { return m_isFallbackScoring.get(); }

	/**
	 * Returns an objective command that scores into the hub as long as it's ran.
	 *
	 * @param isFallback Whether the fallback shooting parameters should be used in case something
	 *                   goes wrong.
	 * @return The objective scoring command.
	 */
	public Command objectiveScoreHub() {
		return Commands.runOnce(() -> m_actionState = ActionState.kScoreFuelHub)
				.andThen(new DeferredCommand(
						() -> shooterCommand(this::getAllianceHubLocation,
								() -> (SeasonUtils.getTimeUntilHubShift() <= m_latestParameters.timeOfFlight()
										+ kShootingFlightTimeFudgeFactors.get(getAllianceHubLocation().getDistance(
												new Translation3d(m_swerve.getFilteredPose().getTranslation())))
										&& isLatestShooterParamsRecent()) ? true : SeasonUtils.isAllianceHubActive()),
						Set.of(m_rollers, m_hood, m_feeder)))
				.onlyWhile(() -> m_isObjectiveOriented && m_actionState == ActionState.kScoreFuelHub)
				.onlyIf(() -> m_isObjectiveOriented && m_actionState == ActionState.kActionIdle
						&& m_primaryState != PrimaryState.kOuttaking)
				.withName("Shooter.ScoreFuel");
	}

	/**
	 * Returns an objective command that passes fuel to the alliance zone as long as it's ran.
	 *
	 * @return The objective scoring command.
	 */
	public Command objectivePassFuelAlliance() {
		return Commands.runOnce(() -> m_actionState = ActionState.kPassFuelAlliance)
				.andThen(new DeferredCommand(
						() -> shooterCommand(this::getBestAllianceFuelPassLocation,
								() -> !isFallbackScoring() && m_latestPassLineOpt.success()),
						Set.of(m_rollers, m_hood, m_feeder)))
				.onlyWhile(() -> m_isObjectiveOriented && m_actionState == ActionState.kPassFuelAlliance)
				.onlyIf(() -> m_isObjectiveOriented && m_actionState != ActionState.kPassFuelAlliance
						&& m_primaryState != PrimaryState.kOuttaking)
				.withName("Shooter.PassFuel");
	}

	/**
	 * Returns the command to stop the shooter.
	 *
	 * @return The command to stop the shooter.
	 */
	public Command stopShooter() {
		return Commands.runOnce(() -> {
			DriveCommands.LatestShootingAngularVelocity = 0;
			m_rollers.stop();
			m_hood.stop();
			stopFeeding();
			m_actionState = ActionState.kActionIdle;
		}).withName("Shooter.Stop");
	}

	// Miscellaneous Commands //

	/**
	 * Returns the command to align to the corral and start outtaking fuel.
	 *
	 * @return The command to align to the corral and start outtaking fuel.
	 */
	public Command alignCorralOuttake() {
		return stopShooter().andThen(Commands.runOnce(() -> m_intake.openIntake()))
				.alongWith(AutoBuilder.pathfindThenFollowPath(m_corralAlignPath, AutoConstants.kPathConstraints)
						.andThen(startOuttaking().andThen(Commands.waitSeconds(kCorralOuttakeTimerLimit))
								.andThen(stopIntake())))
				.alongWith(Commands.run(() -> setObjectiveOriented(false)))
				.onlyIf(() -> m_primaryState == PrimaryState.kPrimaryIdle || m_primaryState == PrimaryState.kOuttaking)
				.onlyWhile(
						() -> m_primaryState == PrimaryState.kPrimaryIdle || m_primaryState == PrimaryState.kOuttaking);
	}

	/**
	 * Returns the absolute maximum chassis speeds at the given state.
	 *
	 * @return The chassis speed limit.
	 */
	public double getChassisLimitVelocity() {
		if (!kLimitVelocityWhenShootingTeleop || DriverStation.isAutonomous())
			return DriveConstants.kMaxSpeedMetersPerSecond;
		if (m_actionState == ActionState.kScoreFuelHub) {
			double d = getAllianceHubLocation().toTranslation2d()
					.getDistance(m_swerve.getFilteredPose().getTranslation());
			return kLimitK0Hub + kLimitK1Hub * d * d / (1 + d * d);
		} else if (m_actionState == ActionState.kPassFuelAlliance) {
			double d = m_latestPassLineOpt.optimizedLine()
					.end()
					.getDistance(m_swerve.getFilteredPose().getTranslation());
			return kLimitK0Pass + kLimitK1Pass * d * d / (1 + d * d);
		}

		return DriveConstants.kMaxSpeedMetersPerSecond;
	}

	/**
	 * Returns the latest shooter parameters.
	 *
	 * @return The latest shooter parameters.
	 */
	public ShooterParameters getLatestShooterParameters() { return m_latestParameters; }

	/**
	 * Returns whether the latest shot parameters are stale or not.
	 *
	 * @return Whether the latest shot parameters are stale or not.
	 */
	public boolean isLatestShooterParamsRecent() {
		return Timer.getFPGATimestamp() - m_lastShooterUpdate <= kShooterStateStaleTimeThreshold;
	}

	/**
	 * Returns the timestamp of the last update to the shooter parameters.
	 *
	 * @return The timestamp of the last update to the shooter parameters.
	 */
	public double getLastShooterParameterUpdate() { return m_lastShooterUpdate; }

	// Private Utils //

	/** Starts feeding the shooter. */
	private void startFeeding() {
		if (m_isFeeding) return;
		m_feeder.feed();
		m_intake.runIntakeRollers();
		m_isFeeding = true;
	}

	/** Stops feeding the shooter. */
	private void stopFeeding() {
		if (!m_isFeeding) return;
		m_feeder.stop();
		if (m_primaryState != PrimaryState.kIntaking) m_intake.stopRollers();
		m_isFeeding = false;
	}

	/** Runs the shooter to hit the given target. */
	private Command shooterCommand(Supplier<Translation3d> target, BooleanSupplier firingCondition) {
		BooleanSupplier canFire = () -> isLatestShooterParamsRecent() && m_latestParameters != null
				&& m_latestParameters.isValid() && m_hood.isNearSetpoint() && m_swerve.ArbitraryPIDAngular.atSetpoint()
				&& m_rollers.isNearSetpoint() && firingCondition.getAsBoolean();

		if (Robot.kRobotMode == RobotMode.kSim) return Commands.run(() -> {
			// Continuously update shooter setpoints
			updateShooterSetpointState(target.get());
			if (m_latestParameters != null && m_latestParameters.isValid()) {
				if (!overrideRollerSetpointChooser.get())
					m_rollers.setRollerVelocity(m_latestParameters.rollerSpeedsRPM());
				if (!overrideHoodSetpointChooser.get())
					m_hood.setHoodAngle(kExitAngleOffset - m_latestParameters.hoodAngleDegs());
			}

			// Update commanded shot angular velocity
			DriveCommands.LatestShootingAngularVelocity = m_swerve.ArbitraryPIDAngular.calculate(
					m_swerve.getRobotRotation().getRadians(),
					getLatestShooterParameters().turretAngleField().getRadians() - Math.toRadians(kHoodCalibrationYaw));

			// Feed only if the shot is possible
			if (canFire.getAsBoolean()) startFeeding();

			// Spawn projectile if there are game pieces in the hopper
			if (canFire.getAsBoolean() && Timer.getFPGATimestamp() - m_lastFuelShotSim >= kAverageShotTime
					&& m_intakeSim.getGamePiecesAmount() > 0) {
				// startFeeding();

				// Make sure to use current state values, and not the ideal values
				var currState = m_shooterCalculator.computeCurrentExitParameters(m_latestParameters,
						kExitAngleOffset - m_hood.getAngle(), kHoodCalibrationYaw, m_rollers.getVelocity(),
						new Pose3d(RobotContainer.getInstance().swerveSim.getSimulatedDriveTrainPose()),
						RobotContainer.getInstance().swerveSim.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
						kFuelProjectile);
				m_lastFuelShotSim = Timer.getFPGATimestamp();
				m_projectiles.add(new ShooterProjectile(kFuelProjectile, currState.exitVelocityVec3(),
						new Pose3d(currState.exitPose(), Rotation3d.kZero)));
				m_intakeSim.obtainGamePieceFromIntake();
			}
		})
				// Stop feeder when unable to fire
				.alongWith(Commands.run(() -> { if (!canFire.getAsBoolean()) stopFeeding(); }))
				.alongWith(Commands.run(() -> {
					// Oscillate intake
					if (m_primaryState != PrimaryState.kIntaking && m_primaryState != PrimaryState.kOuttaking
							&& canFire.getAsBoolean()) {
						if (m_intake.getLatestArmSetpoint() == kArmPushFuelAngle && (m_intake.isArmNearSetpoint()
								|| m_intakeOscillationTimer.hasElapsed(kMaxOscillationTime))) {
							m_intake.openIntake();
							m_intakeOscillationTimer.restart();
						} else if (m_intake.getLatestArmSetpoint() == kArmOpenedAngle && (m_intake.isArmNearSetpoint()
								|| m_intakeOscillationTimer.hasElapsed(kMaxOscillationTime))) {
									m_intake.intakePushAngle();
									m_intakeOscillationTimer.restart();
								} else
							if (m_intake.getLatestArmSetpoint() != kArmOpenedAngle
									&& m_intake.getLatestArmSetpoint() != kArmPushFuelAngle) {
										m_intake.openIntake();
									} else
								if (!m_intakeOscillationTimer.isRunning()) { m_intakeOscillationTimer.start(); }
					} else if (m_primaryState != PrimaryState.kIntaking && m_primaryState != PrimaryState.kOuttaking
							&& !canFire.getAsBoolean()) {
								m_intake.stopIntakeArm();
								m_intakeOscillationTimer.stop();
								m_intakeOscillationTimer.reset();
							}
				}))
				.onlyWhile(() -> m_isObjectiveOriented)
				.finallyDo(() -> {
					DriveCommands.LatestShootingAngularVelocity = 0;
					m_rollers.stop();
					m_hood.stop();
					stopFeeding();
					m_actionState = ActionState.kActionIdle;
					m_intakeOscillationTimer.stop();
					m_intakeOscillationTimer.reset();
				});

		return Commands.run(() -> {
			// Continuously update shooter setpoints
			updateShooterSetpointState(target.get());
			if (m_latestParameters != null && m_latestParameters.isValid()) {
				if (!overrideRollerSetpointChooser.get())
					m_rollers.setRollerVelocity(m_latestParameters.rollerSpeedsRPM());
				if (!overrideHoodSetpointChooser.get())
					m_hood.setHoodAngle(kExitAngleOffset - m_latestParameters.hoodAngleDegs());
			}

			// Update commanded shot angular velocity
			DriveCommands.LatestShootingAngularVelocity = m_swerve.ArbitraryPIDAngular.calculate(
					m_swerve.getRobotRotation().getRadians(),
					getLatestShooterParameters().turretAngleField().getRadians() - Math.toRadians(kHoodCalibrationYaw));

			// Shoot if possible by enabling the feeder
			if (canFire.getAsBoolean()) startFeeding();
		})
				// Stop feeder when unable to fire
				.alongWith(Commands.run(() -> { if (!canFire.getAsBoolean()) stopFeeding(); }))
				.alongWith(Commands.run(() -> {
					// Oscillate intake
					if (m_primaryState != PrimaryState.kIntaking && m_primaryState != PrimaryState.kOuttaking
							&& canFire.getAsBoolean()) {
						if (m_intake.getLatestArmSetpoint() == kArmPushFuelAngle && (m_intake.isArmNearSetpoint()
								|| m_intakeOscillationTimer.hasElapsed(kMaxOscillationTime))) {
							m_intake.openIntake();
							m_intakeOscillationTimer.restart();
						} else if (m_intake.getLatestArmSetpoint() == kArmOpenedAngle && (m_intake.isArmNearSetpoint()
								|| m_intakeOscillationTimer.hasElapsed(kMaxOscillationTime))) {
									m_intake.intakePushAngle();
									m_intakeOscillationTimer.restart();
								} else
							if (m_intake.getLatestArmSetpoint() != kArmOpenedAngle
									&& m_intake.getLatestArmSetpoint() != kArmPushFuelAngle) {
										m_intake.openIntake();
									} else
								if (!m_intakeOscillationTimer.isRunning()) { m_intakeOscillationTimer.start(); }
					} else if (m_primaryState != PrimaryState.kIntaking && m_primaryState != PrimaryState.kOuttaking
							&& !canFire.getAsBoolean()) {
								m_intake.stopIntakeArm();
								m_intakeOscillationTimer.stop();
								m_intakeOscillationTimer.reset();
							}
				}))
				.onlyWhile(() -> m_isObjectiveOriented)
				.finallyDo(() -> {
					DriveCommands.LatestShootingAngularVelocity = 0;
					m_rollers.stop();
					m_hood.stop();
					stopFeeding();
					m_intakeOscillationTimer.stop();
					m_intakeOscillationTimer.reset();
					m_actionState = ActionState.kActionIdle;
				});
	}

	/** Recalculates the shooter parameters. */
	private void updateShooterSetpointState(Translation3d target) {
		// Update timer beforehand
		m_lastShooterUpdate = Timer.getFPGATimestamp();
		if (!isFallbackScoring()) {
			var hubPose = new Pose3d(target, Rotation3d.kZero);
			m_latestParameters = m_shooterCalculator.updateParameters(hubPose.getTranslation(),
					new Pose3d(m_swerve.getFilteredPose()), m_swerve.getRobotRotation(),
					m_swerve.getChassisSpeedsFieldRelative(), kFuelProjectile);
		} else {
			// Use fallback parameters if enabled
			m_latestParameters = flipParametersAlliance(kPresetShootingParametersBlue);
		}
	}

	/** Returns the alliance's hub location. */
	private Translation3d getAllianceHubLocation() {
		return AllianceUtil.flipWithAlliance(FieldConstants.kBlueHubCenterPose);
	}

	/** Returns the most optimized point to pass the fuel towards. */
	private Translation3d getBestAllianceFuelPassLocation() {
		Tracer.start("OptimizePassTrajectory");

		Line netLine = new Line(
				AllianceUtil.flipWithAlliance(new Translation3d(FieldConstants.kBlueAllianceNetRightPoint))
						.toTranslation2d(),
				AllianceUtil.flipWithAlliance(new Translation3d(FieldConstants.kBlueAllianceNetLeftPoint))
						.toTranslation2d());
		Line passLine = new Line(
				AllianceUtil.flipWithAlliance(new Translation3d(FieldConstants.kBlueAllianceZoneFuelPassLineRightPoint))
						.toTranslation2d(),
				AllianceUtil.flipWithAlliance(new Translation3d(FieldConstants.kBlueAllianceZoneFuelPassLineLeftPoint))
						.toTranslation2d());
		// Performance should be fine? 0.1 - 1ms???
		OptimizedLine opt = LineTrajectoryUtils.optimizeUntilNoIntersect(m_swerve.getFilteredPose()
				.transformBy(new Transform2d(kHoodCentralPivotRobotRelative.getTranslation().toTranslation2d(),
						Rotation2d.kZero))
				.getTranslation(), List.of(netLine), passLine, kPassTrajOptSteps, kPassFuelAcceptDistance);
		Tracer.finish("OptimizePassTrajectory");
		m_latestPassLineOpt = opt;

		return new Translation3d(opt.optimizedLine().end());
	}

	/** Flips the shooter parameters based on alliance. */
	private ShooterParameters flipParametersAlliance(ShooterParameters blueRelativeParams) {
		if (!AllianceUtil.isRedAlliance()) return blueRelativeParams;

		return new ShooterParameters(blueRelativeParams.isValid(),
				blueRelativeParams.turretAngleField().plus(Rotation2d.kPi), blueRelativeParams.turretAngleRobot(),
				blueRelativeParams.hoodAngleDegs(), blueRelativeParams.rollerSpeedsRPM(),
				blueRelativeParams.timeOfFlight(), AllianceUtil.flipWithAlliance(blueRelativeParams.exitPose()),
				new Vector3(-blueRelativeParams.exitVelocityVec3().x, blueRelativeParams.exitVelocityVec3().y,
						blueRelativeParams.exitVelocityVec3().z),
				blueRelativeParams.exitVelocityMetersPerSec());
	}
}
