package frc.robot;

import static frc.robot.constants.FieldConstants.path;
import static frc.robot.constants.OIConstants.kDriverAutomaticClimbKeybind;
import static frc.robot.constants.OIConstants.kDriverController;
import static frc.robot.constants.OIConstants.kDriverExtendClimbKeybind;
import static frc.robot.constants.OIConstants.kDriverRetractClimbKeybind;
import static frc.robot.constants.OIConstants.kDriverSlowModeKeybind;
import static frc.robot.constants.OIConstants.kDriverXBrakeKeybind;
import static frc.robot.constants.OIConstants.kDriverZeroHeadingKeybind;
import static frc.robot.constants.OIConstants.kOperatorController;
import static frc.robot.constants.OIConstants.kOperatorCorralKeybind;
import static frc.robot.constants.OIConstants.kOperatorIntakeKeybind;
import static frc.robot.constants.OIConstants.kOperatorOuttakeKeybind;
import static frc.robot.constants.OIConstants.kOperatorPassFuelKeybind;
import static frc.robot.constants.OIConstants.kOperatorScoreHubKeybind;
import static frc.robot.constants.OIConstants.kOperatorToggleObjectiveKeybind;
import static frc.robot.subsystems.climb.ClimbConstants.kHasClimb;
import static frc.robot.subsystems.shooter.hood.HoodConstants.kHoodCalibrationAngle;
import static frc.robot.subsystems.shooter.turret.TurretConstants.kTurretCalibrationAngle;

import com.bobcats.lib.auto.AutonomousManager;
import com.bobcats.lib.auto.AutonomousManager.PathfindStrategy;
import com.bobcats.lib.subsystem.objectDetection.ObjectDetectionIO;
import com.bobcats.lib.subsystem.objectDetection.ObjectDetectionSubsystem;
import com.bobcats.lib.subsystem.objectDetection.io.ObjectDetectionIOLimelight;
import com.bobcats.lib.subsystem.objectDetection.io.ObjectDetectionIOPhotonVisionSim;
import com.bobcats.lib.subsystem.vision.LibVisionIO;
import com.bobcats.lib.subsystem.vision.io.LibVisionIOLimelight;
import com.bobcats.lib.subsystem.vision.io.LibVisionIOPhotonVisionSim;
import com.bobcats.lib.utils.AllianceUtil;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.AutonomousBuilder;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.constants.Constants;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.OIConstants;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.io.ClimbIO;
import frc.robot.subsystems.climb.io.ClimbIOKraken;
import frc.robot.subsystems.climb.io.ClimbIOSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.io.IntakeIO;
import frc.robot.subsystems.intake.io.IntakeIOKraken;
import frc.robot.subsystems.intake.io.IntakeIOSim;
import frc.robot.subsystems.shooter.feeder.Feeder;
import frc.robot.subsystems.shooter.feeder.io.FeederIO;
import frc.robot.subsystems.shooter.feeder.io.FeederIOSim;
import frc.robot.subsystems.shooter.feeder.io.FeederIOSparkMax;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.hood.io.HoodIO;
import frc.robot.subsystems.shooter.hood.io.HoodIOKraken;
import frc.robot.subsystems.shooter.hood.io.HoodIOSim;
import frc.robot.subsystems.shooter.rollers.Rollers;
import frc.robot.subsystems.shooter.rollers.io.RollerIO;
import frc.robot.subsystems.shooter.rollers.io.RollerIOKraken;
import frc.robot.subsystems.shooter.rollers.io.RollerIOSim;
import frc.robot.subsystems.shooter.turret.Turret;
import frc.robot.subsystems.shooter.turret.io.TurretIO;
import frc.robot.subsystems.shooter.turret.io.TurretIOKraken;
import frc.robot.subsystems.shooter.turret.io.TurretIOSim;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.Superstructure.PrimaryState;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveConstants.AutoConstants;
import frc.robot.subsystems.swerve.SwerveConstants.DriveConstants;
import frc.robot.subsystems.swerve.SwerveConstants.SwerveSimulationConstants;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.swerve.VisionConstants;
import frc.robot.subsystems.swerve.gyro.Gyro;
import frc.robot.subsystems.swerve.gyro.GyroIO;
import frc.robot.subsystems.swerve.gyro.GyroIOPigeon2;
import frc.robot.subsystems.swerve.gyro.GyroIOSim;
import frc.robot.subsystems.swerve.module.SwerveModuleIO;
import frc.robot.subsystems.swerve.module.SwerveModuleIOKraken;
import frc.robot.subsystems.swerve.module.SwerveModuleIOSim;
import java.util.Comparator;
import java.util.function.Supplier;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.VisionTargetSim;

/**
 * The main robot container instance.
 */
public class RobotContainer {
	// Singleton instance
	private static RobotContainer instance;

	// Subsystems //

	// Swerve
	public SwerveSubsystem swerve;
	public SwerveDriveSimulation swerveSim;

	// Hopper systems
	// public Backpack backpack;
	public Intake intake;

	// Shooter systems
	public Feeder feeder;
	public Hood hood;
	public Rollers rollers;
	public Turret turret;

	// Generic systems
	public Superstructure superstructure;
	public Climb climb;
	public ObjectDetectionIO objectDetectionCamRear;
	public ObjectDetectionSubsystem objectDetection;

	private AutonomousManager m_autoManager;
	private AutonomousBuilder m_autoSequenceBuilder;
	// Autonomous scoring command, in order to not hang the auto routine when called
	private Command m_autonScoreFuelCommand = Commands.none();

	// Tests
	private LoggedDashboardChooser<Supplier<Command>> m_testChooser = new LoggedDashboardChooser<>(
			"Identification Test Chooser");

	// Controller alerts
	private Alert m_operatorControlWarning = new Alert("The operator controller is disconnected!", AlertType.kError);
	private Alert m_driverControlWarning = new Alert("The driver controller is disconnected!", AlertType.kError);

	/** Constructs a new RobotContainer. */
	public RobotContainer() {
		instance = this;

		// Initialize subsystems based on robot mode
		initSwerveAndVision();
		initShooter();
		initIntake();
		initSuperstructure();

		// Autonomous manager
		initAutoManager();

		// Register named commands after subsystems
		registerNamedCommands();

		// Robot identification routines
		createTests();

		// Register OI keybinds
		attachKeybinds();

		// Calibrate all encoders
		// zeroSubsytemEncoders();

		// Mode triggers
		// Calibrate all encoders when autonomous is starting, then schedule the auto
		// routine
		RobotModeTriggers.autonomous()
				.onTrue(Commands.runOnce(this::zeroSubsytemEncoders)
						.andThen(Commands.runOnce(m_autoManager::scheduleAuto)));
		RobotModeTriggers.autonomous().onFalse(Commands.runOnce(m_autoManager::cancelAuto));

		// Extend climber in teleop when in climbing state to unclimb
		if (kHasClimb) RobotModeTriggers.teleop()
				.onTrue(superstructure.climbExtend()
						.onlyIf(() -> superstructure.getPrimaryState() == PrimaryState.kClimbing));

		// Disable objective oriented mode on disable
		RobotModeTriggers.disabled()
				.onTrue(Commands.runOnce(() -> superstructure.setObjectiveOriented(false))
						.andThen(superstructure.stopShooter())
						.ignoringDisable(true));
	}

	/** Registers all controller keybinds. */
	private void attachKeybinds() {
		// Driver Keybinds //

		// Chassis keybinds
		swerve.setDefaultCommand(DriveCommands.joystickDrive(swerve,
				() -> MathUtil.applyDeadband(kDriverController.getLeftY(), OIConstants.kDriverControllerDeadband),
				() -> MathUtil.applyDeadband(kDriverController.getLeftX(), OIConstants.kDriverControllerDeadband),
				() -> MathUtil.applyDeadband(-kDriverController.getRightX(), OIConstants.kDriverControllerDeadband)));

		// Zero gyro by aligning to the alliance driver station with the robot facing
		// away from the DS
		// (intake is towards the DS) in the case of too much gyro drift
		kDriverZeroHeadingKeybind.onTrue(Commands.runOnce(swerve::zeroHeadingFieldRelative).ignoringDisable(true));
		// Toggle slow mode
		kDriverSlowModeKeybind.onTrue(Commands.runOnce(() -> swerve.setSlowMode(!swerve.isSlowMode())));
		// Stop the robot
		kDriverXBrakeKeybind.whileTrue(Commands.run(swerve::setX));
		// Climber keybinds
		if (kHasClimb) {
			kDriverExtendClimbKeybind.onTrue(superstructure.climbExtend());
			kDriverRetractClimbKeybind.onTrue(superstructure.climbRetract());
			kDriverAutomaticClimbKeybind.whileTrue(superstructure.alignClimberAndClimbCommand()
					.handleInterrupt(
							() -> { System.out.println("Align & Climb was interrupted!"); climb.extendArm(); }));
		}

		// Operator Keybinds //

		// Objective buttons
		kOperatorToggleObjectiveKeybind.onTrue(Commands.parallel(
				Commands.runOnce(() -> superstructure.setObjectiveOriented(!superstructure.isObjectiveOriented())),
				superstructure.stopShooter()
						.andThen(Commands.parallel(
								Commands.runOnce(() -> turret.setTurretAngle(kTurretCalibrationAngle)),
								Commands.runOnce(() -> hood.setHoodAngle(kHoodCalibrationAngle))))));
		kOperatorScoreHubKeybind.onTrue(superstructure.objectiveScoreHub());
		kOperatorPassFuelKeybind.onTrue(superstructure.objectivePassFuelAlliance());
		// Intake bindings
		kOperatorIntakeKeybind.whileTrue(superstructure.startIntaking());
		kOperatorIntakeKeybind.onFalse(
				superstructure.stopIntake().onlyIf(() -> superstructure.getPrimaryState() == PrimaryState.kIntaking));
		// Outtake bindings
		kOperatorOuttakeKeybind.whileTrue(superstructure.startOuttaking());
		kOperatorOuttakeKeybind.onFalse(
				superstructure.stopIntake().onlyIf(() -> superstructure.getPrimaryState() == PrimaryState.kOuttaking));
		kOperatorCorralKeybind.whileTrue(superstructure.alignCorralOuttake());
		kOperatorCorralKeybind.onFalse(superstructure.stopIntake());
	}

	/** Registers the NamedCommands for PathPlanner. */
	private void registerNamedCommands() {
		// Intake commands
		NamedCommands.registerCommand("OpenIntake", superstructure.startIntaking());
		NamedCommands.registerCommand("CloseIntake", superstructure.stopIntake());

		// Scoring commands
		NamedCommands.registerCommand("StartScoringFuel", Commands.runOnce(this::initAutonScoreFuelCommand));
		NamedCommands.registerCommand("StopScoringFuel",
				Commands.runOnce(this::cancelAutonScoreFuelCommand)
						.andThen(Commands.runOnce(() -> superstructure.setObjectiveOriented(false)))
						.andThen(superstructure.stopShooter())
						.andThen(Commands.parallel(
								Commands.runOnce(() -> turret.setTurretAngle(kTurretCalibrationAngle)),
								Commands.runOnce(() -> hood.setHoodAngle(kHoodCalibrationAngle)))));

		// Climb commands
		NamedCommands.registerCommand("ClimbExtend", superstructure.climbExtend());
		NamedCommands.registerCommand("ClimbRetract", superstructure.climbRetract());
		NamedCommands.registerCommand("AlignAndClimb", superstructure.alignClimberAndClimbCommand());
		NamedCommands.registerCommand("ClimbSequenced", superstructure.climbSequenceAuton());

		NamedCommands.registerCommand("PathfindDepot",
				m_autoManager.getPathfindCommand(path("Depot"), AutoConstants.kPathfindSkipTolerance));
		NamedCommands.registerCommand("PathfindOutpost",
				m_autoManager.getPathfindCommand(path("Outpost"), AutoConstants.kPathfindSkipTolerance));

		// Finally create the sequence builder
		m_autoSequenceBuilder = new AutonomousBuilder();
		m_autoManager.setCustomRoutineBuilder(m_autoSequenceBuilder);
		m_autoManager.setRobotPoseSupplier(swerve::getFilteredPose);
	}

	/** Creates robot identification tests. */
	private void createTests() {
		m_testChooser.addDefaultOption("None", () -> Commands.none());
		// Swerve Characterization
		m_testChooser.addOption("Swerve Drive Feedforward Characterization",
				() -> DriveCommands.feedforwardCharacterization(swerve));
		m_testChooser.addOption("Swerve Drive Skew Coefficient Characterization",
				() -> DriveCommands.skewCharacterization(swerve));
		m_testChooser.addOption("Swerve Drive Wheel Radius Characterization",
				() -> DriveCommands.wheelRadiusCharacterization(swerve));
		m_testChooser.addOption("Swerve Drive Gather SysId Data", () -> DriveCommands.sysIdFullTest(swerve,
				SwerveConstants.kDynamicTestTimeLimit, SwerveConstants.kQuasistaticTestTimeLimit));
		m_testChooser.addOption("Swerve Drive Wheel COF Characterization",
				() -> DriveCommands.identifyWheelCOF(swerve));
		// Vision Calibration
		m_testChooser.addOption("Vision Camera Offset Calibration (Right LL)", () -> DriveCommands
				.calibrateVisionCameraOffset(swerve, VisionConstants.kLLIndex_Right, Rotation2d.kPi));
		m_testChooser.addOption("Vision Camera Offset Calibration (Left LL)", () -> DriveCommands
				.calibrateVisionCameraOffset(swerve, VisionConstants.kLLIndex_Left, Rotation2d.kZero));

		// Add mechanism tests
		m_testChooser.addOption("SysId Turret",
				() -> turret.sysIdQuasistatic(Direction.kForward)
						.andThen(turret.sysIdQuasistatic(Direction.kReverse))
						.andThen(turret.sysIdDynamic(Direction.kForward))
						.andThen(turret.sysIdDynamic(Direction.kReverse)));
		m_testChooser.addOption("SysId Rollers",
				() -> rollers.sysIdQuasistatic(Direction.kForward)
						.andThen(rollers.sysIdQuasistatic(Direction.kReverse))
						.andThen(rollers.sysIdDynamic(Direction.kForward))
						.andThen(rollers.sysIdDynamic(Direction.kReverse)));
		m_testChooser.addOption("SysId Hood (Q-F)", () -> hood.sysIdQuasistatic(Direction.kForward));
		m_testChooser.addOption("SysId Hood (Q-R)", () -> hood.sysIdQuasistatic(Direction.kReverse));
		m_testChooser.addOption("SysId Hood (D-F)", () -> hood.sysIdDynamic(Direction.kForward));
		m_testChooser.addOption("SysId Hood (D-R)", () -> hood.sysIdDynamic(Direction.kReverse));

	}

	/** Initializes the autonomous routine manager. */
	private void initAutoManager() {
		m_autoManager = new AutonomousManager(AutoConstants.kPathfindPoses, AutoConstants.kRoutineCommands,
				AutoConstants.kPathConstraints, false);
		m_autoManager.setPathfindStrategy(PathfindStrategy.kPrecisionPathfind);
	}

	// Mechanism Initializations //

	/** Initializes the chassis. */
	private void initSwerveAndVision() {
		switch (Robot.kRobotMode) {
			case kReal:
				swerveSim = null;
				Gyro gyro = new Gyro(new GyroIOPigeon2(DriveConstants.kPigeon2CanId, SwerveConstants.kDrivetrainBus));

				swerve = new SwerveSubsystem(new SwerveModuleIOKraken(0), new SwerveModuleIOKraken(1),
						new SwerveModuleIOKraken(2), new SwerveModuleIOKraken(3), gyro, null,
						VisionConstants.kOdometryStdDevs,
						new LibVisionIOLimelight(VisionConstants.kLLName_Right,
								() -> Rotation2d.fromDegrees(gyro.getInputs().yawDegrees),
								VisionConstants.kRobotToCamera_Right),
						new LibVisionIOLimelight(VisionConstants.kLLName_Left,
								() -> Rotation2d.fromDegrees(gyro.getInputs().yawDegrees),
								VisionConstants.kRobotToCamera_Left));
				// Arbitrary PIDs
				swerve.ArbitraryPIDx = new PIDController(AutoConstants.kPathDriveP, 0, AutoConstants.kPathDriveD);
				swerve.ArbitraryPIDy = new PIDController(AutoConstants.kPathDriveP, 0, AutoConstants.kPathDriveD);
				swerve.ArbitraryPIDAngular = new PIDController(AutoConstants.kPathTurnP, 0, AutoConstants.kPathTurnD);
				swerve.ArbitraryPIDAngular.enableContinuousInput(-Math.PI, Math.PI);

				// Initialize the object detection
				objectDetectionCamRear = new ObjectDetectionIOLimelight(VisionConstants.kLLName_Left,
						VisionConstants.kRobotToCamera_Left.getTranslation().toTranslation2d(), swerve::getFilteredPose,
						VisionConstants.kRobotToCamera_Left.getZ(),
						Math.toDegrees(VisionConstants.kRobotToCamera_Left.getRotation().getZ()),
						Math.toDegrees(VisionConstants.kRobotToCamera_Left.getRotation().getY()),
						FieldConstants.kFuelDiameterNominal / 2.0, FieldConstants.kFuelDiameterNominal / 2.0,
						Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY, VisionConstants.kObjectDetectionMaxYaw);
				objectDetection = new ObjectDetectionSubsystem("Fuel", objectDetectionCamRear);
				break;

			case kSim:
				swerveSim = new SwerveDriveSimulation(SwerveSimulationConstants.kSwerveSimConfig,
						AllianceUtil.flipWithAlliance(Constants.kStartingPose));
				SimulatedArena.getInstance().addDriveTrainSimulation(swerveSim);

				swerve = new SwerveSubsystem(new SwerveModuleIOSim(swerveSim.getModules()[0]),
						new SwerveModuleIOSim(swerveSim.getModules()[1]),
						new SwerveModuleIOSim(swerveSim.getModules()[2]),
						new SwerveModuleIOSim(swerveSim.getModules()[3]),
						new Gyro(new GyroIOSim(swerveSim.getGyroSimulation())), swerveSim::setSimulationWorldPose,
						VisionConstants.kOdometryStdDevs,
						new LibVisionIOPhotonVisionSim(VisionConstants.kLLName_Right,
								VisionConstants.kRobotToCamera_Right, null,
								() -> superstructure.superstructureVisualizer
										.getRobotPoseWithHeight(swerveSim.getSimulatedDriveTrainPose()),
								VisionConstants.kSimCameraProperties),
						new LibVisionIOPhotonVisionSim(VisionConstants.kLLName_Left,
								VisionConstants.kRobotToCamera_Left, null,
								() -> superstructure.superstructureVisualizer
										.getRobotPoseWithHeight(swerveSim.getSimulatedDriveTrainPose()),
								VisionConstants.kSimCameraProperties));
				// Arbitrary PIDs
				swerve.ArbitraryPIDx = new PIDController(AutoConstants.kPathDriveP, 0, AutoConstants.kPathDriveD);
				swerve.ArbitraryPIDy = new PIDController(AutoConstants.kPathDriveP, 0, AutoConstants.kPathDriveD);
				swerve.ArbitraryPIDAngular = new PIDController(AutoConstants.kPathTurnP, 0, AutoConstants.kPathTurnD);
				swerve.ArbitraryPIDAngular.enableContinuousInput(-Math.PI, Math.PI);

				// Initialize the object detection
				objectDetectionCamRear = new ObjectDetectionIOPhotonVisionSim(VisionConstants.kLLName_Left + "_od",
						"Fuel", VisionConstants.kRobotToCamera_Left, swerveSim::getSimulatedDriveTrainPose,
						swerve::getFilteredPose, VisionConstants.kRobotToCamera_Left.getZ(),
						Math.toDegrees(VisionConstants.kRobotToCamera_Left.getRotation().getZ()),
						Math.toDegrees(VisionConstants.kRobotToCamera_Left.getRotation().getY()),
						FieldConstants.kFuelDiameterNominal / 2.0, FieldConstants.kFuelDiameterNominal / 2.0,
						Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY, VisionConstants.kObjectDetectionMaxYaw,
						VisionConstants.kSimCameraProperties,
						() -> SimulatedArena.getInstance()
								.getGamePiecesPosesByType("Fuel")
								.stream()
								.map(k -> new VisionTargetSim(k, new TargetModel(FieldConstants.kFuelDiameterNominal)))
								.sorted(Comparator.comparingDouble(k -> k.getPose()
										.toPose2d()
										.getTranslation()
										.getDistance(swerveSim.getSimulatedDriveTrainPose().getTranslation())))
								// TODO May filter out some important data, filter out by FOV instead
								.limit(50) // PV outputs at most 50
								.toList());
				objectDetection = new ObjectDetectionSubsystem("Fuel", objectDetectionCamRear);

				SimulatedArena.getInstance().resetFieldForAuto();
				break;

			case kReplay:
				swerveSim = null;
				// Dummy IOs for replay
				swerve = new SwerveSubsystem(new SwerveModuleIO() {}, new SwerveModuleIO() {}, new SwerveModuleIO() {},
						new SwerveModuleIO() {}, new Gyro(new GyroIO() {}), null, VecBuilder.fill(0.1, 0.1, 0.05),
						new LibVisionIO() {});
				objectDetection = new ObjectDetectionSubsystem("Fuel", new ObjectDetectionIO() {});
			default:
				break;
		}

		// Disable object detection initially
		objectDetectionCamRear.setEnabled(false);
		// Reset pipeline to use localization by default
		objectDetectionCamRear.setPipeline(VisionConstants.kLocalizationPipelineId);

		// Zero chassis
		swerve.resetOdometry(AllianceUtil.flipWithAlliance(Constants.kStartingPose));
		swerve.zeroHeadingFieldRelative();
	}

	/** Initializes the shooter related mechanisms. */
	private void initShooter() {
		switch (Robot.kRobotMode) {
			case kReal:
				feeder = new Feeder(new FeederIOSparkMax());
				hood = new Hood(new HoodIOKraken());
				rollers = new Rollers(new RollerIOKraken());
				turret = new Turret(new TurretIOKraken());
				break;

			case kSim:
				feeder = new Feeder(new FeederIOSim());
				hood = new Hood(new HoodIOSim());
				rollers = new Rollers(new RollerIOSim());
				turret = new Turret(new TurretIOSim());
				break;

			case kReplay:
				feeder = new Feeder(new FeederIO() {});
				hood = new Hood(new HoodIO() {});
				rollers = new Rollers(new RollerIO() {});
				turret = new Turret(new TurretIO() {});
				break;

			default:
				break;
		}
	}

	/** Initializes the intake-related mechanisms. */
	private void initIntake() {
		switch (Robot.kRobotMode) {
			case kReal:
				intake = new Intake(new IntakeIOKraken());
				// backpack = new Backpack(new BackpackIOKraken());
				break;

			case kSim:
				intake = new Intake(new IntakeIOSim());
				// backpack = new Backpack(new BackpackIOSim());
				break;

			case kReplay:
				intake = new Intake(new IntakeIO() {});
				// backpack = new Backpack(new BackpackIO() {});
				break;

			default:
				break;
		}
	}

	/** Initializes the climb mechanism and the superstructure. */
	private void initSuperstructure() {
		switch (Robot.kRobotMode) {
			case kReal:
				climb = new Climb(kHasClimb ? new ClimbIOKraken() : new ClimbIO() {});
				break;

			case kSim:
				climb = new Climb(new ClimbIOSim());
				break;

			case kReplay:
				climb = new Climb(new ClimbIO() {});
				break;

			default:
				break;
		}

		superstructure = new Superstructure(feeder, hood, rollers, turret, climb, intake, swerve);
	}

	/**
	 * Returns the autonomous routine manager.
	 *
	 * @return The autonomous routine manager.
	 */
	public AutonomousManager getAutoManager() { return m_autoManager; }

	/**
	 * Returns the current identification command. Returns an empty command if in competition mode.
	 *
	 * @return The current identification command.
	 */
	public Command getTestCommand() {
		if (Constants.kIsCompetition) return Commands.none();
		return m_testChooser.get().get();
	}

	/**
	 * Resets all encoders except swerve encoders. All subsystem positions must be set to the
	 * correct angles or positions with a <b>CALIBRATED TOOL or POINT</b> beforehand, do not guess
	 * positions.
	 */
	public void zeroSubsytemEncoders() {
		intake.zeroEncoders();
		hood.zeroEncoders();
		turret.zeroEncoders();
		rollers.zeroEncoders();
	}

	/**
	 * Updates the NT alerts.
	 */
	public void updateAlerts() {
		m_operatorControlWarning.set(!kOperatorController.isConnected());
		m_driverControlWarning.set(!kDriverController.isConnected());
		m_autoSequenceBuilder.updateNTAlerts();
	}

	/**
	 * Sets the controllers to rumble.
	 *
	 * @param rumbleStrength The strength in the range [0, 1].
	 */
	public void setControllerRumble(double rumbleStrength) {
		kDriverController.setRumble(RumbleType.kBothRumble, rumbleStrength);
		kOperatorController.setRumble(RumbleType.kBothRumble, rumbleStrength);
	}

	/**
	 * Returns the singleton instance of the RobotContainer. Returns null if called before a
	 * RobotContainer is instantiated.
	 *
	 * @return The singleton instance of the RobotContainer.
	 */
	public static RobotContainer getInstance() { return instance; }

	/** Resets and schedules the autonomous start scoring fuel command. */
	public void initAutonScoreFuelCommand() {
		cancelAutonScoreFuelCommand();
		m_autonScoreFuelCommand = Commands.runOnce(() -> superstructure.setObjectiveOriented(true))
				.andThen(superstructure.objectiveScoreHub());
		CommandScheduler.getInstance().schedule(m_autonScoreFuelCommand);
	}

	/** Cancels the autonomous start scoring fuel command. */
	public void cancelAutonScoreFuelCommand() {
		superstructure.setObjectiveOriented(false);
		if (m_autonScoreFuelCommand != null && m_autonScoreFuelCommand.isScheduled()) m_autonScoreFuelCommand.cancel();
	}
}
