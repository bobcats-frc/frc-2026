package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.swerve.SwerveConstants.ModuleConfigs.kPerModuleDriveGearbox;
import static frc.robot.subsystems.swerve.SwerveConstants.ModuleConfigs.kPerModuleTurnGearbox;
import static frc.robot.subsystems.swerve.SwerveConstants.ModuleConstants.kDrivingMotorReduction;
import static frc.robot.subsystems.swerve.SwerveConstants.ModuleConstants.kTurningReduction;
import static frc.robot.subsystems.swerve.SwerveConstants.ModuleConstants.kWheelCircumferenceMeters;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants.PhysicalParameters;
import java.util.Map;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;

// TODO Needs checking

/**
 * The constants required for the holonomic drive chassis.
 */
public final class SwerveConstants {
	// SysId routine time limits
	public static final double kDynamicTestTimeLimit = 3.0;
	public static final double kQuasistaticTestTimeLimit = 8.0;

	public static final CANBus kDrivetrainBus = new CANBus("");

	// Frequency of the high-frequency odometry thread
	// Keep in mind this follows the default 250Hz if in sim
	public static final double kOdometryFrequencyHz = kDrivetrainBus.isNetworkFD() ? 250.0 : 100.0;
	public static final int kOdometryUpdatePeriodMs = (int) (1000.0 / kOdometryFrequencyHz);

	// Whether the gyro angle should be inverted when zeroGyroFieldRelative is
	// called, shooter faces away from DS
	public static final boolean kInvertGyroZero = false;

	/** Class holding the drive constants. */
	public static final class DriveConstants {
		// Maximum allowed speeds (not nescessarily the hardware max)
		public static final double kMaxSpeedMetersPerSecond = 4.5; // m/s
		public static final double kMaxAngularSpeed = Math.toRadians(720); // rad/s

		// Note: if either is infinite, the other is also considered to be infinite.
		// public static final double kMaxLinearAcceleration = Double.POSITIVE_INFINITY; // m/s^2
		// public static final double kMaxAngularAcceleration = Double.POSITIVE_INFINITY; // rad/s^2

		// Hardware limit speed of the robot
		public static final double kHardwareMaxSpeed = 4.69;

		// Slow mode velocity multipliers
		public static final double kSlowModeTranslationalMultiplier = 0.4;
		public static final double kSlowModeRotationalMultiplier = 0.35;

		// Chassis configuration
		// Distance between the centers of the right and left wheels on robot
		public static final double kTrackWidth = 0.54165; // meters
		// Distance between the centers of the front and back wheels on robot
		public static final double kWheelBase = 0.57665; // meters
		// Instantiate the kinematics
		public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
				new Translation2d(kWheelBase / 2, kTrackWidth / 2), new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
				new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
				new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

		// Distance from each wheel to the center of the robot
		public static final double kDrivebaseRadius = Math.hypot(kTrackWidth / 2, kWheelBase / 2);

		// public static final double kBrownoutVoltage = 6.8;
		// static {
		// // Only effective on the roboRIO 2
		// RobotController.setBrownoutVoltage(kBrownoutVoltage);
		// }

		// Motor IDs
		public static final int kFrontLeftDrivingCanId = 11;
		public static final int kRearLeftDrivingCanId = 31;
		public static final int kFrontRightDrivingCanId = 21;
		public static final int kRearRightDrivingCanId = 41;

		public static final int kFrontLeftTurningCanId = 12;
		public static final int kRearLeftTurningCanId = 32;
		public static final int kFrontRightTurningCanId = 22;
		public static final int kRearRightTurningCanId = 42;

		public static final int kFrontLeftCANCoderId = 13;
		public static final int kRearLeftCANCoderId = 33;
		public static final int kFrontRightCANCoderId = 23;
		public static final int kRearRightCANCoderId = 42;

		public static final int kPigeon2CanId = 1;

		// Whether to enable cosine compensation for the real robot and simulation
		public static final boolean kCosineCompensateReal = true;
		public static final boolean kCosineCompensateSim = false;

		// Factor to correct for skews in the swerve modules while steering and driving
		// * Start with 0 to figure out the default skew direction when rotating in a
		// specific direction
		// Ideally should be in the range -0.15 to 0.15, start tuning with 0.1
		// * If skew gets worse, negate the value
		// * If the skew changes direction while rotating in the same direction, make
		// the value closer
		// to 0
		// * For precision, use the identification command
		public static final double kSkewCorrectionFactor = 0.04671;

		// Apply a velocity deadband to prevent jittering
		public static final double kJitterVelocityDeadbandMps = 2E-3;
	}

	/** Class holding the Mk5n modules' constants. */
	public static final class ModuleConstants {
		// Calculations required for conversion factors
		public static final double kDrivingMotorFreeSpeedRps = DriveMotorConstants.kFreeSpeedRpm / 60.0;
		// The wheel diameter of the standard Mk5n module is 4in
		// (TODO measure via command)
		public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
		public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
		// R1 reduction Mk5n gearing (~7.03125)
		public static final double kDrivingMotorReduction = 40500.0 / 5760.0;
		public static final double kDriveWheelFreeSpeed = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
				/ kDrivingMotorReduction;

		// Mk5n precise steering ratio
		public static final double kTurningReduction = 287.0 / 11.0;

		// // Use module constants to calculate conversion factors
		// // Units: m / rev
		// public static final double kDrivingFactor =
		// ModuleConstants.kWheelDiameterMeters * Math.PI /
		// ModuleConstants.kDrivingMotorReduction;
		// // Units: rad / rev
		// public static final double kTurningFactor = 2 * Math.PI;

		// Maximum motor temperatures, in Celsius
		public static final double kDriveMotorMaxTemperature = 80;
		public static final double kTurnMotorMaxTemperature = 80;

		// Whether to forcefully stop the robot when the motors overheat
		public static final boolean kEmergencyStopOnOverheat = true;
	}

	/** Class holding the constants for the autonomous period. */
	public static final class AutoConstants {
		// Autonomous path gains && constraints
		public static final double kPathDriveP = 4;
		public static final double kPathDriveD = 0;

		public static final double kPathTurnP = 15;
		public static final double kPathTurnD = 0.15;

		public static final double kPathDrivePSim = 4;
		public static final double kPathDriveDSim = 0;

		// 2pi rad -> 4pi rad/s
		public static final double kPathTurnPSim = 10;
		public static final double kPathTurnDSim = 0;

		public static final PathConstraints kPathConstraints = new PathConstraints(4.45, 5, Math.toRadians(540),
				Math.toRadians(720));

		// TODO Change for AndyMark field (if using precise path following, change in PP
		// instead)
		// Autonomous manager data
		public static final Map<String, Pose2d> kPathfindPoses = Map.of("Depot",
				new Pose2d(0.572, 5.980, Rotation2d.kPi), "Outpost", new Pose2d(0.497, 0.628, Rotation2d.kZero),
				"TowerThenDepot", new Pose2d(0.572, 5.980, new Rotation2d(-Math.PI / 2)), "AutoPark",
				new Pose2d(0.699, 1.847, Rotation2d.kZero));
		public static final Map<String, Command> kRoutineCommands = Map.of();

		public static final double kPathfindSkipTolerance = 0.045;

		// // Whether to apply the feedforwards provided by PathPlanner
		// public static final boolean kUseAutonomousFeedforward = false;

		// The alliance to invert the paths at
		public static final Alliance kInvertAutonomousPathAlliance = Alliance.Red;
	}

	/** Class holding the module configurations. */
	public static final class ModuleConfigs {
		public static final TalonFXConfiguration kDrivingConfig = new TalonFXConfiguration();
		public static final TalonFXConfiguration kTurningConfig = new TalonFXConfiguration();

		public static final Pigeon2Configuration kGyroConfig = new Pigeon2Configuration();

		// Whether the motor is a FOC motor (only on phoenix pro)
		public static final boolean kIsFOC = false;

		// The gearbox used on each Mk5n module (ignoring reductions)
		// Kraken X60 motor
		public static final DCMotor kPerModuleDriveGearbox = kIsFOC ? DCMotor.getKrakenX60Foc(1)
				: DCMotor.getKrakenX60(1);
		// Kraken X44 motor (reca.lc values)
		public static final DCMotor kPerModuleTurnGearbox = kIsFOC
				? new DCMotor(12, 5.01, 329, 3.2, Units.rotationsPerMinuteToRadiansPerSecond(7367), 1)
				: new DCMotor(12, 4.11, 279, 3.2, Units.rotationsPerMinuteToRadiansPerSecond(7757), 1);

		// Units:
		// Drive: V / (m/s)
		// Turn: V / (rad/s)
		public static final double kDriveP = 0.94/* 0.04 */, kDriveD = 0;
		public static final double kTurnP = 10, kTurnD = 0;

		public static final double kDrivePSim = 10, kDriveDSim = 0;
		public static final double kTurnPSim = 10, kTurnDSim = 0;

		public static final double kDriveKs = 0.02, kDriveKv = 1.220472;
		public static final double kDriveKsSim = 0.03457, kDriveKvSim = 2.55804;

		public static final boolean kInvertRightDriveMotor = true;
		public static final boolean kInvertLeftDriveMotor = false;

		public static final boolean kInvertTurnMotor = false;

		public static final double kDriveCurrentLimitAmps = 70;
		public static final double kTurnCurrentLimitAmps = 50;

		// Determine by Tuner calibration
		// "It’s extremely important for the modules to be aligned such that the bevel
		// gear faces the vertical center of the robot. Failure to perform this step may
		// lead the drive verification tests to fail."
		public static final double kFrontLeftEncoderOffset = -0.33935546875, kFrontRightEncoderOffset = -0.4892578125,
				kRearLeftEncoderOffset = 0.142333984375, kRearRightEncoderOffset = -0.03515625;
		public static final boolean kInvertEncoders = false;

		// TODO Change after Phoenix pro
		// CANCoder source for the turning motor
		public static final FeedbackSensorSourceValue kTurnFeedbackSource = FeedbackSensorSourceValue.RemoteCANcoder; // .FusedCANcoder;

		static {
			// Drive Config
			// kDrivingConfig.CurrentLimits.SupplyCurrentLimitEnable = false;
			kDrivingConfig.CurrentLimits.StatorCurrentLimit = kDriveCurrentLimitAmps;

			kDrivingConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
			kDrivingConfig.MotorOutput.DutyCycleNeutralDeadband = 0.03;
			// kDrivingConfig.MotorOutput.Inverted = kInvertDriveMotor ? InvertedValue.Clockwise_Positive
			// : InvertedValue.CounterClockwise_Positive;

			kDrivingConfig.Slot0 = new Slot0Configs().withKP(kDriveP * kWheelCircumferenceMeters)
					.withKD(kDriveD * kWheelCircumferenceMeters)
					.withKS(kDriveKs)
					.withKV(kDriveKv * kWheelCircumferenceMeters);

			kDrivingConfig.Feedback.SensorToMechanismRatio = kDrivingMotorReduction;

			// Turn Config
			// kTurningConfig.CurrentLimits.SupplyCurrentLimitEnable = false;
			kTurningConfig.CurrentLimits.StatorCurrentLimit = kTurnCurrentLimitAmps;

			kTurningConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
			kTurningConfig.MotorOutput.DutyCycleNeutralDeadband = 0.03;
			kTurningConfig.MotorOutput.Inverted = kInvertTurnMotor ? InvertedValue.Clockwise_Positive
					: InvertedValue.CounterClockwise_Positive;

			kTurningConfig.Slot0 = new Slot0Configs().withKP(kTurnP * 2 * Math.PI).withKD(kTurnD * 2 * Math.PI);
			kTurningConfig.ClosedLoopGeneral.ContinuousWrap = true;

			// ID is not set here as it's set per-module
			kTurningConfig.Feedback.FeedbackSensorSource = kTurnFeedbackSource;
			// kTurningConfig.Feedback.SensorToMechanismRatio = kTurningReduction;
			kTurningConfig.Feedback.RotorToSensorRatio = kTurningReduction;
		}
	}

	/** Class holding the motor constants. */
	public static final class DriveMotorConstants {
		// Motor free speed
		public static final double kFreeSpeedRpm = Units
				.radiansPerSecondToRotationsPerMinute(ModuleConfigs.kPerModuleDriveGearbox.freeSpeedRadPerSec);
	}

	/** Class holding the data required for simulation. */
	public static final class SwerveSimulationConstants {
		// Mk5n sim config
		public static final SwerveModuleSimulationConfig kSwerveModuleSimConfig = new SwerveModuleSimulationConfig(
				kPerModuleDriveGearbox, kPerModuleTurnGearbox, ModuleConstants.kDrivingMotorReduction,
				ModuleConstants.kTurningReduction, Volts.of(0.1), Volts.of(0.2), Inches.of(2),
				KilogramSquareMeters.of(0.03), PhysicalParameters.kWheelCOF);

		public static final DriveTrainSimulationConfig kSwerveSimConfig = DriveTrainSimulationConfig.Default()
				.withBumperSize(PhysicalParameters.kRobotLength, PhysicalParameters.kRobotWidth)
				.withGyro(COTS.ofPigeon2())
				.withRobotMass(PhysicalParameters.kRobotMass)
				.withTrackLengthTrackWidth(Meters.of(DriveConstants.kWheelBase), Meters.of(DriveConstants.kTrackWidth))
				.withSwerveModule(kSwerveModuleSimConfig);
	}

	/**
	 * Adapts the given steer TalonFXConfiguration to use the given sensor ID.
	 *
	 * @param base     The base TalonFXConfiguration to clone.
	 * @param sensorId The sensor ID to set in the new configuration.
	 * @return A new steer TalonFXConfiguration with the specified sensor ID.
	 */
	public static TalonFXConfiguration cloneTurnConfig(TalonFXConfiguration base, int sensorId) {
		TalonFXConfiguration newConfig = new TalonFXConfiguration();

		newConfig.CurrentLimits.SupplyCurrentLimitEnable = base.CurrentLimits.SupplyCurrentLimitEnable;
		newConfig.CurrentLimits.SupplyCurrentLimit = base.CurrentLimits.SupplyCurrentLimit;
		newConfig.CurrentLimits.StatorCurrentLimitEnable = base.CurrentLimits.StatorCurrentLimitEnable;
		newConfig.CurrentLimits.StatorCurrentLimit = base.CurrentLimits.StatorCurrentLimit;

		newConfig.MotorOutput.NeutralMode = base.MotorOutput.NeutralMode;
		newConfig.MotorOutput.DutyCycleNeutralDeadband = base.MotorOutput.DutyCycleNeutralDeadband;
		newConfig.MotorOutput.Inverted = base.MotorOutput.Inverted;
		newConfig.MotorOutput.PeakForwardDutyCycle = base.MotorOutput.PeakForwardDutyCycle;
		newConfig.MotorOutput.PeakReverseDutyCycle = base.MotorOutput.PeakReverseDutyCycle;

		newConfig.Slot0 = new Slot0Configs().withKP(base.Slot0.kP)
				.withKD(base.Slot0.kD)
				.withKS(base.Slot0.kS)
				.withKV(base.Slot0.kV);
		newConfig.ClosedLoopGeneral.ContinuousWrap = base.ClosedLoopGeneral.ContinuousWrap;

		newConfig.Feedback.FeedbackRemoteSensorID = sensorId;
		newConfig.Feedback.FeedbackSensorSource = base.Feedback.FeedbackSensorSource;
		newConfig.Feedback.RotorToSensorRatio = base.Feedback.RotorToSensorRatio;

		return newConfig;
	}

	/**
	 * Adapts the given steer TalonFXConfiguration to use the given drive inversion.
	 *
	 * @param base        The base TalonFXConfiguration to clone.
	 * @param invertDrive The sensor ID to set in the new configuration.
	 * @return A new steer TalonFXConfiguration with the specified sensor ID.
	 */
	public static TalonFXConfiguration cloneDriveConfig(TalonFXConfiguration base, boolean invertDrive) {
		TalonFXConfiguration newConfig = new TalonFXConfiguration();

		newConfig.CurrentLimits.SupplyCurrentLimitEnable = base.CurrentLimits.SupplyCurrentLimitEnable;
		newConfig.CurrentLimits.SupplyCurrentLimit = base.CurrentLimits.SupplyCurrentLimit;
		newConfig.CurrentLimits.StatorCurrentLimitEnable = base.CurrentLimits.StatorCurrentLimitEnable;
		newConfig.CurrentLimits.StatorCurrentLimit = base.CurrentLimits.StatorCurrentLimit;

		newConfig.MotorOutput.NeutralMode = base.MotorOutput.NeutralMode;
		newConfig.MotorOutput.DutyCycleNeutralDeadband = base.MotorOutput.DutyCycleNeutralDeadband;
		newConfig.MotorOutput.Inverted = invertDrive ? InvertedValue.Clockwise_Positive
				: InvertedValue.CounterClockwise_Positive;
		newConfig.MotorOutput.PeakForwardDutyCycle = base.MotorOutput.PeakForwardDutyCycle;
		newConfig.MotorOutput.PeakReverseDutyCycle = base.MotorOutput.PeakReverseDutyCycle;

		newConfig.Slot0 = new Slot0Configs().withKP(base.Slot0.kP)
				.withKD(base.Slot0.kD)
				.withKS(base.Slot0.kS)
				.withKV(base.Slot0.kV);
		newConfig.ClosedLoopGeneral.ContinuousWrap = base.ClosedLoopGeneral.ContinuousWrap;

		newConfig.Feedback.SensorToMechanismRatio = base.Feedback.SensorToMechanismRatio;

		return newConfig;
	}
}
