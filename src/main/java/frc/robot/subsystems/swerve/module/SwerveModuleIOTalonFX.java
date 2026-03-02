package frc.robot.subsystems.swerve.module;

import static com.ctre.phoenix6.configs.TalonUtils.retryUntilOk;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Revolutions;
import static edu.wpi.first.units.Units.RevolutionsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.swerve.SwerveConstants.DriveConstants.kBackLeftChassisAngularOffset;
import static frc.robot.subsystems.swerve.SwerveConstants.DriveConstants.kBackRightChassisAngularOffset;
import static frc.robot.subsystems.swerve.SwerveConstants.DriveConstants.kFrontLeftCANCoderId;
import static frc.robot.subsystems.swerve.SwerveConstants.DriveConstants.kFrontLeftChassisAngularOffset;
import static frc.robot.subsystems.swerve.SwerveConstants.DriveConstants.kFrontLeftDrivingCanId;
import static frc.robot.subsystems.swerve.SwerveConstants.DriveConstants.kFrontLeftTurningCanId;
import static frc.robot.subsystems.swerve.SwerveConstants.DriveConstants.kFrontRightCANCoderId;
import static frc.robot.subsystems.swerve.SwerveConstants.DriveConstants.kFrontRightChassisAngularOffset;
import static frc.robot.subsystems.swerve.SwerveConstants.DriveConstants.kFrontRightDrivingCanId;
import static frc.robot.subsystems.swerve.SwerveConstants.DriveConstants.kFrontRightTurningCanId;
import static frc.robot.subsystems.swerve.SwerveConstants.DriveConstants.kRearLeftCANCoderId;
import static frc.robot.subsystems.swerve.SwerveConstants.DriveConstants.kRearLeftDrivingCanId;
import static frc.robot.subsystems.swerve.SwerveConstants.DriveConstants.kRearLeftTurningCanId;
import static frc.robot.subsystems.swerve.SwerveConstants.DriveConstants.kRearRightCANCoderId;
import static frc.robot.subsystems.swerve.SwerveConstants.DriveConstants.kRearRightDrivingCanId;
import static frc.robot.subsystems.swerve.SwerveConstants.DriveConstants.kRearRightTurningCanId;
import static frc.robot.subsystems.swerve.SwerveConstants.ModuleConfigs.kDriveKs;
import static frc.robot.subsystems.swerve.SwerveConstants.ModuleConfigs.kDriveKv;
import static frc.robot.subsystems.swerve.SwerveConstants.ModuleConfigs.kDrivingConfig;
import static frc.robot.subsystems.swerve.SwerveConstants.ModuleConfigs.kIsFOC;
import static frc.robot.subsystems.swerve.SwerveConstants.ModuleConstants.kDrivingMotorReduction;
import static frc.robot.subsystems.swerve.SwerveConstants.ModuleConstants.kTurningReduction;
import static frc.robot.subsystems.swerve.SwerveConstants.ModuleConstants.kWheelDiameterMeters;
import static frc.robot.subsystems.swerve.SwerveConstants.cloneTurnConfig;
import static frc.robot.subsystems.swerve.SwerveConstants.kDrivetrainBus;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.odometry.OdometryThread;
import frc.robot.subsystems.swerve.odometry.TalonFXOdometryThread;
import java.util.Queue;

/** The swerve module IO implementation for the TalonFX with CANCoders. */
public class SwerveModuleIOTalonFX implements SwerveModuleIO {
	private static final int kMaxConfigAttempts = 5;

	private Debouncer m_driveMotorConnectedDebouncer = new Debouncer(0.5, DebounceType.kFalling);
	private Debouncer m_turnMotorConnectedDebouncer = new Debouncer(0.5, DebounceType.kFalling);

	private final TalonFX m_driveTalon;
	private final TalonFX m_turnTalon;
	private final CANcoder m_turnCancoder;

	private VelocityVoltage m_driveVoltage = new VelocityVoltage(0).withEnableFOC(kIsFOC);
	private PositionVoltage m_turnVoltage = new PositionVoltage(0).withEnableFOC(kIsFOC);

	private Rotation2d m_offset;

	private StatusSignal<Angle> m_drivePosition;
	private StatusSignal<Angle> m_turnPosition;

	private StatusSignal<AngularVelocity> m_driveVelocity;
	private StatusSignal<AngularVelocity> m_turnVelocity;

	private StatusSignal<Voltage> m_driveAppliedVolts;
	private StatusSignal<Voltage> m_turnAppliedVolts;

	private StatusSignal<Current> m_driveSupplyCurrentAmps;
	private StatusSignal<Current> m_turnSupplyCurrentAmps;

	private StatusSignal<Current> m_driveStatorCurrentAmps;
	private StatusSignal<Current> m_turnStatorCurrentAmps;

	private StatusSignal<Temperature> m_driveTemperatureCelsius;
	private StatusSignal<Temperature> m_turnTemperatureCelsius;

	private Queue<Double> m_odometryTimestamps;
	private Queue<Double> m_odometryDrivePositionsRevs;
	private Queue<Double> m_odometryTurnPositionsRevs;

	// Units: m/rev, rad/rev
	private double m_revToMeters, m_revToRads;

	/**
	 * Constructs a new SwerveModuleIOTalonFX.
	 *
	 * @param idx The module ID. 0 = front left, 1 = front right, 2 = back left, 3 = back right.
	 */
	public SwerveModuleIOTalonFX(int idx) {
		if (!OdometryThread.threadType.get().equals("TALONFX"))
			throw new IllegalStateException("can't use TalonFX modules with a non-CTRE odometry thread");

		// Create && configure drive motor
		m_driveTalon = new TalonFX(switch (idx) {
			case 0 -> kFrontLeftDrivingCanId;
			case 1 -> kFrontRightDrivingCanId;
			case 2 -> kRearLeftDrivingCanId;
			case 3 -> kRearRightDrivingCanId;
			default -> 0;
		}, kDrivetrainBus);
		retryUntilOk(() -> m_driveTalon.getConfigurator().apply(kDrivingConfig), kMaxConfigAttempts,
				"Configuring Mk5n module #" + idx + " - drive motor");

		// Create && configure CANCoder
		int cancoderId = switch (idx) {
			case 0 -> kFrontLeftCANCoderId;
			case 1 -> kFrontRightCANCoderId;
			case 2 -> kRearLeftCANCoderId;
			case 3 -> kRearRightCANCoderId;
			default -> 0;
		};
		m_turnCancoder = new CANcoder(cancoderId, kDrivetrainBus);
		CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();
		retryUntilOk(() -> m_turnCancoder.getConfigurator().apply(cancoderConfig), kMaxConfigAttempts,
				"Configuring Mk5n module #" + idx + " - CANCoder");

		// Create && configure turn motor
		m_turnTalon = new TalonFX(switch (idx) {
			case 0 -> kFrontLeftTurningCanId;
			case 1 -> kFrontRightTurningCanId;
			case 2 -> kRearLeftTurningCanId;
			case 3 -> kRearRightTurningCanId;
			default -> 0;
		}, kDrivetrainBus);
		retryUntilOk(() -> m_turnTalon.getConfigurator().apply(cloneTurnConfig(kDrivingConfig, cancoderId)),
				kMaxConfigAttempts, "Configuring Mk5n module #" + idx + " - turn motor");

		// Precompute conversions
		m_revToMeters = Math.PI * kWheelDiameterMeters / (60 * kDrivingMotorReduction);
		m_revToRads = 2 * Math.PI / kTurningReduction;

		m_offset = new Rotation2d(switch (idx) {
			case 0 -> kFrontLeftChassisAngularOffset;
			case 1 -> kFrontRightChassisAngularOffset;
			case 2 -> kBackLeftChassisAngularOffset;
			case 3 -> kBackRightChassisAngularOffset;
			default -> 0;
		});

		// Create status signals
		m_drivePosition = m_driveTalon.getPosition();
		m_turnPosition = m_turnTalon.getPosition();

		m_driveVelocity = m_driveTalon.getVelocity();
		m_turnVelocity = m_turnTalon.getVelocity();

		m_driveAppliedVolts = m_driveTalon.getMotorVoltage();
		m_turnAppliedVolts = m_turnTalon.getMotorVoltage();

		m_driveSupplyCurrentAmps = m_driveTalon.getSupplyCurrent();
		m_turnSupplyCurrentAmps = m_turnTalon.getSupplyCurrent();

		m_driveStatorCurrentAmps = m_driveTalon.getStatorCurrent();
		m_turnStatorCurrentAmps = m_turnTalon.getStatorCurrent();

		m_driveTemperatureCelsius = m_driveTalon.getDeviceTemp();
		m_turnTemperatureCelsius = m_turnTalon.getDeviceTemp();

		// Register odometry signals
		TalonFXOdometryThread odometryThread = (TalonFXOdometryThread) OdometryThread.getInstance();
		m_odometryTimestamps = odometryThread.makeTimestampQueue();
		m_odometryDrivePositionsRevs = odometryThread.registerSignal(m_driveTalon, m_drivePosition);
		m_odometryTurnPositionsRevs = odometryThread.registerSignal(m_turnTalon, m_turnPosition);

		// Adjust update frequencies
		BaseStatusSignal.setUpdateFrequencyForAll(SwerveConstants.kOdometryFrequencyHz, m_drivePosition,
				m_driveVelocity);
		BaseStatusSignal.setUpdateFrequencyForAll(100, m_driveAppliedVolts, m_driveSupplyCurrentAmps,
				m_driveStatorCurrentAmps, m_driveTemperatureCelsius, m_turnPosition, m_turnVelocity, m_turnAppliedVolts,
				m_turnSupplyCurrentAmps, m_turnStatorCurrentAmps, m_turnTemperatureCelsius);
		retryUntilOk(() -> ParentDevice.optimizeBusUtilizationForAll(m_driveTalon, m_turnTalon), 5,
				"Optimizing CAN bus utilization for swerve module #" + idx);
	}

	@Override
	public void updateInputs(SwerveModuleIOInputs inputs) {
		// Update drive inputs
		inputs.driveMotorConnected = m_driveMotorConnectedDebouncer.calculate(
				BaseStatusSignal
						.refreshAll(m_drivePosition, m_driveVelocity, m_driveAppliedVolts, m_driveSupplyCurrentAmps,
								m_driveStatorCurrentAmps, m_driveTemperatureCelsius)
						.isOK());
		inputs.drivePositionMeters = m_drivePosition.getValue().in(Revolutions) * m_revToMeters;
		inputs.driveVelocityMetersPerSec = m_driveVelocity.getValue().in(RevolutionsPerSecond) * m_revToMeters;
		inputs.driveAppliedVolts = m_driveAppliedVolts.getValue().in(Volts);
		inputs.driveSupplyCurrentAmps = m_driveSupplyCurrentAmps.getValue().in(Amps);
		inputs.driveStatorCurrentAmps = m_driveStatorCurrentAmps.getValue().in(Amps);
		inputs.driveTemperatureCelsius = m_driveTemperatureCelsius.getValue().in(Celsius);

		// Update steering inputs
		inputs.turnMotorConnected = m_turnMotorConnectedDebouncer.calculate(
				BaseStatusSignal
						.refreshAll(m_turnPosition, m_turnVelocity, m_turnAppliedVolts, m_turnSupplyCurrentAmps,
								m_turnStatorCurrentAmps, m_turnTemperatureCelsius)
						.isOK());
		inputs.turnPosition = new Rotation2d(m_turnPosition.getValue().in(Revolutions) * m_revToRads).minus(m_offset);
		inputs.turnVelocityRadPerSec = m_turnVelocity.getValue().in(RevolutionsPerSecond) * m_revToRads;
		inputs.turnAppliedVolts = m_turnAppliedVolts.getValue().in(Volts);
		inputs.turnSupplyCurrentAmps = m_turnSupplyCurrentAmps.getValue().in(Amps);
		inputs.turnStatorCurrentAmps = m_turnStatorCurrentAmps.getValue().in(Amps);
		inputs.turnTemperatureCelsius = m_turnTemperatureCelsius.getValue().in(Celsius);

		// Update odometry inputs
		inputs.odometryTimestamps = m_odometryTimestamps.stream().mapToDouble((v) -> v).toArray();
		inputs.odometryDrivePositionsMeters = m_odometryDrivePositionsRevs.stream()
				.mapToDouble((v) -> v * m_revToMeters)
				.toArray();
		inputs.odometryTurnPositions = m_odometryTurnPositionsRevs.stream()
				.map((v) -> new Rotation2d(v * m_revToRads).minus(m_offset))
				.toArray(Rotation2d[]::new);

		m_odometryTimestamps.clear();
		m_odometryDrivePositionsRevs.clear();
		m_odometryTurnPositionsRevs.clear();
	}

	@Override
	public void runDriveVelocity(double velocityMetersPerSec) {
		runDriveVelocityFF(velocityMetersPerSec, 0);
	}

	@Override
	public void runDriveVelocityFF(double velocityMetersPerSec, double ff) {
		double ffVolts = ff + kDriveKs * Math.signum(velocityMetersPerSec) + kDriveKv * velocityMetersPerSec;
		m_driveTalon
				.setControl(m_driveVoltage.withVelocity(RevolutionsPerSecond.of(velocityMetersPerSec / m_revToMeters))
						.withFeedForward(ffVolts));
	}

	@Override
	public void runDriveVelocityFF(double velocityMetersPerSec, double ff, boolean onlyProvidedFF) {
		double ffVolts = ff
				+ (onlyProvidedFF ? 0 : kDriveKs * Math.signum(velocityMetersPerSec) + kDriveKv * velocityMetersPerSec);
		m_driveTalon
				.setControl(m_driveVoltage.withVelocity(RevolutionsPerSecond.of(velocityMetersPerSec / m_revToMeters))
						.withFeedForward(ffVolts));
	}

	@Override
	public void runDriveOpenLoop(double output) {
		m_driveTalon.setVoltage(output);
	}

	@Override
	public void runTurnPosition(Rotation2d rotation) {
		double wrapped = MathUtil.inputModulus(rotation.plus(m_offset).getRadians(), 0, 2 * Math.PI);
		m_turnTalon.setControl(m_turnVoltage.withPosition(Revolutions.of(wrapped / m_revToRads)));
	}

	@Override
	public void runTurnOpenLoop(double output) {
		m_turnTalon.setVoltage(output);
	}

	@Override
	public void resetEncoders() {
		m_driveTalon.setPosition(0);
	}
}
