package frc.robot.subsystems.swerve.module;

import static com.ctre.phoenix6.configs.TalonUtils.retryUntilOk;
import static frc.robot.subsystems.swerve.SwerveConstants.DriveConstants.kFrontLeftCANCoderId;
import static frc.robot.subsystems.swerve.SwerveConstants.DriveConstants.kFrontLeftDrivingCanId;
import static frc.robot.subsystems.swerve.SwerveConstants.DriveConstants.kFrontLeftTurningCanId;
import static frc.robot.subsystems.swerve.SwerveConstants.DriveConstants.kFrontRightCANCoderId;
import static frc.robot.subsystems.swerve.SwerveConstants.DriveConstants.kFrontRightDrivingCanId;
import static frc.robot.subsystems.swerve.SwerveConstants.DriveConstants.kFrontRightTurningCanId;
import static frc.robot.subsystems.swerve.SwerveConstants.DriveConstants.kRearLeftCANCoderId;
import static frc.robot.subsystems.swerve.SwerveConstants.DriveConstants.kRearLeftDrivingCanId;
import static frc.robot.subsystems.swerve.SwerveConstants.DriveConstants.kRearLeftTurningCanId;
import static frc.robot.subsystems.swerve.SwerveConstants.DriveConstants.kRearRightCANCoderId;
import static frc.robot.subsystems.swerve.SwerveConstants.DriveConstants.kRearRightDrivingCanId;
import static frc.robot.subsystems.swerve.SwerveConstants.DriveConstants.kRearRightTurningCanId;
import static frc.robot.subsystems.swerve.SwerveConstants.ModuleConfigs.kDrivingConfig;
import static frc.robot.subsystems.swerve.SwerveConstants.ModuleConfigs.kFrontLeftEncoderOffset;
import static frc.robot.subsystems.swerve.SwerveConstants.ModuleConfigs.kFrontRightEncoderOffset;
import static frc.robot.subsystems.swerve.SwerveConstants.ModuleConfigs.kInvertEncoders;
import static frc.robot.subsystems.swerve.SwerveConstants.ModuleConfigs.kIsFOC;
import static frc.robot.subsystems.swerve.SwerveConstants.ModuleConfigs.kRearLeftEncoderOffset;
import static frc.robot.subsystems.swerve.SwerveConstants.ModuleConfigs.kRearRightEncoderOffset;
import static frc.robot.subsystems.swerve.SwerveConstants.ModuleConfigs.kTurningConfig;
import static frc.robot.subsystems.swerve.SwerveConstants.ModuleConstants.kWheelDiameterMeters;
import static frc.robot.subsystems.swerve.SwerveConstants.cloneTurnConfig;
import static frc.robot.subsystems.swerve.SwerveConstants.kDrivetrainBus;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.TalonFXOdometryThread;
import java.util.Queue;

/** The swerve module IO implementation for the TalonFX with CANCoders. */
public class SwerveModuleIOKraken implements SwerveModuleIO {
	private Debouncer m_driveMotorConnectedDebouncer = new Debouncer(0.5, DebounceType.kFalling);
	private Debouncer m_turnMotorConnectedDebouncer = new Debouncer(0.5, DebounceType.kFalling);
	private Debouncer m_cancoderConnectedDebouncer = new Debouncer(0.5, DebounceType.kFalling);

	private final TalonFX m_driveTalon;
	private final TalonFX m_turnTalon;
	private final CANcoder m_turnCancoder;

	private VoltageOut m_openLoopVoltage = new VoltageOut(0).withEnableFOC(kIsFOC);
	private VelocityVoltage m_driveVoltage = new VelocityVoltage(0).withEnableFOC(kIsFOC);
	private PositionVoltage m_turnVoltage = new PositionVoltage(0).withEnableFOC(kIsFOC);

	private StatusSignal<Angle> m_drivePosition;
	private StatusSignal<Angle> m_turnPosition;
	private StatusSignal<Angle> m_turnAbsolutePosition;

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

	// Units: m/rot
	private double m_rotToMeters;

	/**
	 * Constructs a new SwerveModuleIOTalonFX.
	 *
	 * @param idx The module ID. 0 = FL, 1 = FR, 2 = BL, 3 = BR.
	 */
	public SwerveModuleIOKraken(int idx) {
		// Create && configure drive motor
		m_driveTalon = new TalonFX(switch (idx) {
			case 0 -> kFrontLeftDrivingCanId;
			case 1 -> kFrontRightDrivingCanId;
			case 2 -> kRearLeftDrivingCanId;
			case 3 -> kRearRightDrivingCanId;
			default -> 0;
		}, kDrivetrainBus);
		retryUntilOk(() -> m_driveTalon.getConfigurator().apply(kDrivingConfig), 5,
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
		cancoderConfig.MagnetSensor.MagnetOffset = switch (idx) {
			case 0 -> kFrontLeftEncoderOffset;
			case 1 -> kFrontRightEncoderOffset;
			case 2 -> kRearLeftEncoderOffset;
			case 3 -> kRearRightEncoderOffset;
			default -> 0;
		};
		cancoderConfig.MagnetSensor.SensorDirection = kInvertEncoders ? SensorDirectionValue.Clockwise_Positive
				: SensorDirectionValue.CounterClockwise_Positive;
		retryUntilOk(() -> m_turnCancoder.getConfigurator().apply(cancoderConfig), 5,
				"Configuring Mk5n module #" + idx + " - CANCoder");

		// Create && configure turn motor
		m_turnTalon = new TalonFX(switch (idx) {
			case 0 -> kFrontLeftTurningCanId;
			case 1 -> kFrontRightTurningCanId;
			case 2 -> kRearLeftTurningCanId;
			case 3 -> kRearRightTurningCanId;
			default -> 0;
		}, kDrivetrainBus);
		retryUntilOk(() -> m_turnTalon.getConfigurator().apply(cloneTurnConfig(kTurningConfig, cancoderId)), 5,
				"Configuring Mk5n module #" + idx + " - turn motor");

		// Precompute conversions
		m_rotToMeters = Math.PI * kWheelDiameterMeters;

		// Create status signals
		m_drivePosition = m_driveTalon.getPosition();
		m_turnPosition = m_turnTalon.getPosition();

		m_turnAbsolutePosition = m_turnCancoder.getAbsolutePosition();

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
		TalonFXOdometryThread odometryThread = TalonFXOdometryThread.getInstance();
		m_odometryTimestamps = odometryThread.makeTimestampQueue();
		m_odometryDrivePositionsRevs = odometryThread.registerSignal(m_driveTalon, m_drivePosition);
		m_odometryTurnPositionsRevs = odometryThread.registerSignal(m_turnTalon, m_turnPosition);

		// Adjust update frequencies
		BaseStatusSignal.setUpdateFrequencyForAll(SwerveConstants.kOdometryFrequencyHz, m_drivePosition,
				m_driveVelocity);
		BaseStatusSignal.setUpdateFrequencyForAll(50, m_driveAppliedVolts, m_driveSupplyCurrentAmps,
				m_driveStatorCurrentAmps, m_driveTemperatureCelsius, m_turnPosition, m_turnAbsolutePosition,
				m_turnVelocity, m_turnAppliedVolts, m_turnSupplyCurrentAmps, m_turnStatorCurrentAmps,
				m_turnTemperatureCelsius);
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
		inputs.drivePositionMeters = m_drivePosition.getValueAsDouble() * m_rotToMeters;
		inputs.driveVelocityMetersPerSec = m_driveVelocity.getValueAsDouble() * m_rotToMeters;
		inputs.driveAppliedVolts = m_driveAppliedVolts.getValueAsDouble();
		inputs.driveSupplyCurrentAmps = m_driveSupplyCurrentAmps.getValueAsDouble();
		inputs.driveStatorCurrentAmps = m_driveStatorCurrentAmps.getValueAsDouble();
		inputs.driveTemperatureCelsius = m_driveTemperatureCelsius.getValueAsDouble();

		// Update steering inputs
		inputs.turnMotorConnected = m_turnMotorConnectedDebouncer.calculate(
				BaseStatusSignal
						.refreshAll(m_turnPosition, m_turnVelocity, m_turnAppliedVolts, m_turnSupplyCurrentAmps,
								m_turnStatorCurrentAmps, m_turnTemperatureCelsius)
						.isOK());
		inputs.turnPosition = Rotation2d.fromRotations(m_turnPosition.getValueAsDouble());
		inputs.turnVelocityRadPerSec = Units.rotationsToRadians(m_turnVelocity.getValueAsDouble());
		inputs.turnAppliedVolts = m_turnAppliedVolts.getValueAsDouble();
		inputs.turnSupplyCurrentAmps = m_turnSupplyCurrentAmps.getValueAsDouble();
		inputs.turnStatorCurrentAmps = m_turnStatorCurrentAmps.getValueAsDouble();
		inputs.turnTemperatureCelsius = m_turnTemperatureCelsius.getValueAsDouble();

		// Update odometry inputs
		inputs.odometryTimestamps = m_odometryTimestamps.stream().mapToDouble((v) -> v).toArray();
		inputs.odometryDrivePositionsMeters = m_odometryDrivePositionsRevs.stream()
				.mapToDouble(value -> value * m_rotToMeters)
				.toArray();
		inputs.odometryTurnPositions = m_odometryTurnPositionsRevs.stream()
				.map(value -> Rotation2d.fromRotations(value))
				.toArray(Rotation2d[]::new);

		// Update CANCoder inputs
		inputs.cancoderConnected = m_cancoderConnectedDebouncer
				.calculate(BaseStatusSignal.refreshAll(m_turnAbsolutePosition).isOK());
		inputs.turnAbsolutePosition = Rotation2d.fromRotations(m_turnAbsolutePosition.getValueAsDouble());

		// Reset queues
		m_odometryTimestamps.clear();
		m_odometryDrivePositionsRevs.clear();
		m_odometryTurnPositionsRevs.clear();
	}

	@Override
	public void runDriveVelocity(double velocityMetersPerSec) {
		m_driveTalon.setControl(m_driveVoltage.withVelocity(velocityMetersPerSec / m_rotToMeters));
	}

	@Override
	public void runDriveOpenLoop(double output) {
		m_driveTalon.setControl(m_openLoopVoltage.withOutput(output));
	}

	@Override
	public void runTurnPosition(Rotation2d rotation) {
		m_turnTalon.setControl(m_turnVoltage.withPosition(Units.radiansToRotations(rotation.getRadians())));
	}

	@Override
	public void runTurnOpenLoop(double output) {
		m_turnTalon.setControl(m_openLoopVoltage.withOutput(output));
	}

	@Override
	public void resetEncoders() {
		m_driveTalon.setPosition(0);
	}
}
