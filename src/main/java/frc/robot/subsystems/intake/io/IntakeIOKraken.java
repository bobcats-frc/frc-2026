package frc.robot.subsystems.intake.io;

import static frc.robot.subsystems.intake.IntakeConstants.kArmCalibrationAngleDeg;
import static frc.robot.subsystems.intake.IntakeConstants.kArmGearboxReduction;
import static frc.robot.subsystems.intake.IntakeConstants.kArmMotorCANBus;
import static frc.robot.subsystems.intake.IntakeConstants.kArmMotorID;
import static frc.robot.subsystems.intake.IntakeConstants.kArmMotorInverted;
import static frc.robot.subsystems.intake.IntakeConstants.kArmMotorStatorLimitAmps;
import static frc.robot.subsystems.intake.IntakeConstants.kArmMotorSupplyLimitAmps;
import static frc.robot.subsystems.intake.IntakeConstants.kD;
import static frc.robot.subsystems.intake.IntakeConstants.kG;
import static frc.robot.subsystems.intake.IntakeConstants.kIsFOC;
import static frc.robot.subsystems.intake.IntakeConstants.kMotionMagicAccelerationDegPerSecSq;
import static frc.robot.subsystems.intake.IntakeConstants.kMotionMagicMaxVelocityDegPerSec;
import static frc.robot.subsystems.intake.IntakeConstants.kP;
import static frc.robot.subsystems.intake.IntakeConstants.kRollerGearboxReduction;
import static frc.robot.subsystems.intake.IntakeConstants.kRollerMotorCANBus;
import static frc.robot.subsystems.intake.IntakeConstants.kRollerMotorID;
import static frc.robot.subsystems.intake.IntakeConstants.kRollerMotorInverted;
import static frc.robot.subsystems.intake.IntakeConstants.kRollerMotorStatorLimitAmps;
import static frc.robot.subsystems.intake.IntakeConstants.kRollerMotorSupplyLimitAmps;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonUtils;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

/** The IO hardware implementation for intake hardware interacions with the TalonFX. */
public class IntakeIOKraken implements IntakeIO {
	private TalonFX m_rollerTalon, m_armTalon;

	private Debouncer m_rollerMotorConnectedDebouncer = new Debouncer(0.5, DebounceType.kFalling);
	private Debouncer m_armMotorConnectedDebouncer = new Debouncer(0.5, DebounceType.kFalling);

	private VoltageOut m_rollerVoltage = new VoltageOut(0.0).withEnableFOC(kIsFOC),
			m_armVoltage = new VoltageOut(0).withEnableFOC(kIsFOC);
	private MotionMagicVoltage m_armPosition = new MotionMagicVoltage(0.0).withEnableFOC(kIsFOC).withSlot(0);

	private StatusSignal<Voltage> m_rollerVoltsSignal;
	private StatusSignal<AngularVelocity> m_rollerRpmSignal;
	private StatusSignal<Current> m_rollerSupplySignal, m_rollerStatorSignal;
	private StatusSignal<Temperature> m_rollerTempSignal;

	private StatusSignal<Voltage> m_armVoltsSignal;
	private StatusSignal<AngularVelocity> m_armRpmSignal;
	private StatusSignal<Angle> m_armPositionSignal;
	private StatusSignal<Current> m_armSupplySignal, m_armStatorSignal;
	private StatusSignal<Temperature> m_armTempSignal;

	/**
	 * Constructs a new IntakeIOKraken.
	 */
	public IntakeIOKraken() {
		m_rollerTalon = new TalonFX(kRollerMotorID, new CANBus(kRollerMotorCANBus));
		m_armTalon = new TalonFX(kArmMotorID, new CANBus(kArmMotorCANBus));

		// Configure arm motor
		TalonFXConfiguration armMotorConfig = new TalonFXConfiguration();
		armMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
		armMotorConfig.CurrentLimits.StatorCurrentLimit = kArmMotorStatorLimitAmps;
		armMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		armMotorConfig.CurrentLimits.SupplyCurrentLimit = kArmMotorSupplyLimitAmps;
		armMotorConfig.MotorOutput.DutyCycleNeutralDeadband = 0.03;
		armMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
		armMotorConfig.MotorOutput.Inverted = kArmMotorInverted ? InvertedValue.Clockwise_Positive
				: InvertedValue.CounterClockwise_Positive;
		armMotorConfig.Feedback.SensorToMechanismRatio = kArmGearboxReduction;
		armMotorConfig.Slot0.kP = kP * 360.0;
		armMotorConfig.Slot0.kD = kD * 360.0;
		armMotorConfig.Slot0.kG = kG;
		armMotorConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
		armMotorConfig.MotionMagic.MotionMagicCruiseVelocity = kMotionMagicMaxVelocityDegPerSec / 360.0;
		armMotorConfig.MotionMagic.MotionMagicAcceleration = kMotionMagicAccelerationDegPerSecSq / 360.0;
		armMotorConfig.ClosedLoopGeneral.ContinuousWrap = true;

		TalonUtils.retryUntilOk(() -> m_armTalon.getConfigurator().apply(armMotorConfig), 3,
				"Applying configuration to intake arm motor");

		// Configure roller motor
		TalonFXConfiguration motorConfig = new TalonFXConfiguration();
		motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
		motorConfig.CurrentLimits.StatorCurrentLimit = kRollerMotorStatorLimitAmps;
		motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		motorConfig.CurrentLimits.SupplyCurrentLimit = kRollerMotorSupplyLimitAmps;
		motorConfig.MotorOutput.DutyCycleNeutralDeadband = 0.03;
		motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
		motorConfig.MotorOutput.Inverted = kRollerMotorInverted ? InvertedValue.Clockwise_Positive
				: InvertedValue.CounterClockwise_Positive;
		motorConfig.Feedback.SensorToMechanismRatio = kRollerGearboxReduction;

		TalonUtils.retryUntilOk(() -> m_rollerTalon.getConfigurator().apply(motorConfig), 3,
				"Applying configuration to intake roller motor");

		// Status signals
		m_rollerVoltsSignal = m_rollerTalon.getMotorVoltage();
		m_rollerRpmSignal = m_rollerTalon.getVelocity();
		m_rollerSupplySignal = m_rollerTalon.getSupplyCurrent();
		m_rollerStatorSignal = m_rollerTalon.getStatorCurrent();
		m_rollerTempSignal = m_rollerTalon.getDeviceTemp();

		m_armVoltsSignal = m_armTalon.getMotorVoltage();
		m_armRpmSignal = m_armTalon.getVelocity();
		m_armPositionSignal = m_armTalon.getPosition();
		m_armSupplySignal = m_armTalon.getSupplyCurrent();
		m_armStatorSignal = m_armTalon.getStatorCurrent();
		m_armTempSignal = m_armTalon.getDeviceTemp();

		BaseStatusSignal.setUpdateFrequencyForAll(100, m_rollerVoltsSignal, m_rollerRpmSignal, m_rollerSupplySignal,
				m_rollerStatorSignal, m_rollerTempSignal, m_armVoltsSignal, m_armRpmSignal, m_armPositionSignal,
				m_armSupplySignal, m_armStatorSignal, m_armTempSignal);
		ParentDevice.optimizeBusUtilizationForAll(m_rollerTalon, m_armTalon);
	}

	@Override
	public void updateInputs(IntakeIOInputs inputs) {
		inputs.rollerMotorConnected = m_rollerMotorConnectedDebouncer.calculate(
				BaseStatusSignal
						.refreshAll(m_rollerVoltsSignal, m_rollerRpmSignal, m_rollerSupplySignal, m_rollerStatorSignal,
								m_rollerTempSignal)
						.isOK());
		inputs.rollerAppliedVoltage = m_rollerVoltsSignal.getValueAsDouble();
		inputs.rollerRpm = m_rollerRpmSignal.getValueAsDouble() * 60.0;
		inputs.rollerSupplyCurrentAmps = Math.abs(m_rollerSupplySignal.getValueAsDouble());
		inputs.rollerStatorCurrentAmps = Math.abs(m_rollerStatorSignal.getValueAsDouble());
		inputs.rollerTemperatureCelsius = m_rollerTempSignal.getValueAsDouble();

		inputs.armMotorConnected = m_armMotorConnectedDebouncer.calculate(
				BaseStatusSignal
						.refreshAll(m_armVoltsSignal, m_armRpmSignal, m_armPositionSignal, m_armSupplySignal,
								m_armStatorSignal, m_armTempSignal)
						.isOK());
		inputs.armAppliedVoltage = m_armVoltsSignal.getValueAsDouble();
		inputs.armRpm = m_armRpmSignal.getValueAsDouble() * 60.0;
		inputs.armPositionDegrees = MathUtil
				.inputModulus(Units.rotationsToDegrees(m_armPositionSignal.getValueAsDouble()), -180, 180);
		inputs.armSupplyCurrentAmps = Math.abs(m_armSupplySignal.getValueAsDouble());
		inputs.armStatorCurrentAmps = Math.abs(m_armStatorSignal.getValueAsDouble());
		inputs.armTemperatureCelsius = m_armTempSignal.getValueAsDouble();
	}

	@Override
	public void runRollerVolts(double volts) {
		m_rollerTalon.setControl(m_rollerVoltage.withOutput(volts));
	}

	@Override
	public void runArmVolts(double volts) {
		m_armTalon.setControl(m_armVoltage.withOutput(volts));
	}

	@Override
	public void stopRollers() {
		m_rollerTalon.stopMotor();
	}

	@Override
	public void runArmPosition(double degrees) {
		m_armTalon.setControl(
				m_armPosition.withPosition(Units.degreesToRotations(MathUtil.inputModulus(degrees, -180, 180))));
	}

	@Override
	public void zeroArmEncoders() {
		m_armTalon.setPosition(Units.degreesToRotations(kArmCalibrationAngleDeg));
	}

	@Override
	public void stopArm() {
		m_armTalon.stopMotor();
	}
}
