package frc.robot.subsystems.shooter.rollers.io;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Revolutions;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.shooter.rollers.RollerConstants.kD;
import static frc.robot.subsystems.shooter.rollers.RollerConstants.kGearboxReduction;
import static frc.robot.subsystems.shooter.rollers.RollerConstants.kIsFOC;
import static frc.robot.subsystems.shooter.rollers.RollerConstants.kMotorCANBus;
import static frc.robot.subsystems.shooter.rollers.RollerConstants.kMotorID;
import static frc.robot.subsystems.shooter.rollers.RollerConstants.kMotorInverted;
import static frc.robot.subsystems.shooter.rollers.RollerConstants.kMotorStatorLimitAmps;
import static frc.robot.subsystems.shooter.rollers.RollerConstants.kMotorSupplyLimitAmps;
import static frc.robot.subsystems.shooter.rollers.RollerConstants.kP;
import static frc.robot.subsystems.shooter.rollers.RollerConstants.kS;
import static frc.robot.subsystems.shooter.rollers.RollerConstants.kV;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonUtils;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

/**
 * The IO hardware implementation for roller hardware interacions with the TalonFX.
 */
public class RollerIOKraken implements RollerIO {
	private TalonFX m_talon;

	// Control objects
	private VelocityVoltage m_rpmOut = new VelocityVoltage(0).withEnableFOC(kIsFOC).withSlot(0);
	private VoltageOut m_voltsOut = new VoltageOut(0).withEnableFOC(kIsFOC);

	private StatusSignal<Voltage> m_voltsSignal;
	private StatusSignal<AngularVelocity> m_rpmSignal;
	private StatusSignal<Angle> m_positionSignal;
	private StatusSignal<Current> m_supplySignal, m_statorSignal;
	private StatusSignal<Temperature> m_tempSignal;

	/**
	 * Constructs a new RollerIOKraken.
	 */
	public RollerIOKraken() {
		m_talon = new TalonFX(kMotorID, new CANBus(kMotorCANBus));

		// Configure motors
		TalonFXConfiguration motorConfig = new TalonFXConfiguration();
		motorConfig.CurrentLimits.StatorCurrentLimit = kMotorStatorLimitAmps;
		motorConfig.CurrentLimits.SupplyCurrentLimit = kMotorSupplyLimitAmps;
		motorConfig.MotorOutput.DutyCycleNeutralDeadband = 0.03;
		motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
		motorConfig.MotorOutput.Inverted = kMotorInverted ? InvertedValue.Clockwise_Positive
				: InvertedValue.CounterClockwise_Positive;
		motorConfig.Feedback.SensorToMechanismRatio = kGearboxReduction;
		// Convert relevant PID units from RPM to RPS-frame while maintaining output
		motorConfig.Slot0.kP = kP * 60.0;
		motorConfig.Slot0.kD = kD * 60.0;
		motorConfig.Slot0.kS = kS;
		motorConfig.Slot0.kV = kV * 60.0;

		TalonUtils.retryUntilOk(() -> m_talon.getConfigurator().apply(motorConfig), 3,
				"Applying configuration to roller motor");

		// Status signals
		m_voltsSignal = m_talon.getMotorVoltage();
		m_rpmSignal = m_talon.getVelocity();
		m_positionSignal = m_talon.getPosition();
		m_supplySignal = m_talon.getSupplyCurrent();
		m_statorSignal = m_talon.getStatorCurrent();
		m_tempSignal = m_talon.getDeviceTemp();
		BaseStatusSignal.setUpdateFrequencyForAll(100, m_voltsSignal, m_rpmSignal, m_positionSignal, m_supplySignal,
				m_statorSignal, m_tempSignal);
		ParentDevice.optimizeBusUtilizationForAll(m_talon);
	}

	@Override
	public void updateInputs(RollerIOInputs inputs) {
		inputs.motorConnected = BaseStatusSignal
				.refreshAll(m_voltsSignal, m_rpmSignal, m_supplySignal, m_statorSignal, m_tempSignal)
				.isOK();
		inputs.appliedVoltage = m_voltsSignal.getValue().in(Volts);
		inputs.rpm = m_rpmSignal.getValue().in(RPM);
		inputs.positionRevs = m_positionSignal.getValue().in(Revolutions);
		inputs.supplyCurrentAmps = Math.abs(m_supplySignal.getValue().in(Amps));
		inputs.statorCurrentAmps = Math.abs(m_statorSignal.getValue().in(Amps));
		inputs.temperatureCelsius = m_tempSignal.getValue().in(Celsius);
	}

	@Override
	public void runVolts(double volts) {
		m_talon.setControl(m_voltsOut.withOutput(volts));
	}

	@Override
	public void runVelocity(double rpm) {
		m_talon.setControl(m_rpmOut.withVelocity(rpm / 60.0));
	}

	@Override
	public void zeroEncoders() {
		m_talon.setPosition(0);
	}

	@Override
	public void stop() {
		m_talon.stopMotor();
	}
}
