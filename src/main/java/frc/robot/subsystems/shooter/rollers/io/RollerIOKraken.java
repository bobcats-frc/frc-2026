package frc.robot.subsystems.shooter.rollers.io;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Revolutions;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.shooter.rollers.RollerConstants.kD;
import static frc.robot.subsystems.shooter.rollers.RollerConstants.kFollowerMotorID;
import static frc.robot.subsystems.shooter.rollers.RollerConstants.kFollowerOpposesMain;
import static frc.robot.subsystems.shooter.rollers.RollerConstants.kGearboxReduction;
import static frc.robot.subsystems.shooter.rollers.RollerConstants.kIsFOC;
import static frc.robot.subsystems.shooter.rollers.RollerConstants.kMainMotorID;
import static frc.robot.subsystems.shooter.rollers.RollerConstants.kMotorCANBus;
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
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
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
	private TalonFX m_mainTalon;
	private TalonFX m_followerTalon;

	// Control objects
	private VelocityVoltage m_rpmOut = new VelocityVoltage(0).withEnableFOC(kIsFOC).withSlot(0);
	private VoltageOut m_voltsOut = new VoltageOut(0).withEnableFOC(kIsFOC);

	private StatusSignal<Voltage> m_voltsSignal;
	private StatusSignal<AngularVelocity> m_rpmSignal;
	private StatusSignal<Angle> m_positionSignal;
	private StatusSignal<Current> m_supplySignal, m_statorSignal;
	private StatusSignal<Temperature> m_tempSignal;

	private StatusSignal<Voltage> m_voltsSignalFollower;
	private StatusSignal<AngularVelocity> m_rpmSignalFollower;
	private StatusSignal<Angle> m_positionSignalFollower;
	private StatusSignal<Current> m_supplySignalFollower, m_statorSignalFollower;
	private StatusSignal<Temperature> m_tempSignalFollower;

	/**
	 * Constructs a new RollerIOKraken.
	 */
	public RollerIOKraken() {
		m_mainTalon = new TalonFX(kMainMotorID, new CANBus(kMotorCANBus));
		m_followerTalon = new TalonFX(kFollowerMotorID, new CANBus(kMotorCANBus));

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

		TalonUtils.retryUntilOk(() -> m_mainTalon.getConfigurator().apply(motorConfig), 3,
				"Applying configuration to main roller motor");

		TalonUtils.retryUntilOk(() -> m_followerTalon.getConfigurator().apply(motorConfig), 3,
				"Applying configuration to follower roller motor");

		m_followerTalon.setControl(new Follower(kMainMotorID,
				kFollowerOpposesMain ? MotorAlignmentValue.Opposed : MotorAlignmentValue.Aligned));

		// Status signals
		m_voltsSignal = m_mainTalon.getMotorVoltage();
		m_rpmSignal = m_mainTalon.getVelocity();
		m_positionSignal = m_mainTalon.getPosition();
		m_supplySignal = m_mainTalon.getSupplyCurrent();
		m_statorSignal = m_mainTalon.getStatorCurrent();
		m_tempSignal = m_mainTalon.getDeviceTemp();

		m_voltsSignalFollower = m_followerTalon.getMotorVoltage();
		m_rpmSignalFollower = m_followerTalon.getVelocity();
		m_positionSignalFollower = m_followerTalon.getPosition();
		m_supplySignalFollower = m_followerTalon.getSupplyCurrent();
		m_statorSignalFollower = m_followerTalon.getStatorCurrent();
		m_tempSignalFollower = m_followerTalon.getDeviceTemp();

		BaseStatusSignal.setUpdateFrequencyForAll(100, m_voltsSignal, m_rpmSignal, m_positionSignal, m_supplySignal,
				m_statorSignal, m_tempSignal, m_voltsSignalFollower, m_rpmSignalFollower, m_positionSignalFollower,
				m_supplySignalFollower, m_statorSignalFollower, m_tempSignalFollower);

		ParentDevice.optimizeBusUtilizationForAll(m_mainTalon, m_followerTalon);
	}

	@Override
	public void updateInputs(RollerIOInputs inputs) {
		inputs.mainMotorConnected = BaseStatusSignal
				.refreshAll(m_voltsSignal, m_rpmSignal, m_supplySignal, m_statorSignal, m_tempSignal)
				.isOK();
		inputs.appliedVoltageMain = m_voltsSignal.getValue().in(Volts);
		inputs.rpmMain = m_rpmSignal.getValue().in(RPM);
		inputs.positionRevsMain = m_positionSignal.getValue().in(Revolutions);
		inputs.supplyCurrentAmpsMain = Math.abs(m_supplySignal.getValue().in(Amps));
		inputs.statorCurrentAmpsMain = Math.abs(m_statorSignal.getValue().in(Amps));
		inputs.temperatureCelsiusMain = m_tempSignal.getValue().in(Celsius);

		inputs.followerMotorConnected = BaseStatusSignal
				.refreshAll(m_voltsSignalFollower, m_rpmSignalFollower, m_supplySignalFollower, m_statorSignalFollower,
						m_tempSignalFollower)
				.isOK();
		inputs.appliedVoltageFollower = m_voltsSignalFollower.getValue().in(Volts);
		inputs.rpmFollower = m_rpmSignalFollower.getValue().in(RPM);
		inputs.positionRevsFollower = m_positionSignalFollower.getValue().in(Revolutions);
		inputs.supplyCurrentAmpsFollower = Math.abs(m_supplySignalFollower.getValue().in(Amps));
		inputs.statorCurrentAmpsFollower = Math.abs(m_statorSignalFollower.getValue().in(Amps));
		inputs.temperatureCelsiusFollower = m_tempSignalFollower.getValue().in(Celsius);
	}

	@Override
	public void runVolts(double volts) {
		m_mainTalon.setControl(m_voltsOut.withOutput(volts));
	}

	@Override
	public void runVelocity(double rpm) {
		m_mainTalon.setControl(m_rpmOut.withVelocity(rpm / 60.0));
	}

	@Override
	public void zeroEncoders() {
		m_mainTalon.setPosition(0);
	}

	@Override
	public void stop() {
		m_mainTalon.stopMotor();
	}
}
