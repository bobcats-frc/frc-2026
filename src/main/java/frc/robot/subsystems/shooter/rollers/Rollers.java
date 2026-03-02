package frc.robot.subsystems.shooter.rollers;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.shooter.rollers.RollerConstants.kMaxAllowedRPM;
import static frc.robot.subsystems.shooter.rollers.RollerConstants.kMaxTemperature;
import static frc.robot.subsystems.shooter.rollers.RollerConstants.kMotorID;
import static frc.robot.subsystems.shooter.rollers.RollerConstants.kSysIdTimeout;
import static frc.robot.subsystems.shooter.rollers.RollerConstants.kSysIdVoltageRampRate;
import static frc.robot.subsystems.shooter.rollers.RollerConstants.kSysIdVoltageStep;

import com.bobcats.lib.utils.Tracer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.shooter.rollers.io.RollerIO;
import frc.robot.subsystems.shooter.rollers.io.RollerIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

/**
 * The main shooter subsystem to shoot fuel.
 */
public class Rollers extends SubsystemBase {
	private final RollerIO m_io;
	private RollerIOInputsAutoLogged m_inputs = new RollerIOInputsAutoLogged();

	private double m_setpoint = 0.0;

	private Alert m_motorDisconnectedAlert = new Alert("Roller motor has disconnected! (ID " + kMotorID + ")",
			AlertType.kError);
	private Alert m_motorOverheatAlert = new Alert("Roller motor is overheating! (ID: " + kMotorID + ")",
			AlertType.kWarning);

	private SysIdRoutine m_routine;

	private boolean m_isZeroed;

	private static final double kVelocityDefaultToleranceRPM = 50.0;

	/**
	 * Constructs a new Rollers.
	 *
	 * @param io The IO implementation to use.
	 */
	public Rollers(RollerIO io) {
		m_io = io;

		// SysID routine
		m_routine = new SysIdRoutine(
				new SysIdRoutine.Config(Volts.of(kSysIdVoltageRampRate).per(Seconds.of(1).unit()),
						Volts.of(kSysIdVoltageStep), Seconds.of(kSysIdTimeout),
						(state) -> { Logger.recordOutput("Rollers/SysIdState", state.toString()); }),
				new SysIdRoutine.Mechanism((voltage) -> m_io.runVolts(voltage.in(Volts)), null, this));
		setName("Rollers");
	}

	@Override
	public void periodic() {
		Tracer.start("RollersPeriodic");
		m_io.updateInputs(m_inputs);
		Logger.processInputs("Rollers", m_inputs);

		if (DriverStation.isDisabled()) { m_io.stop(); m_setpoint = 0; }

		m_motorOverheatAlert.set(m_inputs.temperatureCelsius > kMaxTemperature);

		Logger.recordOutput("Rollers/EncodersZeroed", areEncodersZeroed());
		Logger.recordOutput("Rollers/NearSetpoint", isNearSetpoint());

		m_motorDisconnectedAlert.set(!m_inputs.motorConnected);
		Tracer.finish("RollersPeriodic");
	}

	/**
	 * Stops the rollers.
	 */
	public void stop() {
		m_io.stop();
	}

	/**
	 * Sets the roller's angular velocity to the given value.
	 *
	 * @param rpm The target velocity, in RPM.
	 */
	public void setRollerVelocity(double rpm) {
		if (Math.abs(rpm) > kMaxAllowedRPM) {
			DriverStation.reportWarning("WARNING: Rollers::setRollerVelocity, velocity exceeds limit: " + rpm, true);
			stop();
			return;
		}
		m_io.runVelocity(rpm);
		m_setpoint = rpm;
	}

	public void setVoltage(double volts) {
		m_io.runVolts(volts);
	}

	/**
	 * Returns whether the roller is near its setpoint.
	 *
	 * @return Whether the roller is near its setpoint.
	 */
	public boolean isNearSetpoint() {
		return MathUtil.isNear(m_setpoint, m_inputs.rpm, kVelocityDefaultToleranceRPM);
	}

	/**
	 * Returns the latest setpoint.
	 *
	 * @return The latest setpoint, in RPM.
	 */
	public double getLatestSetpoint() { return m_setpoint; }

	/**
	 * Returns the current angular velocity.
	 *
	 * @return The current angular velocity, in RPM.
	 */
	public double getVelocity() { return m_inputs.rpm; }

	/**
	 * Resets the encoders of the rollers. Encoders are set to read 0.
	 */
	public void zeroEncoders() {
		m_io.zeroEncoders();
		m_isZeroed = true;
		stop();
	}

	/**
	 * Returns whether the encoders have been zeroed.
	 *
	 * @return Whether the encoders have been zeroed.
	 */
	public boolean areEncodersZeroed() {
		return m_isZeroed;
	}

	/**
	 * Runs a quasistatic (negligble acceleration) system identification routine.
	 *
	 * @param direction The direction.
	 * @return The command to run the identification test.
	 */
	public Command sysIdQuasistatic(Direction direction) {
		return m_routine.quasistatic(direction);
	}

	/**
	 * Runs a dynamic (non-negligble acceleration) system identification routine.
	 *
	 * @param direction The direction.
	 * @return The command to run the identification test.
	 */
	public Command sysIdDynamic(Direction direction) {
		return m_routine.dynamic(direction);
	}
}
