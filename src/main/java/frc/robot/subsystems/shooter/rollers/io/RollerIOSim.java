package frc.robot.subsystems.shooter.rollers.io;

import static frc.robot.subsystems.shooter.rollers.RollerConstants.kDSim;
import static frc.robot.subsystems.shooter.rollers.RollerConstants.kGearbox;
import static frc.robot.subsystems.shooter.rollers.RollerConstants.kGearboxReduction;
import static frc.robot.subsystems.shooter.rollers.RollerConstants.kMomentOfInertia;
import static frc.robot.subsystems.shooter.rollers.RollerConstants.kMotorStatorLimitAmps;
import static frc.robot.subsystems.shooter.rollers.RollerConstants.kMotorSupplyLimitAmps;
import static frc.robot.subsystems.shooter.rollers.RollerConstants.kPSim;
import static frc.robot.subsystems.shooter.rollers.RollerConstants.kSSim;
import static frc.robot.subsystems.shooter.rollers.RollerConstants.kVSim;

import com.bobcats.lib.sim.CustomDCMotorSim;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.LinearSystemId;
import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.junction.Logger;

/** The IO sim implementation for roller hardware interacions without any hardware. */
public class RollerIOSim implements RollerIO {
	private CustomDCMotorSim m_sim;

	// Control objects
	private PIDController m_pid = new PIDController(kPSim, 0.0, kDSim);
	private SimpleMotorFeedforward m_ff = new SimpleMotorFeedforward(kSSim, kVSim);
	private double m_closedLoopSetpoint;
	private boolean m_isSetpoint = false;

	/**
	 * Constructs a new RollerIOSim.
	 */
	public RollerIOSim() {
		// Create sim
		m_sim = new CustomDCMotorSim(LinearSystemId.createDCMotorSystem(kGearbox, kMomentOfInertia, kGearboxReduction),
				kGearbox, 2);
		m_sim.setSupplyCurrentLimitAmps(kMotorSupplyLimitAmps);
		m_sim.setStatorCurrentLimitAmps(kMotorStatorLimitAmps);

		// Register sim
		SimulatedArena.getInstance().addCustomSimulation(m_sim);
	}

	@Override
	public void updateInputs(RollerIOInputs inputs) {
		// Update controls
		if (m_isSetpoint) {
			// Closed-loop control
			double pidOutput = m_pid.calculate(m_sim.getOutputAngularVelocityRPM());
			double ffOutput = m_ff.calculate(m_closedLoopSetpoint);
			m_sim.setInputVoltage(pidOutput + ffOutput);
		}

		Logger.recordOutput("Rollers/SimPIDSetpoint", m_pid.getSetpoint());
		Logger.recordOutput("Rollers/SimIsSetpoint", m_isSetpoint);

		// Update inputs
		inputs.mainMotorConnected = true;
		inputs.appliedVoltageMain = m_sim.getAppliedInputVoltage();
		inputs.rpmMain = m_sim.getOutputAngularVelocityRPM();
		inputs.positionRevsMain = m_sim.getOutputAngularPositionRotations();
		inputs.supplyCurrentAmpsMain = Math.abs(m_sim.getSupplyCurrentDrawAmps());
		inputs.statorCurrentAmpsMain = Math.abs(m_sim.getStatorCurrentDrawAmps());
		inputs.temperatureCelsiusMain = 20;

		inputs.followerMotorConnected = true;
		inputs.appliedVoltageFollower = m_sim.getAppliedInputVoltage();
		inputs.rpmFollower = m_sim.getOutputAngularVelocityRPM();
		inputs.positionRevsFollower = m_sim.getOutputAngularPositionRotations();
		inputs.supplyCurrentAmpsFollower = Math.abs(m_sim.getSupplyCurrentDrawAmps());
		inputs.statorCurrentAmpsFollower = Math.abs(m_sim.getStatorCurrentDrawAmps());
		inputs.temperatureCelsiusFollower = 20;
	}

	@Override
	public void runVolts(double volts) {
		m_sim.setInputVoltage(volts);
		m_closedLoopSetpoint = 0;
		m_isSetpoint = false;
		m_pid.reset();
	}

	@Override
	public void zeroEncoders() {
		m_sim.setState(0, 0);
		stop();
	}

	@Override
	public void runVelocity(double rpm) {
		m_pid.setSetpoint(rpm);
		m_closedLoopSetpoint = rpm;
		m_isSetpoint = true;
	}
}
