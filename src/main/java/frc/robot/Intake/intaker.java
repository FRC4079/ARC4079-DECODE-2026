package frc.robot.Intake;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;




public class intaker extends StateMachine<intaker.State> {
	public enum State { OFF, INTAKE, FEED, REVERSE }

	private final PhoenixPIDController cdgPID;
	private static final double INTAKE_POWER = 4000;
	private static final double FEED_POWER = 6.5 * 0.75;
	private static final double REVERSE_POWER = -4 * 0.75;

	private final TalonFX motor;
	private final VelocityTorqueCurrentFOC mmRequest = new VelocityTorqueCurrentFOC(0).withSlot(0);

	public intaker(TalonFX motor) {
		super(SubsystemPriority.DEPLOY, State.OFF);
		this.motor = motor;

		var cfg = new TalonFXConfiguration();

		cfg.CurrentLimits = new CurrentLimitsConfigs()
				.withSupplyCurrentLimit(20)
				.withSupplyCurrentLimitEnable(true)
				.withStatorCurrentLimit(40)
				.withStatorCurrentLimitEnable(true);


		cdgPID = new PhoenixPIDController(3.0, 0.0, 0.0);
		
		// apply PID gains to slot 0 so the TalonFX uses them for closed-loop control
		cfg.Slot0 = new Slot0Configs()
				.withKP(3.0)
				.withKI(0.0)
				.withKD(0.0);

		motor.getConfigurator().apply(cfg);
	}

	public void intake() { setStateFromRequest(State.INTAKE); }
	public void feed() { setStateFromRequest(State.FEED); }
	public void reverse() { setStateFromRequest(State.REVERSE); }
	public void stop() { setStateFromRequest(State.OFF); }

	@Override
	protected State getNextState(State current) { return current; }

	@Override
	protected void afterTransition(State newState) {
		switch (newState) {
			case OFF -> motor.setControl(new VoltageOut(0.0));
 			case INTAKE -> motor.setControl(mmRequest.withVelocity(INTAKE_POWER));
			case FEED -> motor.setControl(new VoltageOut(FEED_POWER));
			case REVERSE -> motor.setControl(new VoltageOut(REVERSE_POWER));
	}
}
}
