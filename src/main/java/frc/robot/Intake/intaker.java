package frc.robot.Intake;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import com.ctre.phoenix6.signals.MotorAlignmentValue;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;

public class intaker extends StateMachine<intaker.State> {
	public enum State { OFF, INTAKE, FEED, REVERSE }

	private static final double INTAKE_POWER = 4000;
	private static final double FEED_POWER = 6.5 * 0.75;
	private static final double REVERSE_POWER = -4 * 0.75;

	private final TalonFX motorA;
	private final TalonFX motorB;
	private final VelocityTorqueCurrentFOC mmRequest = new VelocityTorqueCurrentFOC(0).withSlot(0);

	public intaker(TalonFX motorA, TalonFX motorB) {
		super(SubsystemPriority.DEPLOY, State.OFF);
		this.motorA = motorA;
		this.motorB = motorB;

		var cfg = new TalonFXConfiguration();

		cfg.CurrentLimits = new CurrentLimitsConfigs()
				.withSupplyCurrentLimit(40)
				.withSupplyCurrentLimitEnable(true)
				.withStatorCurrentLimit(60)
				.withStatorCurrentLimitEnable(true);

		cfg.ClosedLoopRamps = new ClosedLoopRampsConfigs()
				.withDutyCycleClosedLoopRampPeriod(0.5)
				.withTorqueClosedLoopRampPeriod(0.5)
				.withVoltageClosedLoopRampPeriod(0.5);

		cfg.OpenLoopRamps = new OpenLoopRampsConfigs()
				.withDutyCycleOpenLoopRampPeriod(0.2)
				.withTorqueOpenLoopRampPeriod(0.2)
				.withVoltageOpenLoopRampPeriod(0.2);

		cfg.Slot0 = new Slot0Configs()
				.withKP(3.0)
				.withKI(0.0)
				.withKD(0.0);

		motorA.getConfigurator().apply(cfg);



		motorB.getConfigurator().apply(cfg);
		motorB.setControl(new Follower(motorA.getDeviceID(), MotorAlignmentValue.Opposed));
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
			case OFF -> {
				motorA.setControl(new VoltageOut(0.0));
//				motorB.setControl(new VoltageOut(0.0));
			}
			case INTAKE -> {
				motorA.setControl(new VoltageOut(10));
//				motorA.setControl(mmRequest.withVelocity(INTAKE_POWER));
//				motorB.setControl(mmRequest.withVelocity(INTAKE_POWER));
			}
			case FEED -> {
				motorA.setControl(new VoltageOut(FEED_POWER));
//				motorB.setControl(new VoltageOut(FEED_POWER));
			}
			case REVERSE -> {
				motorA.setControl(new VoltageOut(REVERSE_POWER));
//				motorB.setControl(new VoltageOut(REVERSE_POWER));
			}
		}
	}
}
