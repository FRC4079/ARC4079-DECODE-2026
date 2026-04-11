package frc.robot.Intake;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;

public class IntakePosition extends StateMachine<IntakePosition.State> {
	public enum State {
		OFF,
		DEPLOY,
		SPIN,
		DELAY,
		PULSE,
		RETRACT
	}



	private static final double DEPLOY_ROTATIONS = -10;
	private static final double FULL_RETRACT_ROTATIONS = -0.55;
	private static final double RETRACT_FRACTION_FROM_DEPLOY = 0.80;
	private static final double PULSE_OUT_FRACTION_FROM_DEPLOY = 0.30;
	private static final double PULSE_IN_FRACTION_FROM_DEPLOY = 0.80;
	private static final double PULSE_INTERVAL_SECONDS = 0.35;
	private static final double RETRACT_ROTATIONS =
			DEPLOY_ROTATIONS
					+ (FULL_RETRACT_ROTATIONS - DEPLOY_ROTATIONS) * RETRACT_FRACTION_FROM_DEPLOY;
	private static final double PULSE_OUT_ROTATIONS =
			DEPLOY_ROTATIONS
					+ (FULL_RETRACT_ROTATIONS - DEPLOY_ROTATIONS) * PULSE_OUT_FRACTION_FROM_DEPLOY;
	private static final double PULSE_IN_ROTATIONS =
			DEPLOY_ROTATIONS
					+ (FULL_RETRACT_ROTATIONS - DEPLOY_ROTATIONS) * PULSE_IN_FRACTION_FROM_DEPLOY;
	private static final double PIVOT_SPEED_SCALE = 0.5;

	private State pulseReturnState = State.RETRACT;

	private final Timer pulseTimer = new Timer();

	private boolean intakeToggle;
	private boolean pulseGoInward = true;

	private final TalonFX motor;
	private final MotionMagicTorqueCurrentFOC mmRequest = new MotionMagicTorqueCurrentFOC(0).withSlot(0);
	private final VelocityTorqueCurrentFOC veloRequest = new VelocityTorqueCurrentFOC(0).withSlot(0);

	public IntakePosition(TalonFX motor) {
		super(SubsystemPriority.DEPLOY, State.OFF);
		this.motor = motor;


		var cfg = new TalonFXConfiguration();
cfg.Slot0 = new Slot0Configs()
    .withKP(30).withKI(0).withKD(0)
    .withKG(0.3)
    .withKS(0.1);

cfg.MotionMagic = new MotionMagicConfigs()
	.withMotionMagicCruiseVelocity(150.0 * PIVOT_SPEED_SCALE)
	.withMotionMagicAcceleration(160.0 * PIVOT_SPEED_SCALE);
cfg.CurrentLimits = new CurrentLimitsConfigs()
    .withSupplyCurrentLimit(60)
    .withSupplyCurrentLimitEnable(true)
    .withStatorCurrentLimit(160)
    .withStatorCurrentLimitEnable(true);
	motor.getConfigurator().apply(cfg);
		motor.setPosition(0.0);

}

	/** Move elevator to deployed position. */
	public void deploy()  {
		pulseReturnState = State.DEPLOY;
		setStateFromRequest(State.DEPLOY);
	}

	/** Retract elevator back to boot/zero position. */
	public void retract() {
		pulseReturnState = State.RETRACT;
		setStateFromRequest(State.RETRACT);
	}

	public void pulse() {
		if (getState() != State.PULSE) {
			pulseReturnState = (getState() == State.DEPLOY) ? State.DEPLOY : State.RETRACT;
		}
		setStateFromRequest(State.PULSE);
	}

	public void stopPulseAndRestore() {
		if (getState() == State.PULSE) {
			setStateFromRequest(pulseReturnState);
		}
	}

	public void spin() {setStateFromRequest(State.SPIN); }

	public void toggle() {
		intakeToggle = !intakeToggle;
		if (intakeToggle){
			deploy();
		}
		else {
			retract();
		}
	}


	private void managePulse() {
		if (pulseTimer.get() > PULSE_INTERVAL_SECONDS)
		{
			pulseTimer.restart();
			motor.setControl(mmRequest.withPosition(pulseGoInward ? PULSE_IN_ROTATIONS : PULSE_OUT_ROTATIONS));
			pulseGoInward = !pulseGoInward;

		}

	}

	@Override
	protected void collectInputs() {
		if (getState() == State.PULSE) {
			managePulse();
		}
	}

	@Override
	protected State getNextState(State current) { return current; }

	@Override
	protected void afterTransition(State newState) {
		switch (newState) {
			case OFF -> motor.setControl(new CoastOut());
			case SPIN -> motor.setControl(veloRequest.withVelocity(10));
			case DEPLOY  -> motor.setControl(mmRequest.withPosition(DEPLOY_ROTATIONS));
				case RETRACT -> motor.setControl(mmRequest.withPosition(RETRACT_ROTATIONS));
			case PULSE -> {
					pulseGoInward = pulseReturnState == State.DEPLOY;
					motor.setControl(mmRequest.withPosition(pulseGoInward ? PULSE_IN_ROTATIONS : PULSE_OUT_ROTATIONS));
					pulseGoInward = !pulseGoInward;
				pulseTimer.restart();
			}
		}
	}
}
