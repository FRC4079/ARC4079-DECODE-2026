package frc.robot.FlywheelSubsystem;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Flywheel {
	private final TalonFX a1, a2;

	// ── Motion Magic FOC velocity request (reused each cycle) ────
	private final MotionMagicVelocityTorqueCurrentFOC mmVelocity =
			new MotionMagicVelocityTorqueCurrentFOC(0).withSlot(0);

	// ── Tuning constants ─────────────────────────────────────────
	// Slot 0 gains — tune these on the real robot
	private static final double kS = 0.0;   // static friction offset (amps)
	private static final double kV = 0.6;  // velocity feed-forward (amps per rps)
	private static final double kA = 0.0;   // acceleration feed-forward (amps per rps²)
	private static final double kP = 0;   // proportional (amps per rps of error)
	private static final double kI = 0.0;   // integral
	private static final double kD = 0;   // derivative

	// Motion-profile constraints (units: rps/s and rps/s²)
	private static final double MM_ACCELERATION = 400.0; // rps/s
	private static final double MM_JERK         = 4000.0; // rps/s² (0 = unlimited)

	// Current limits
	private static final double SUPPLY_LIMIT = 80.0;
	private static final double STATOR_LIMIT = 100.0;
	private static final double TORQUE_CURRENT_LIMIT = 80; // peak amps for FOC

	/** RPM tolerance for the "at goal" flag. */
	private static final double RPM_TOLERANCE = 50.0;

	// ── State ────────────────────────────────────────────────────
	private final Debouncer atGoalDebounce =
			new Debouncer(0.2, Debouncer.DebounceType.kFalling);

	private double targetRpm = 0.0;
	private boolean atGoal   = false;
	private double liveKP = kP;
	private double liveKD = kD;
	private double liveKV = kV;

	public Flywheel(TalonFX a1, TalonFX a2) {
		this.a1 = a1;
		this.a2 = a2;

		var cfg = new TalonFXConfiguration();

		// --- PID + FF gains (Slot 0) ---
		cfg.Slot0 = new Slot0Configs()
				.withKS(kS)
				.withKV(kV)
				.withKA(kA)
				.withKP(kP)
				.withKI(kI)
				.withKD(kD);

		// --- Motion Magic profile ---
		cfg.MotionMagic = new MotionMagicConfigs()
				.withMotionMagicAcceleration(MM_ACCELERATION)
				.withMotionMagicJerk(MM_JERK);

		// --- Supply / stator current limits ---
		cfg.CurrentLimits = new CurrentLimitsConfigs()
				.withSupplyCurrentLimit(SUPPLY_LIMIT)
				.withSupplyCurrentLimitEnable(true)
				.withStatorCurrentLimit(STATOR_LIMIT)
				.withStatorCurrentLimitEnable(true);

		// --- Torque-current limits (used by FOC control modes) ---
		cfg.TorqueCurrent = new TorqueCurrentConfigs()
				.withPeakForwardTorqueCurrent(TORQUE_CURRENT_LIMIT)
				.withPeakReverseTorqueCurrent(-TORQUE_CURRENT_LIMIT);

		a1.getConfigurator().apply(cfg);
		a2.getConfigurator().apply(cfg);

		// Publish initial PID values for live tuning
		SmartDashboard.putNumber("Flywheel/Tuning/kP", kP);
		SmartDashboard.putNumber("Flywheel/Tuning/kD", kD);
		SmartDashboard.putNumber("Flywheel/Tuning/kV", kV);
	}

	/**
	 * Command the flywheel to a target RPM using Motion Magic FOC velocity.
	 * The Talon's on-board profile generator ramps to the target while the
	 * FOC loop tracks the profiled setpoint every cycle.
	 */
	public void spinFlywheel(double rpm) {
		targetRpm = Math.max(0.0, rpm);

		double measured = getRpm();
		double error    = targetRpm - measured;
		boolean inTol   = Math.abs(error) <= RPM_TOLERANCE;
		atGoal = atGoalDebounce.calculate(inTol);

		if (targetRpm <= 1e-3) {
			stop();
			return;
		}

		// Convert RPM → rotations per second (Phoenix 6 native velocity unit)
		double targetRps = targetRpm / 60.0;
		var request = mmVelocity.withVelocity(targetRps);

		a1.setControl(request);
		a2.setControl(request);
	}

	public void stop() {
		targetRpm = 0.0;
		atGoal = false;
		a1.setControl(new NeutralOut());
		a2.setControl(new NeutralOut());
	}

	public double getRpm() {
		double v1 = safeVel(a1);
		double v2 = safeVel(a2);
		double rpsAvg = (Math.abs(v1) + Math.abs(v2)) / 2.0;
		return rpsAvg * 60.0;
	}

	public boolean isAtGoal() { return atGoal; }

	private static double safeVel(TalonFX fx) {
		try {
			return fx.getRotorVelocity().getValueAsDouble();
		} catch (Exception ex) {
			return 0.0;
		}
	}

	public void periodicTelemetry() {
		SmartDashboard.putNumber("Flywheel/ActualRPM", getRpm());
		SmartDashboard.putNumber("Flywheel/TargetRPM", targetRpm);
		SmartDashboard.putBoolean("Flywheel/AtGoal", atGoal);

		// Live PID tuning from SmartDashboard
		double newKP = SmartDashboard.getNumber("Flywheel/Tuning/kP", liveKP);
		double newKD = SmartDashboard.getNumber("Flywheel/Tuning/kD", liveKD);
		double newKV = SmartDashboard.getNumber("Flywheel/Tuning/kV", liveKV);
		if (newKP != liveKP || newKD != liveKD || newKV != liveKV) {
			liveKP = newKP;
			liveKD = newKD;
			liveKV = newKV;
			var newSlot0 = new Slot0Configs()
					.withKS(kS).withKV(liveKV).withKA(kA)
					.withKP(liveKP).withKI(kI).withKD(liveKD);
			a1.getConfigurator().apply(newSlot0);
			a2.getConfigurator().apply(newSlot0);
		}
	}
}
