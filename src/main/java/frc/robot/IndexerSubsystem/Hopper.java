package frc.robot.IndexerSubsystem;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;


public class Hopper extends StateMachine<Hopper.HopperState> {
  public enum HopperState { OFF, INTAKE, FEED, REVERSE }

  public static final double INTAKE_POWER = 1;
  public static final double FEED_POWER = 0.7;
  public static final double REVERSE_POWER = -0.5;

  private final TalonFX Hopper;
  private double dutyPercent = FEED_POWER;

  public Hopper(TalonFX Hopper) {
    super(SubsystemPriority.DEPLOY, HopperState.OFF);
    var cfg = new TalonFXConfiguration();
		cfg.CurrentLimits = new CurrentLimitsConfigs()
				.withSupplyCurrentLimit(60)
				.withSupplyCurrentLimitEnable(true)
				.withStatorCurrentLimit(75)
				.withStatorCurrentLimitEnable(true);
		Hopper.getConfigurator().apply(cfg);
    Hopper.getConfigurator().apply(cfg);
    this.Hopper = Hopper;
  }

  public void intake() { setStateFromRequest(HopperState.INTAKE); }
  public void feed() { setStateFromRequest(HopperState.FEED); }
  public void reverse() { setStateFromRequest(HopperState.REVERSE); }
  public void stop() { setStateFromRequest(HopperState.OFF); }

  /**
   * Set an arbitrary duty cycle percent and enter FEED state.
   * Percent is clamped to [-1.0, 1.0].
   */
  public void setDutyPercent(double percent) {
    dutyPercent = Math.max(-1.0, Math.min(1.0, percent));
    setStateFromRequest(HopperState.FEED);
  }

  @Override
  protected HopperState getNextState(HopperState current) { return current; }

  @Override
  protected void afterTransition(HopperState newState) {
    switch (newState) {
      case OFF -> Hopper.setControl(new DutyCycleOut(0.0));
      case INTAKE -> Hopper.setControl(new DutyCycleOut(INTAKE_POWER));
      case FEED -> Hopper.setControl(new DutyCycleOut(FEED_POWER));
      case REVERSE -> Hopper.setControl(new DutyCycleOut(REVERSE_POWER));
    }
  }
}
