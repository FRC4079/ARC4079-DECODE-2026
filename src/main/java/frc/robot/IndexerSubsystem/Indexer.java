package frc.robot.IndexerSubsystem;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;

public class Indexer extends StateMachine<Indexer.IndexerState> {
  public enum IndexerState { OFF, INTAKE, FEED, REVERSE }

  public static final double INTAKE_POWER = 1;
  public static final double FEED_POWER = 1;
  public static final double REVERSE_POWER = -0.5;
  private static final double SUPPLY_CURRENT_LIMIT_AMPS = 30.0;
  private static final double STATOR_CURRENT_LIMIT_AMPS = 40.0;

  private final TalonFX Indexer;
  private double dutyPercent = FEED_POWER;

  public Indexer(TalonFX Indexer) {
    super(SubsystemPriority.DEPLOY, IndexerState.OFF);
    var cfg = new TalonFXConfiguration();
    cfg.CurrentLimits = new CurrentLimitsConfigs()
        .withSupplyCurrentLimit(SUPPLY_CURRENT_LIMIT_AMPS)
        .withSupplyCurrentLimitEnable(true)
        .withStatorCurrentLimit(STATOR_CURRENT_LIMIT_AMPS)
        .withStatorCurrentLimitEnable(true);
    Indexer.getConfigurator().apply(cfg);
    this.Indexer = Indexer;
  }

  public void intake() { setStateFromRequest(IndexerState.INTAKE); }
  public void feed() { setStateFromRequest(IndexerState.FEED); }
  public void reverse() { setStateFromRequest(IndexerState.REVERSE); }
  public void stop() { setStateFromRequest(IndexerState.OFF); }

  /**
   * Set an arbitrary duty cycle percent and enter FEED state.
   * Percent is clamped to [-1.0, 1.0].
   */
  public void setDutyPercent(double percent) {
    dutyPercent = Math.max(-1.0, Math.min(1.0, percent));
    setStateFromRequest(IndexerState.FEED);
  }

  @Override
  protected IndexerState getNextState(IndexerState current) { return current; }

  @Override
  protected void afterTransition(IndexerState newState) {
    switch (newState) {
      case OFF -> Indexer.setControl(new DutyCycleOut(0.0));
      case INTAKE -> Indexer.setControl(new DutyCycleOut(INTAKE_POWER));
      case FEED -> Indexer.setControl(new DutyCycleOut(dutyPercent));
      case REVERSE -> Indexer.setControl(new DutyCycleOut(REVERSE_POWER));
    }
  }
}
