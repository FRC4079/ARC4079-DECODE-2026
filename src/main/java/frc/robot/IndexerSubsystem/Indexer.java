package frc.robot.IndexerSubsystem;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;

public class Indexer extends StateMachine<Indexer.IndexerState> {
  public enum IndexerState { OFF, INTAKE, FEED, REVERSE }

  public static final double INTAKE_POWER = 0.6;
  public static final double FEED_POWER = 0.4;
  public static final double REVERSE_POWER = -0.5;

  private final TalonFX motor;

  public Indexer(TalonFX motor) {
    super(SubsystemPriority.DEPLOY, IndexerState.OFF);
    this.motor = motor;
  }

  public void intake() { setStateFromRequest(IndexerState.INTAKE); }
  public void feed() { setStateFromRequest(IndexerState.FEED); }
  public void reverse() { setStateFromRequest(IndexerState.REVERSE); }
  public void stop() { setStateFromRequest(IndexerState.OFF); }

  @Override
  protected IndexerState getNextState(IndexerState current) { return current; }

  @Override
  protected void afterTransition(IndexerState newState) {
    switch (newState) {
      case OFF -> motor.setControl(new DutyCycleOut(0.0));
      case INTAKE -> motor.setControl(new DutyCycleOut(INTAKE_POWER));
      case FEED -> motor.setControl(new DutyCycleOut(FEED_POWER));
      case REVERSE -> motor.setControl(new DutyCycleOut(REVERSE_POWER));
    }
  }
}
