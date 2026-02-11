package frc.robot.IndexerSubsystem;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;


public class Hopper extends StateMachine<Hopper.HopperState> {
  public enum HopperState { OFF, INTAKE, FEED, REVERSE }

  public static final double INTAKE_POWER = 0.6;
  public static final double FEED_POWER = 0.4;
  public static final double REVERSE_POWER = -0.5;

  private final TalonFX motor;

  public Hopper(TalonFX motor) {
    super(SubsystemPriority.DEPLOY, HopperState.OFF);
    this.motor = motor;
  }

  public void intake() { setStateFromRequest(HopperState.INTAKE); }
  public void feed() { setStateFromRequest(HopperState.FEED); }
  public void reverse() { setStateFromRequest(HopperState.REVERSE); }
  public void stop() { setStateFromRequest(HopperState.OFF); }

  @Override
  protected HopperState getNextState(HopperState current) { return current; }

  @Override
  protected void afterTransition(HopperState newState) {
    switch (newState) {
      case OFF -> motor.setControl(new DutyCycleOut(0.0));
      case INTAKE -> motor.setControl(new DutyCycleOut(INTAKE_POWER));
      case FEED -> motor.setControl(new DutyCycleOut(FEED_POWER));
      case REVERSE -> motor.setControl(new DutyCycleOut(REVERSE_POWER));
    }
  }
}
