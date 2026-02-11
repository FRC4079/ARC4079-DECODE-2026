
package frc.robot.FlywheelSubsystem;

import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;

/**
 * Wrapper state machine for the Flywheel.
 * States:
 * - OFF: stop
 * - SPIN_RPM: spin to target RPM (bang-bang controller inside Flywheel)
 */
public class FlywheelStateMachine extends StateMachine<FlywheelStateMachine.State> {
  public enum State { OFF, SPIN_RPM }

  private final Flywheel flywheel;
  private double targetRpm = 0.0;

  public FlywheelStateMachine(Flywheel flywheel) {
    super(SubsystemPriority.DEPLOY, State.OFF);
    this.flywheel = flywheel;
  }

  public void requestOff() { setStateFromRequest(State.OFF); }
  public void requestRpm(double rpm) {
    this.targetRpm = rpm;
    setStateFromRequest(State.SPIN_RPM);
  }

  @Override
  protected State getNextState(State current) { return current; }

  @Override
  protected void afterTransition(State newState) {
    switch (newState) {
      case OFF -> flywheel.stop();
      case SPIN_RPM -> flywheel.spinFlywheel(targetRpm);
    }
  }
}
