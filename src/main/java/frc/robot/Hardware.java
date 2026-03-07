package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.vision.limelight.Limelight;
import frc.robot.vision.limelight.LimelightModel;
import frc.robot.vision.limelight.LimelightState;

public class Hardware {

  public final CommandXboxController driverController = new CommandXboxController(0);
  public final CommandXboxController testController   = new CommandXboxController(1);

  // testing only

  public final TalonFX flywheelA1        = new TalonFX(4);   // shooter bottom
  public final TalonFX flywheelA2        = new TalonFX(6);   // shooter top spin
  public final TalonFX hopperMotor       = new TalonFX(3);   // hopper
  public final TalonFX hoodMotor         = new TalonFX(5);   // shooter hood
  public final TalonFX indexerMotor      = new TalonFX(7);   // indexer
  public final TalonFX intakePivotMotor  = new TalonFX(2);   // intake pivot
  public final TalonFX intakeRollerMotor = new TalonFX(16);  // intake rollers

  public final Limelight leftLimelight  = new Limelight("left",  LimelightState.TAGS, LimelightModel.FOUR);
  public final Limelight rightLimelight = new Limelight("right", LimelightState.TAGS, LimelightModel.FOUR);

}
