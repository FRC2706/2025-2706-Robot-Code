package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralIntakeSubsystem;
import java.util.function.DoubleSupplier;

public class CoralIntake extends Command {

  private final DoubleSupplier leftTrigger;
  private final DoubleSupplier rightTrigger;
  private final CoralIntakeSubsystem coralIntake;

  /** Creates a new CoralIntake. */
  public CoralIntake(DoubleSupplier leftTrigger, DoubleSupplier rightTrigger) {
    this.leftTrigger = leftTrigger;
    this.rightTrigger = rightTrigger;
    this.coralIntake = CoralIntakeSubsystem.getInstance();

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(coralIntake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double leftPercent = leftTrigger.getAsDouble();
    double rightPercent = rightTrigger.getAsDouble();
    coralIntake.startIntakePercent(leftPercent, rightPercent);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    coralIntake.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}