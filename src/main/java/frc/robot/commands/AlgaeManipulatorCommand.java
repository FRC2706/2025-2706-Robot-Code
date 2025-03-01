package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeManipulatorSubsystem;

public class AlgaeManipulatorCommand extends Command {

  private final AlgaeManipulatorSubsystem algaeManipulator;

  public AlgaeManipulatorCommand() {
    algaeManipulator = AlgaeManipulatorSubsystem.getInstance();
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(algaeManipulator);
  }



@Override
public void initialize() {}

// Called every time the scheduler runs while the command is scheduled.
@Override
public void execute() {
algaeManipulator.moveToAngle(90);
}

// Called once the command ends or is interrupted.
@Override
public void end(boolean interrupted) {
    algaeManipulator.stop();
  
}

// Returns true when the command should end.
@Override
public boolean isFinished() {
  return false;
}

}
