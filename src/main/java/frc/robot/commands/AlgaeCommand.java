package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeSubsystem;


public class AlgaeCommand extends Command {

  private final AlgaeSubsystem algaeManipulator;
  private double angle;
  private double ratio;

  public AlgaeCommand(double angle, double gearRatio) {
    this.angle = angle;
    this.ratio = gearRatio;
    algaeManipulator = AlgaeSubsystem.getInstance();
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(algaeManipulator);
  }



@Override
public void initialize() {}

// Called every time the scheduler runs while the command is scheduled.
@Override
public void execute() {
  this.angle = angle * ratio;
  algaeManipulator.moveToAngle(angle);
}

// Called once the command ends or is interrupted.
@Override
public void end(boolean interrupted) {
  algaeManipulator.moveToAngle(0);
    algaeManipulator.stop();
  
}

// Returns true when the command should end.
@Override
public boolean isFinished() {
  algaeManipulator.moveToAngle(0);
  return false;
}

}
