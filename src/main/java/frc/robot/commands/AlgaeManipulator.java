package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeMainpulatorSubsystem;


public class AlgaeManipulator extends Command {

  private final AlgaeMainpulatorSubsystem algaeManipulator;
  private double angle;
  private double ratio;

  public AlgaeManipulator(double angle, double gearRatio) {
    this.angle = angle;
    this.ratio = gearRatio;
    algaeManipulator = AlgaeMainpulatorSubsystem.getInstance();
    
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
    algaeManipulator.stop();
  
}

// Returns true when the command should end.
@Override
public boolean isFinished() {
  return false;
}

}

