// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralIntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ManipulateCoralIntake extends Command {
  /** Creates a new ManipulateCoralIntake. */
  double leftPercent;
  double rightPercent;
  int counter;

  CoralIntakeSubsystem coralIntake = null;

  public ManipulateCoralIntake() {

    coralIntake = CoralIntakeSubsystem.getInstance();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(coralIntake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    counter = 0;
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    counter ++;

    if (counter < 25)
    {  
      coralIntake.startIntakePercent(-0.3, -0.3);
    }
    else if (counter < 75)
    {
      coralIntake.startIntakePercent(-0.3, 0.3);
    }
    else if (counter < 100)
    {
      coralIntake.startIntakePercent(0.3, 0.3);

    }

    if (counter == 100)
    {
      counter = 0;
    }


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
