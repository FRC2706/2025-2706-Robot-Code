// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ResetElevator extends Command {

  ElevatorSubsystem elevatorSubsystem;
  double setPercent = 0;
  /** Creates a new ResetElevator. */
  public ResetElevator(double percent) {
    // Use addRequirements() here to declare subsystem dependencies.
    elevatorSubsystem = ElevatorSubsystem.getInstance();
    addRequirements(elevatorSubsystem);
    setPercent = percent;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevatorSubsystem.setServoBrake(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevatorSubsystem.setPercent(setPercent);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevatorSubsystem.stopMotor(); 
    elevatorSubsystem.setServoBrake(true);

    if(elevatorSubsystem.isLimitSwitchPressed())
    {
      elevatorSubsystem.resetPrevPos(0);
    }    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return false;
  
  }
}
