// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberRPM extends CommandBase {

  private ClimberSubsystem climber;
  private double m_getPercentOutput;
  /** Creates a new IndexerCargo. */
  public ClimberRPM(double getPercentOutput) {

    climber = ClimberSubsystem.getInstance();
    
    // Use addRequirements() here to declare subsystem dependencies.
    if ( climber != null )
    {
      addRequirements(climber);
    }

    m_getPercentOutput = getPercentOutput;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}
    
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (climber == null){
      return;
    }
    else{
      climber.StartClimberRPM(m_getPercentOutput);
    }
  }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    if ( climber != null )
      climber.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}