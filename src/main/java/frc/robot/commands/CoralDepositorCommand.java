// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralDepositorSubsystem;

public class CoralDepositorCommand extends Command {
    private CoralDepositorSubsystem coralDepositorSubsystem;
    private boolean direction;
    private boolean bUseSensor;
  /** Creates a new IntakeControl. */

  /**
   * @brief
   * @param feedNote true to feed it, false to back it 
   * @param bUseSensor true with sensor
      * @return 
      * @return
      */
    public CoralDepositorCommand(boolean feedCoral, boolean bUseSensor) {
      direction = feedCoral;
      this.bUseSensor = bUseSensor;

      // Use addRequirements() here to declare subsystem dependencies.
      coralDepositorSubsystem = CoralDepositorSubsystem.getInstance(); 
      addRequirements(coralDepositorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (direction == true) 
      coralDepositorSubsystem.setVoltage(-0.20);
    else 
      coralDepositorSubsystem.setVoltage(0.2);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    coralDepositorSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
     
   if (bUseSensor)
   {
    //use the sensor to stop
    return coralDepositorSubsystem.isSensorActive() == false;
   }
   else
    return false;
  }
}