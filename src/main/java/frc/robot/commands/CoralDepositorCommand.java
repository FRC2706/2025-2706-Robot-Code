// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralDepositorControl;
import frc.robot.subsystems.IntakeSubsystem;

public class CoralDepositorCommand extends Command {
    private CoralDepositorControl coralDepositorControl;
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
    public CoralDepositorCommand(boolean feedNote) {
      this.direction = feedNote;
      coralDepositorControl = CoralDepositorControl.getInstance();

    // Use addRequirements() here to declare subsystem dependencies.
    if (coralDepositorControl != null) {
      addRequirements(coralDepositorControl);
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (direction == true) 
      coralDepositorControl.setVoltage(4.0);
    else 
      coralDepositorControl.setVoltage(-2.0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  coralDepositorControl.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (direction)
      // return intakeSubsystem.isSensor7True() == false;
      return false;
    else
      return coralDepositorControl.isSensorActive() == false;
  }
}