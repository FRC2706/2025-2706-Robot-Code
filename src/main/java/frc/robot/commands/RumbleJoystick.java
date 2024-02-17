// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RumbleJoystick extends Command {
  private Timer m_timer = new Timer();
  private CommandXboxController m_joystick;
  private RumbleType rumbleType;
  private double strength;
  private double duration;
  private boolean doDoubleRumble;
  private double counter;
  private double x;
  //private set variables here before
  /** Creates a new RumbleJoystick. */
  public RumbleJoystick(CommandXboxController joystick, RumbleType rumbleType, double strength, double duration, boolean doDoubleRumble) {
    /**
     strength: strength of rumble
            value between 0 and 1
     
     */
    counter = 0;
    x = 0;
    this.m_joystick = joystick;
    this.rumbleType = rumbleType;
    this.strength = strength;
    this.duration = duration;
    this.doDoubleRumble = doDoubleRumble;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.restart();
    m_joystick.getHID().setRumble(rumbleType, strength);
    
  }     //reset timer here 

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (doDoubleRumble == true) {
      
      if (m_timer.hasElapsed(x+(duration/3))){
        x+=(duration/3);
        counter++;
    }
      if (counter ==1) {
        m_joystick.getHID().setRumble(rumbleType,0);
      }
      if (counter ==2){
        m_joystick.getHID().setRumble(rumbleType,strength);
      }
    }

  } //autocalled every 20 mils

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_joystick.getHID().setRumble(rumbleType,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() { //
    return m_timer.hasElapsed(duration);
  }
}
