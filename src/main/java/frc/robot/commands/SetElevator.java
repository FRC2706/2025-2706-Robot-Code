// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Config.ElevatorSetPoints;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class SetElevator extends Command {
    private ElevatorSetPoints setPos;
    ElevatorSubsystem elevatorSubsystem;
    Timer m_timer = new Timer();
    Boolean bUpDirection = true;

    /** Creates a new SetArm. */
    public SetElevator(ElevatorSetPoints setPos) {
        this.setPos = setPos;
        
        // Use addRequirements() here to declare subsystem dependencies.
        elevatorSubsystem = ElevatorSubsystem.getInstance();
        addRequirements(elevatorSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        elevatorSubsystem.setServoBrake(false);
        elevatorSubsystem.setTargetPos(setPos.position);

        bUpDirection = elevatorSubsystem.isMoveUpDirection();

        m_timer.start();
        m_timer.reset();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        elevatorSubsystem.setElevatorHeight(setPos.position, bUpDirection);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        elevatorSubsystem.stopMotor();
        elevatorSubsystem.setServoBrake(true);
        elevatorSubsystem.resetPrevPos(setPos.position);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        //check the set position AND timer
        //@todo: check the timeout value
        if (elevatorSubsystem.isAtTargetPos() == true 
            || m_timer.hasElapsed(5))
        {
            return true;
        }
        else
        {
            return false;
        }

       
    }
}

