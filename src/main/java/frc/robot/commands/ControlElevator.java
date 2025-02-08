// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

import java.util.function.DoubleSupplier;

public class ControlElevator extends Command {
    private double direction; // 0 = stop, 1 = down, 2 = up

    /** Creates a new SetArm. */
    public ControlElevator(double direction) {
        this.direction = direction;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(ElevatorSubsystem.getInstance());
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (direction == 0) {
            ElevatorSubsystem.getInstance().stopMotors();
            System.out.println("Stopped motors");
        } else if (direction == 1) {
            ElevatorSubsystem.getInstance().lowerMotor();
        } else if (direction == 2) {
            ElevatorSubsystem.getInstance().raiseMotor();
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        ElevatorSubsystem.getInstance().stopMotors();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        //keep running motor
        return false;
    }
}

