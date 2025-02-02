// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class SetElevator extends Command {
    private DoubleSupplier extensionAmt;

    /** Creates a new SetArm. */
    public SetElevator(DoubleSupplier extensionAmt) {
        this.extensionAmt = extensionAmt;
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
        ElevatorSubsystem.getInstance().setElevatorHeight(Math.toRadians(extensionAmt.getAsDouble()));
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

