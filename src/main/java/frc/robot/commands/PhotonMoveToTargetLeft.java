// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

//imports
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.Config.PhotonConfig;
import frc.robot.subsystems.PhotonSubsystemLeftReef;

//class
public class PhotonMoveToTargetLeft extends Command {
  //declerations
  Translation2d targetOffset;
  boolean centerTarget;
  boolean isWaypoint;
  boolean shouldNeverEnd;

/**
 * Command for moving to the target currently selected in the PhotonSubsystem. Without a desired heading, the robot turns so that the camera faces the target.
 * @param _targetOffset
 * the field oriented offset from the aprilTag to move towards
 * @param _isWaypoint
 * whether or not to use the larger tolerences meant for stop-and-go waypoints
 */
  public PhotonMoveToTargetLeft(boolean _isWaypoint, boolean neverEnd) {
    addRequirements(SwerveSubsystem.getInstance());
    addRequirements(PhotonSubsystemLeftReef.getInstance());
    centerTarget=true;
    isWaypoint=_isWaypoint;
    shouldNeverEnd = neverEnd;
  }

/**
 * Command for moving to the target currently selected in the PhotonSubsystem
 * @param _targetOffset
 * the field oriented offset from the aprilTag to move towards

 * @param _isWaypoint
 * whether or not to use the larger tolerences meant for stop-and-go waypoints
 */
  public PhotonMoveToTargetLeft(boolean _centerTarget, boolean _isWaypoint, boolean neverEnd) {
    addRequirements(SwerveSubsystem.getInstance());
    addRequirements(PhotonSubsystemLeftReef.getInstance());
    centerTarget=_centerTarget;
    isWaypoint=_isWaypoint;
    shouldNeverEnd = neverEnd;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SwerveSubsystem.getInstance().resetDriveToPose();
    targetOffset = PhotonSubsystemLeftReef.getInstance().getTargetOffset();  
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    try {
      Translation2d setPoint = PhotonSubsystemLeftReef.getInstance().getTargetPos();

      Rotation2d targetRotation = PhotonSubsystemLeftReef.getInstance().getTargetRotation();
      Rotation2d targetRobotHeading = PhotonSubsystemLeftReef.getInstance().getTargetRobotHeading();

      //@todo:
      //rotationSetPoint = current robot heading + yaw. Note yaw needs to keep updating
      //convert to Robot: facing to AprilTag
      Rotation2d rotationSetPoint = Rotation2d.fromDegrees(targetRotation.getDegrees() + 180);

      Rotation2d desiredRotation;
      if (centerTarget) {
        desiredRotation = rotationSetPoint;
      } else {
        desiredRotation = targetRobotHeading;
      }

      // Grab the latest target offset.
      targetOffset = PhotonSubsystemLeftReef.getInstance().getTargetOffset();

      Pose2d desiredPose = new Pose2d(setPoint.plus(targetOffset), desiredRotation);

      SwerveSubsystem.getInstance().driveToPose(desiredPose);
    } catch (Exception e) {
      System.out.println(e);
    }
  }
 

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
     {      
       return false;  

    }
      
  }
}
