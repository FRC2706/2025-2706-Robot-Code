// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

//imports
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.Config.PhotonConfig;
import frc.robot.subsystems.PhotonSubsystem;

//class
public class PhotonMoveToTarget extends Command {
  //declerations
  Translation2d targetOffset;
  boolean centerTarget;
  boolean isWaypoint;
  boolean shouldNeverEnd;

  private Timer m_timer = new Timer();

/**
 * Command for moving to the target currently selected in the PhotonSubsystem. Without a desired heading, the robot turns so that the camera faces the target.
 * @param _targetOffset
 * the field oriented offset from the aprilTag to move towards
 * @param _isWaypoint
 * whether or not to use the larger tolerences meant for stop-and-go waypoints
 */
  public PhotonMoveToTarget(Translation2d _targetOffset, boolean _isWaypoint, boolean neverEnd) {
    addRequirements(SwerveSubsystem.getInstance());
    addRequirements(PhotonSubsystem.getInstance());
    targetOffset = _targetOffset;
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
  public PhotonMoveToTarget(Translation2d _targetOffset, boolean _centerTarget, boolean _isWaypoint, boolean neverEnd) {
    addRequirements(SwerveSubsystem.getInstance());
    addRequirements(PhotonSubsystem.getInstance());
    targetOffset = _targetOffset;
    centerTarget=_centerTarget;
    isWaypoint=_isWaypoint;
    shouldNeverEnd = neverEnd;
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SwerveSubsystem.getInstance().resetDriveToPose();
    m_timer.restart();

    // m_timer.start();
    // m_timer.reset();
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Translation2d setPoint = PhotonSubsystem.getInstance().getTargetPos();
    //Translation2d setPoint = PhotonSubsystem.getInstance().getNewTargetPos();

    Rotation2d targetRotation = PhotonSubsystem.getInstance().getTargetRotation();
    Rotation2d targetRobotHeading = PhotonSubsystem.getInstance().getTargetRobotHeading();

    //@todo: 
    //rotationSetPoint = current robot heading + yaw. Note yaw needs to keep updating
    //convert to Robot: facing to AprilTag
    Rotation2d rotationSetPoint = Rotation2d.fromDegrees(targetRotation.getDegrees()+180);     
        
    if (centerTarget){
      SwerveSubsystem.getInstance().driveToPose(new Pose2d(setPoint.plus(targetOffset), rotationSetPoint));
    }else{
      //use the absolute heading
      SwerveSubsystem.getInstance().driveToPose(new Pose2d(setPoint.plus(targetOffset), targetRobotHeading));
    }
  }
 

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_timer.stop();
  }

  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (shouldNeverEnd) {
      return false;
    }
    else
    {
      //if no vision data, stop
      return (PhotonSubsystem.getInstance().hasTarget()==false 
      || SwerveSubsystem.getInstance().isAtTargetPose(PhotonSubsystem.getInstance().getTargetPos()) 
      || m_timer.hasElapsed(1.5)
      );
    }

    //Note: if isAtTargetPose() works, may not need to hasTarget().
    
    
    // if (isWaypoint){
    //   return SwerveSubsystem.getInstance().isAtPose(PhotonConfig.WAYPOINT_POS_TOLERANCE, PhotonConfig.WAYPOINT_ANGLE_TOLERANCE);
    // } else {
    //   return SwerveSubsystem.getInstance().isAtPose(PhotonConfig.POS_TOLERANCE, PhotonConfig.ANGLE_TOLERANCE) 
    //       && !SwerveSubsystem.getInstance().isChassisMoving(PhotonConfig.VEL_TOLERANCE)
    //       && (PhotonSubsystem.getInstance().hasData()==false);
    // }
  

    
  }
}
