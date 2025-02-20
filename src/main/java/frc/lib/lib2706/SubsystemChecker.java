// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.lib2706;

import frc.robot.Config;

/** Add your docs here. */
public class SubsystemChecker {

    /**
     * Name of possible subsystems
     */
    public enum SubsystemType {
        DiffNeoSubsystem,
        DiffTalonSubsystem,
        SwerveSubsystem,
        ClimberSubsystem,
        ShooterSubsystem,
        IntakeSubsystem,      
        ArmSubsystem,
        RelaySubsystem,
        BlingSubsystem,
        PhotonSubsystem,
        
    };

    /**
     * Allowed Subsystems for each robot
     */
    // RobotID: 0, 2025 Competition robot, unnamed
    private static SubsystemType[] compBotId0 = new SubsystemType[] {
        SubsystemType.SwerveSubsystem,
        SubsystemType.BlingSubsystem,
        SubsystemType.PhotonSubsystem,
    };

    // RobotID: 1, 2024 robot, Apollo
    private static SubsystemType[] apolloId1 = new SubsystemType[] {
        SubsystemType.SwerveSubsystem,
        SubsystemType.ArmSubsystem,
        SubsystemType.ClimberSubsystem,
        SubsystemType.ShooterSubsystem,       
        SubsystemType.BlingSubsystem,
        SubsystemType.PhotonSubsystem,
    };

    // RobotID: 2, Half-scale talon differential drive robot, Beetle
    private static SubsystemType[] beetleId2 = new SubsystemType[] {
        SubsystemType.DiffTalonSubsystem,  // Chassis
        SubsystemType.RelaySubsystem,
        SubsystemType.BlingSubsystem,
    };

      // RobotID: 3, ControlBox
      private static SubsystemType[] ControlBox3 = new SubsystemType[] {
        SubsystemType.BlingSubsystem,
    };

    // RobotID: 3, 2019 Comp Robot, Deep Space, Mergonaut
    private static SubsystemType[] mergonautId3 = new SubsystemType[] {
        SubsystemType.DiffTalonSubsystem,  // Chassis
    };

    // Use robotSpecific to know what robot is currently running the code
    private static SubsystemType[] activeRobotAllowedTypes = Config.robotSpecific(compBotId0, apolloId1, beetleId2, ControlBox3);

    /**
     * Check if the subsystem is allowed for the robot this is deployed onto
     * 
     * @param subsystem A SubsystemType to identify the subsystem that has been constructed
     */
    public static void subsystemConstructed(SubsystemType subsystem) {
         if (!canSubsystemConstruct(subsystem)) {
            throw new RuntimeException(String.format(
                "SUBSYSTEM INITALIZED - NOT ALLOWED ON THIS ROBOT - RobotID: %d, IllegalSubsystem: %s",
                Config.getRobotId(), subsystem.toString())
            );
        }
    }

    /**
     * Search the array of allowed subsystems to see if this subsystem is in it. 
     * 
     * @param allowed List of allowed Subsystems
     * @param subsystem The subsystem that is being checked
     * @return True is the subsystem is in the array
     */
    public static boolean canSubsystemConstruct(SubsystemType subsystem) {
        for (int i = 0; i < activeRobotAllowedTypes.length; i++) 
            if (activeRobotAllowedTypes[i].equals(subsystem)) 
                return true;      

        // Never found the subsystem in the array, return false
        return false;
    }
}