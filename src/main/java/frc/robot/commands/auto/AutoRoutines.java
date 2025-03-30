package frc.robot.commands.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.lib.lib2706.SelectByAllianceCommand;
import frc.robot.Config;
import frc.robot.Config.ArmConfig;
import frc.robot.Config.PhotonConfig;
import frc.robot.Config.PhotonConfig.PhotonPositions;
import frc.robot.commands.CombinedCommands;
import frc.robot.commands.CoralDepositorCommand;
import frc.robot.commands.IntakeControl;
import frc.robot.commands.MakeIntakeMotorSpin;
import frc.robot.commands.PhotonMoveToTarget;
import frc.robot.commands.PhotonMoveToTargetLeft;
import frc.robot.commands.SetArm;
import frc.robot.commands.SetElevator;
import frc.robot.commands.Shooter_PID_Tuner;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PhotonSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.commands.BlingCommand;
import frc.robot.commands.BlingCommand.BlingColour;

public class AutoRoutines extends SubsystemBase {
    
    // PathPlannerPath speakerPath = PathPlannerPath.fromPathFile("Speaker Path");
   
    PathPlannerAuto 
                    oneL4Coral_leftBlue,
                    oneL4Coral_rightBlue,
                    oneL4Coral_CD,
                    twoL4Coral,
                    threeL4Coral,
                    testVisionAuto,
                    NOVISION_oneL4Coral_leftBlue,
                    centerMove;
    

    public AutoRoutines() {
        registerCommandsToPathplanner();
        centerMove = new PathPlannerAuto("centerMove"); // blue side, starts in center of starting line and goes forward to reef
        oneL4Coral_leftBlue = new PathPlannerAuto("oneL4Coral-leftBlue"); // blue side, starts on the left of starting line (robot perspective) and goes to right angled side of top half of reef
        oneL4Coral_rightBlue = new PathPlannerAuto("oneL4Coral-rightBlue"); // blue side, starts on the right of starting line (robot perspective) and goes to right angled side bottom half of reef
        oneL4Coral_CD = new PathPlannerAuto("oneL4Coral-CD"); // blue side, starts on the left of starting line (robot perspective), goes to right angled side of top half of reef, and then to the coral depot
        twoL4Coral= new PathPlannerAuto("twoL4Coral"); // blue side, starts on the left of starting line (robot perspective) and puts two coral on L4 reef
        threeL4Coral = new PathPlannerAuto("threeL4Coral"); // blue side, starts on the left of starting line (robot perspective) and puts three coral on L4 reef
        testVisionAuto = new PathPlannerAuto(("vision"));
        NOVISION_oneL4Coral_leftBlue = new PathPlannerAuto("NOVISION-oneL4Coral-leftBlue"); // blue side, starts on the left of starting line (robot perspective) and goes to right angled side of top half of reef

    }

    public void registerCommandsToPathplanner() {
         NamedCommands.registerCommand("purpleBling", new BlingCommand(BlingColour.PURPLE));
         NamedCommands.registerCommand("honeydewBling", new BlingCommand(BlingColour.HONEYDEW));
         NamedCommands.registerCommand("redBling", new BlingCommand(BlingColour.RED));

         NamedCommands.registerCommand("elevatorL2",new SetElevator(Config.ElevatorSetPoints.L2));

         NamedCommands.registerCommand("elevatorL3",new SetElevator(Config.ElevatorSetPoints.L3));
         NamedCommands.registerCommand("elevatorL4",new SetElevator(Config.ElevatorSetPoints.AUTO_L4));
         NamedCommands.registerCommand("elevatorIntake",new SetElevator(Config.ElevatorSetPoints.FEEDER).withTimeout(2));

        // NamedCommands.registerCommand("coralIntake", new CoralIntake(-0.3,  0.3).withTimeout(1.5));
         NamedCommands.registerCommand("CoralScore", new CoralDepositorCommand(true, false).withTimeout(1));
         NamedCommands.registerCommand("reset", PhotonSubsystem.getInstance().getResetCommand());
         NamedCommands.registerCommand("vision-move",new PhotonMoveToTargetLeft(false, false, false));
    }

    public Command getAutonomousCommand(int selectAuto) {
        switch (selectAuto) {
            case 0:
            default: 
                return null;
            case 1:
                return centerMove;
            case 2:
                return oneL4Coral_leftBlue;
            case 3:
                return oneL4Coral_rightBlue;
            case 4:
                return testVisionAuto;
            case 5:
                return NOVISION_oneL4Coral_leftBlue;
            case 6:
                return threeL4Coral;
            case 7:
                return oneL4Coral_CD;
            case 8:
                return twoL4Coral;
            case 9:
                var alliance = DriverStation.getAlliance();

                 // Default to blue alliance
                 if (alliance.isEmpty()) {
                 DriverStation.reportWarning("Unable to detect alliance color.", false);
                     return new InstantCommand();
                 }
                 return Commands.sequence(
                     SwerveSubsystem.getInstance().setOdometryCommand(new Pose2d(0, 0, SwerveSubsystem.rotateForAlliance(Rotation2d.fromDegrees(0)))),
                     SwerveSubsystem.getInstance().getDriveToPoseCommand(new Pose2d((alliance.get() == DriverStation.Alliance.Blue)? 2.5 : -2.5, 0, SwerveSubsystem.rotateForAlliance(Rotation2d.fromDegrees(0))))
                 );
         }
    }

    
}
