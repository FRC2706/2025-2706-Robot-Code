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
import frc.robot.commands.IntakeControl;
import frc.robot.commands.MakeIntakeMotorSpin;
import frc.robot.commands.PhotonMoveToTarget;
import frc.robot.commands.SetArm;
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
   
    PathPlannerAuto //fourNoteAuto,
                    // twoNoteAuto,
                    // threeNoteAuto,
                    //twoNoteLeftAuto,
                    //twoNoteCenter,
                    //threeNoteCenterSourceSideNote,
                    //threeNoteCenterAmpSideNote,
                    //oneNoteSourceSide,
                    Right_R_CD,
                    Right_R_CD_R,
                    RIGHTCenter_R_CD,
                    RIGHTCenter_R_CD_R,
                    LEFTCenter_R_CD,
                    LEFTCenter_R_CD_R,
                    Left_R_CD,
                    Left_R_CD_R,
                    centerMove;
                    //twoNoteSourceSide;
    

    public AutoRoutines() {
        registerCommandsToPathplanner();

        Right_R_CD = new PathPlannerAuto("rightReefCd");
        Right_R_CD_R = new PathPlannerAuto("rightReefCdReef");
        Left_R_CD = new PathPlannerAuto("leftReefCd");
        Left_R_CD_R = new PathPlannerAuto("leftReefCdReef");
        RIGHTCenter_R_CD = new PathPlannerAuto("rightCenterReefCd");
        RIGHTCenter_R_CD_R = new PathPlannerAuto("rightCenterReefCdReef");
        LEFTCenter_R_CD_R = new PathPlannerAuto("leftCenterReefCd");
        LEFTCenter_R_CD = new PathPlannerAuto("leftCenterReefCdReef");
        centerMove = new PathPlannerAuto("centerMove");

         
        // twoNoteAuto = new PathPlannerAuto("twoNoteSpeaker");
        // threeNoteAuto = new PathPlannerAuto("threeNoteSpeaker");
       
    }

    public void registerCommandsToPathplanner() {
         NamedCommands.registerCommand("purpleBling", new BlingCommand(BlingColour.PURPLE));
         NamedCommands.registerCommand("honeydewBling", new BlingCommand(BlingColour.HONEYDEW));
         NamedCommands.registerCommand("blueBling", new BlingCommand(BlingColour.BLUE));
         NamedCommands.registerCommand("redBling", new BlingCommand(BlingColour.RED));
         NamedCommands.registerCommand("yellowBling", new BlingCommand(BlingColour.YELLOW));
         NamedCommands.registerCommand("rainbowBling", new BlingCommand(BlingColour.RAINBOW));
         NamedCommands.registerCommand("fireBling", new BlingCommand(BlingColour.FIRE));
         NamedCommands.registerCommand("rgbfadeBling", new BlingCommand(BlingColour.RGBFADE));
         NamedCommands.registerCommand("whiteSBling", new BlingCommand(BlingColour.WHITESTROBE));
         NamedCommands.registerCommand("redSBling", new BlingCommand(BlingColour.REDSTROBE));
         NamedCommands.registerCommand("yellowSBling", new BlingCommand(BlingColour.YELLOWSTROBE));
         NamedCommands.registerCommand("blueSBling", new BlingCommand(BlingColour.BLUESTROBE));
         NamedCommands.registerCommand("purpleSBling", new BlingCommand(BlingColour.PURPLESTROBE));
         NamedCommands.registerCommand("disableBLING", new BlingCommand(BlingColour.DISABLED));

    }

    public Command getAutonomousCommand(int selectAuto) {
        switch (selectAuto) {
            case 0:
            default: 
                return null;
            case 1:
                return centerMove;
            case 2:
                return Right_R_CD_R;
            case 3:
                return RIGHTCenter_R_CD;
            case 4:
                return RIGHTCenter_R_CD_R;
            case 5:
                return LEFTCenter_R_CD;
            case 6:
                return LEFTCenter_R_CD_R;
            case 7:
                return Left_R_CD;
            case 8:
                return Left_R_CD_R;
            case 9: 
                return Right_R_CD;
            case 10: 
            case 11:
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
