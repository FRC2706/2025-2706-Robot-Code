package frc.robot.commands.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.IntakeControl;
import frc.robot.commands.MakeIntakeMotorSpin;
import frc.robot.commands.Shooter_tuner;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoRoutines extends SubsystemBase {
    PathPlannerPath path1 = PathPlannerPath.fromPathFile("4 note");
    PathPlannerPath path2 = PathPlannerPath.fromPathFile("Diagonal45Degrees");
    PathPlannerPath BenPath = PathPlannerPath.fromPathFile("ben ");
    PathPlannerPath SpeakerPath = PathPlannerPath.fromPathFile("Speaker Path");
    PathPlannerAuto SequentialAutoTest = new PathPlannerAuto("Sequential Auto Test");
    PathPlannerAuto ParallelAutoTest = new PathPlannerAuto("Parallel Auto Test");
    PathPlannerAuto SequentialAndParallelAutoTest = new PathPlannerAuto("Sequential and Parallel Auto Test");
    PathPlannerAuto OneNoteTest = new PathPlannerAuto("One Note");
    //PathPlannerAuto tune = new PathPlannerAuto("tuningAuto");
    PathPlannerAuto testIntakeMotor = new PathPlannerAuto("MakeIntakeMotorSpin Auto Test");
    PathPlannerAuto twoNoteAuto = new PathPlannerAuto("TwoNoteSpeaker");
    private static IntakeSubsystem intake = IntakeSubsystem.getInstance();

    public AutoRoutines() {
        
    }

    // private static AutoRoutines instance;
    // public static AutoRoutines getInstance(){
    //     if(instance == null){
    //         instance = new AutoRoutines();
    //     }
    //     return instance;
    // }

    public static void registerCommandsToPathplanner() {
        IntakeSubsystem.getInstance().setDefaultCommand(IntakeSubsystem.getInstance().autoIntake());
        // Intake and Arm Commands
        NamedCommands.registerCommand("IntakeAndArm", new ParallelCommandGroup(
            new WaitCommand(1), // Move arm to intake setpoint
            new WaitCommand(1) // Intake game piece
        ));

        NamedCommands.registerCommand("OutakeRing", new ParallelCommandGroup(
            new WaitCommand(1), // Move arm to speaker 
            new WaitCommand(1) // Outake game piece
        ));

        NamedCommands.registerCommand("StartingZoneAmp", new ParallelCommandGroup(
            new WaitCommand(1), // Exit starting zone
            new WaitCommand(1), // Intake note
            new WaitCommand(1) // Score in amp
        ));

        NamedCommands.registerCommand("IntakeAndArm", new ParallelCommandGroup(
            new WaitCommand(1), // Move arm to intake setpoint
            new WaitCommand(1) // Intake game piece
        ));

        NamedCommands.registerCommand("MakeIntakeMotorSpin", new SequentialCommandGroup(
            new MakeIntakeMotorSpin(3.0,2), // Move arm to intake setpoint
            new WaitCommand(1)
        ));

        
        NamedCommands.registerCommand("shooter", new SequentialCommandGroup(
            Commands.deadline(
                Commands.sequence(
                    Commands.waitSeconds(2), 
                    IntakeSubsystem.getInstance().shootNote()
                ),
                new Shooter_tuner(()->5)
            )
        ));

        NamedCommands.registerCommand("simpleShooter", new ParallelCommandGroup
        (Commands.parallel(
              Commands.sequence(
                new IntakeControl(false), 
                new IntakeControl(true).withTimeout(2)),
              new Shooter_tuner(()->5)
            )));

        // NamedCommands.registerCommand("turnOffIntake", (
        //     Commands.runOnce(()-> IntakeSubsystem.getInstance().setMode(STOP))));
        
        // NamedCommands.registerCommand("turnOnIntake", (
            
        //         Commands.runOnce(()-> IntakeSubsystem.getInstance().setMode(INTAKE))));

        NamedCommands.registerCommand("simpleIntake", (
                new MakeIntakeMotorSpin(3.0,0)));

        // NamedCommands.registerCommand("alignToSpeaker", (
        //     PhotonSubsystem.getInstance().getAprilTagCommand(PhotonPositions.FAR_SPEAKER_RED)));


    }
        






    

    public Command getAutonomousCommand(int selectAuto) {
        switch (selectAuto) {
            case 0:
            default: 
                return null;
            case 1: 
                if (path1 == null) {
                    System.out.println("path1 path is null");
                    return null;
                }
                return Commands.sequence(
                    SwerveSubsystem.getInstance().setOdometryCommand(path1.getPreviewStartingHolonomicPose()),
                    AutoBuilder.followPath(path1)
                );
            case 2:
                return new PathPlannerAuto("testAuto");
            case 3:
                return SequentialAutoTest;
            case 4:
                return ParallelAutoTest;
            case 5:
                return SequentialAndParallelAutoTest;
            case 6:
                return testIntakeMotor;
            case 7:
                return Commands.sequence(
                    SwerveSubsystem.getInstance().setOdometryCommand(BenPath.getPreviewStartingHolonomicPose()),
                    AutoBuilder.followPath(BenPath)
                );
            case 8:
                return Commands.sequence(
                    SwerveSubsystem.getInstance().setOdometryCommand(SpeakerPath.getPreviewStartingHolonomicPose()),
                    AutoBuilder.followPath(SpeakerPath)
                );
            case 9:
                return OneNoteTest;
            case 10:
                return twoNoteAuto;
        }
    }
}