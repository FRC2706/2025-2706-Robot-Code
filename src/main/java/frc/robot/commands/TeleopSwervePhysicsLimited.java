package frc.robot.commands;

import java.util.concurrent.atomic.AtomicReference;

import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Config;
import frc.robot.Config.Swerve.TeleopSpeeds;
import frc.robot.subsystems.SwerveSubsystem;

public class TeleopSwervePhysicsLimited extends Command {
    // Max teleop speeds across all subclasses of TeleopSwerve
    private static TeleopSpeeds speed = TeleopSpeeds.MAX;

    // Whether the swerve is field relative or robot relative across all subclasses of TeleopSwerve
    private static boolean isFieldRelative = true;

    // The swerve subsystem for easy access
    private SwerveSubsystem swerve;

    // Setpoint generator to limit the speeds by what physics can handle
    private SwerveSetpointGenerator setpointGenerator;
    private AtomicReference<SwerveSetpoint> prevSetpoint;
    private AtomicReference<Double> previousTime;
    
    // Driver controller and axis values
    private CommandXboxController driver;
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /**
     * Creates a new TeleopSwervePhysicsLimited command.
     * Drives the swerve drive with physics limited speeds.
     * 
     * @param driver The driver controller
     */
    public TeleopSwervePhysicsLimited(CommandXboxController driver) {
        this.swerve = SwerveSubsystem.getInstance();
        addRequirements(swerve);
        this.driver = driver;

        try {
            setpointGenerator = new SwerveSetpointGenerator(RobotConfig.fromGUISettings(), Config.Swerve.maxAngularVelocity);
        } catch (Exception e) {
            throw new RuntimeException("Failed to create setpoint generator", e);
        }
        resetSetpointGenerator();
    }

    /**
     * Sets the speeds for all TeleopSwervePhysicsLimited commands.
     * 
     * @param newSpeed The new speeds to be used
     */
    public static void setSpeeds(TeleopSpeeds newSpeed) {
        speed = newSpeed;

    }

    /**
     * Sets whether the swerve is field relative or robot relative for all TeleopSwervePhysicsLimited commands.
     * 
     * @param newIsFieldRelative True for field relative, false for robot relative
     */
    public static void setFieldRelative(boolean newIsFieldRelative) {
        isFieldRelative = newIsFieldRelative;
    }

    /**
     * Calculates the translation value based on the driver's input.
     * Overrides this method to change how the translation value is calculated.
     * 
     * @return The translation value to be used in the range [-1, 1]
     */
    protected double calculateTranslationVal() {
        return MathUtil.applyDeadband(-driver.getRawAxis(translationAxis), Config.Swerve.stickDeadband)
                * speed.translationalSpeed;
    }

    /**
     * Calculates the strafe value based on the driver's input.
     * Overrides this method to change how the strafe value is calculated.
     * 
     * @return The strafe value to be used in the range [-1, 1]
     */
    protected double calculateStrafeVal() {
        return MathUtil.applyDeadband(-driver.getRawAxis(strafeAxis), Config.Swerve.stickDeadband)
                * speed.translationalSpeed;
    }

    /**
     * Calculates the rotation value based on the driver's input.
     * Overrides this method to change how the rotation value is calculated.
     * 
     * @return The rotation value to be used in the range [-1, 1]
     */
    protected double calculateRotationVal() {
        return MathUtil.applyDeadband(-driver.getRawAxis(rotationAxis), Config.Swerve.stickDeadband)
                * speed.angularSpeed;
    }

        /**
     * Resets the setpoint generator with the current time and current swerve state.
     */    
    private void resetSetpointGenerator() {
        previousTime.set(Timer.getFPGATimestamp());
        prevSetpoint = new AtomicReference<>(
            new SwerveSetpoint(swerve.getRobotRelativeSpeeds(),
                    swerve.getStates(),
                    DriveFeedforwards.zeros(4)));
    }

    /**
     * Generates a new setpoint based on the current setpoint, current robot speeds, and the time elapsed.
     * 
     * @param speeds The speeds to achieve in either field re
     * @return The new setpoint to achieve the desired speeds in
     */
    private ChassisSpeeds generateSetpoint(ChassisSpeeds speeds) {
        double newTime = Timer.getFPGATimestamp();
        SwerveSetpoint newSetpoint = setpointGenerator.generateSetpoint(prevSetpoint.get(),
                swerve.getRobotRelativeSpeeds(),
                newTime - previousTime.get());

        prevSetpoint.set(newSetpoint);
        previousTime.set(newTime);

        // NOTE: The new setpoint also has the following information:
        // newSetpint.moduleStates() // module states for each module
        // newSetpoint.feedforwards().linearForces() // linear force for each drive motor

        return newSetpoint.robotRelativeSpeeds();
    }

    @Override
    public void initialize() {
        resetSetpointGenerator();
    }

    @Override
    public void execute() {
        // Get the desired speeds
        ChassisSpeeds speeds = new ChassisSpeeds(
                calculateTranslationVal(), 
                calculateStrafeVal(),
                calculateRotationVal());

        // If field relative, rotate the speeds to be robot relative
        Rotation2d heading = SwerveSubsystem.rotateForAlliance(swerve.getHeading());
        if (TeleopSwervePhysicsLimited.isFieldRelative) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, heading);
        }

        // Run the speeds through the setpoint generator to limit the speeds by what physics can handle
        speeds = generateSetpoint(speeds);

        // Drive with robot relative speeds since field relative was already covered
        swerve.drive(
            speeds,
            false,
            true);
    }
}