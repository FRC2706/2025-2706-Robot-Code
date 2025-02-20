package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CoralDepositorSubsystem;
import frc.robot.Config;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DigitalInput;


public class CoralDepositorSubsystem extends SubsystemBase {
    private CoralDepositorSubsystem coralDepositor;
    private boolean direction;
    private DigitalInput sensor;
    private Debouncer sensorDebouncer;
    private BooleanPublisher sensorPub;
    private boolean sensorResult;
    private StringPublisher statesPub;

    private static CoralDepositorSubsystem instance;
    public static CoralDepositorSubsystem getInstance() {
        if (instance == null)
            instance = new CoralDepositorSubsystem();
        return instance;
    } 

    // Left and right motor controllers
    private SparkMax leftMotor;
    //private SparkMax rightMotor;
    private SparkMax rightMotor;

    /** Creates a new CoralDepositorSubsystem. */
    public CoralDepositorSubsystem() {

        
        DigitalInput sensor = new DigitalInput(Config.Intake.Sensor);
        sensorDebouncer = new Debouncer(0.1, Debouncer.DebounceType.kBoth);
        
        // Initialize left and right motors
        leftMotor = new SparkMax(Config.CANID.CoralDepositor_LEFT_MOTOR, MotorType.kBrushless);
        rightMotor = new SparkMax(Config.CANID.CoralDepositor_RIGHT_MOTOR, MotorType.kBrushless);
    /*     leftMotorConfig.inverted(false);
        leftMotorConfig.idleMode(IdleMode.kBrake);
        leftMotorConfig.smartCurrentLimit(40);
        leftMotor.configure(leftMotorConfig, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
        
        
         SparkMaxConfig rightMotorConfig = new SparkMaxConfig();
        rightMotorConfig.inverted(true);
        rightMotorConfig.idleMode(IdleMode.kBrake);
        rightMotorConfig.smartCurrentLimit(40);
        rightMotorConfig.voltageCompensation(10); */

        System.out.println("[Init]Creating Coral Depositor");
        leftMotor = new SparkMax(Config.CANID.CoralDepositor_LEFT_MOTOR, MotorType.kBrushless);
        SparkMaxConfig leftMotorConfig = (SparkMaxConfig) new SparkMaxConfig()
                        .inverted(true)
                        .smartCurrentLimit(70)
                        .idleMode(IdleMode.kBrake)
                        .voltageCompensation(10);
        rightMotor = new SparkMax(Config.CANID.CoralDepositor_RIGHT_MOTOR, MotorType.kBrushless);
        SparkMaxConfig rightMotorConfig = (SparkMaxConfig) new SparkMaxConfig()
                        .inverted(true)
                        .smartCurrentLimit(70)
                        .idleMode(IdleMode.kBrake)
                        .voltageCompensation(10);

        leftMotor.configure(leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightMotor.configure(rightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        sensor = new DigitalInput(Config.Intake.Sensor);

        sensorDebouncer = new Debouncer(0.1, Debouncer.DebounceType.kBoth);
        

        NetworkTable intakeTable = NetworkTableInstance.getDefault().getTable("Intake");
        statesPub = intakeTable.getStringTopic("Depositor's Current State").publish(PubSubOption.periodic(0.02));
        sensorPub = intakeTable.getBooleanTopic("sensor result").publish(PubSubOption.periodic(0.02));
        
        ErrorTrackingSubsystem.getInstance().register(leftMotor);
        ErrorTrackingSubsystem.getInstance().register(rightMotor);
        //ErrorTrackingSubsystem.getInstance().register(rightMotor);

        // Must be the last thing in the constructor
        //burnFlash(); // Broken in 2025
    }

    public void set(double voltage){
        leftMotor.set(voltage);
        rightMotor.set(voltage);
    }

    public void stop(){
        leftMotor.stopMotor(); 
        rightMotor.stopMotor();
    }

    public boolean isSensorActive() {
        return sensorResult;
    }

    @Override
    public void periodic() {

    }
}

    /*@Override
    public void execute() {
        sensorResult = sensorDebouncer.calculate(!sensor.get());

        if (direction) {
            coralDepositor.startManipulatePercent(0.4, 0.4);
        } else {
            coralDepositor.startManipulatePercent(-0.2, -0.2);
        }
    }

    @Override
    public void end(boolean interrupted) {
        coralDepositor.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    public void setVoltage(double voltage) {
        coralDepositor.setVoltage(voltage);
    }

    public void stop() {
        coralDepositor.stop();
    }

    public boolean isBackSensorActive() {
        return sensorResult;
    }
}
*/