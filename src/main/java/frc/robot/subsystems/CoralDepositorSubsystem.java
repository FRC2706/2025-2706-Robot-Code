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
import frc.lib.lib2706.SubsystemChecker;
import frc.lib.lib2706.SubsystemChecker.SubsystemType;
import frc.robot.Config;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DigitalInput;


public class CoralDepositorSubsystem extends SubsystemBase {
    private DigitalInput sensor;
    private Debouncer sensorDebouncer;
    private BooleanPublisher sensorPub;
    private boolean sensorResult;

    private static CoralDepositorSubsystem instance;
    public static CoralDepositorSubsystem getInstance() {
        if (instance == null) {
            SubsystemChecker.subsystemConstructed(SubsystemType.CoralDepositorSubsystem);
            instance = new CoralDepositorSubsystem();
        }
        return instance;
    } 

    // Left and right motor controllers
    private SparkMax leftMotor;
    //private SparkMax rightMotor;
    private SparkMax rightMotor;

    /** Creates a new CoralDepositorSubsystem. */
    public CoralDepositorSubsystem() {
        
        // Initialize the sensor
        //@todo: update sensor
        sensor = new DigitalInput(0);
        sensorDebouncer = new Debouncer(0.1, Debouncer.DebounceType.kBoth);
        
        // Initialize left and right motors
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

        NetworkTable intakeTable = NetworkTableInstance.getDefault().getTable("Coral Intake");
        sensorPub = intakeTable.getBooleanTopic("Coral sensor result").publish(PubSubOption.periodic(0.02));
        
        ErrorTrackingSubsystem.getInstance().register(leftMotor);
        ErrorTrackingSubsystem.getInstance().register(rightMotor);

    }

    public void setVoltage(double voltage){
        leftMotor.set(voltage);
        rightMotor.set(-voltage);
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

        //update the sensor status
        sensorResult = sensorDebouncer.calculate(!sensor.get());
        sensorPub.accept(sensorResult);

    }
}