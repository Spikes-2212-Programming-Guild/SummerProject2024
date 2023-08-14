package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.spikes2212.dashboard.RootNamespace;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.commands.Shoot;

public class Shooter extends SubsystemBase {

    public static RootNamespace namespace = new RootNamespace("shooter");

    public static final double GEAR_RATIO = 1/2.21;

    private final CANSparkMax motor;
    private final RelativeEncoder encoder;

    private static Shooter instance;

    public static Shooter getInstance() {
        if (instance == null) {
            instance = new Shooter(new CANSparkMax(RobotMap.CAN.SHOOTER_SPARK_MAX,
                    CANSparkMaxLowLevel.MotorType.kBrushless));
        }
        return instance;
    }

    private Shooter(CANSparkMax motor) {
        this.motor = motor;
        this.encoder = motor.getEncoder();
        encoder.setPositionConversionFactor(GEAR_RATIO);
        encoder.setVelocityConversionFactor(GEAR_RATIO/60);
    }

    @Override
    public void periodic() {
        namespace.update();

    }

    //Speed is in units of seconds
    public void setSpeed(double speed) {
        motor.set(speed);
    }


    public void stop() {
        motor.stopMotor();
    }

    public void resetEncoders() {
        encoder.setPosition(0);
    }

    public double getVelocity(){
        return encoder.getVelocity();
    }

    public void configureDashboard() {
        namespace.putData("pew pew", new Shoot(this));
        namespace.putNumber("rotations", encoder::getPosition);
        namespace.putNumber("velocity", encoder::getVelocity);
    }
}
