package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.spikes2212.command.DashboardedSubsystem;
import com.spikes2212.dashboard.RootNamespace;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.commands.Shoot;

public class Shooter extends SubsystemBase {

    public static RootNamespace namespace = new RootNamespace("shooter");

    public static final double GEAR_RATIO = 0.1944;

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

    public RelativeEncoder getEncoder() {

        return this.encoder;
    }

    @Override
    public void periodic() {
        namespace.update();
    }

    public void setSpeed(double speed) {
        motor.set(speed);
    }

    public void stop() {
        motor.stopMotor();
    }

    public void resetEncoders() {
        encoder.setPosition(0);
    }

    public void configureDashboard() {
        namespace.putData("pew pew", new Shoot(Shooter.getInstance()));
        namespace.putNumber("rotations", encoder::getPosition);
        namespace.putNumber("velocity", encoder::getVelocity);
    }
}
