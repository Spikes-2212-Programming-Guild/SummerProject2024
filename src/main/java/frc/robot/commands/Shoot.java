package frc.robot.commands;

import com.spikes2212.dashboard.RootNamespace;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

import java.util.function.Supplier;

public class Shoot extends CommandBase {

    public static RootNamespace namespace = new RootNamespace("shoot");
    private static final Supplier<Double> kP = namespace.addConstantDouble("kP", 0.05);
    private static final Supplier<Double> kI = namespace.addConstantDouble("kI", 0);
    private static final Supplier<Double> kD = namespace.addConstantDouble("kD", 0);
    private final PIDController pidController;

    private Shooter shooter;
    private static Supplier<Double> rotationsPerSecond = namespace.addConstantDouble("rotations per second", 0);

    public Shoot(Shooter shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
        pidController = new PIDController(kP.get(), kI.get(), kD.get());
    }

    public static void periodic() {
        namespace.update();
    }

    @Override
    public void execute() {
        pidController.setPID(kP.get(), kI.get(), kD.get());
        shooter.setSpeed(shooter.getMotor().get() + (pidController.calculate(shooter.getEncoder().getVelocity(),
                rotationsPerSecond.get())) * 0.02);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
    }
}
