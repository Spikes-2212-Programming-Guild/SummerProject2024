package frc.robot.commands;

import com.spikes2212.control.FeedForwardController;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.dashboard.RootNamespace;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

import java.util.function.Supplier;

public class Shoot extends CommandBase {

    public static final RootNamespace namespace = new RootNamespace("shoot");
    private static final Supplier<Double> kP = namespace.addConstantDouble("kP", 0.05);
    private static final Supplier<Double> kI = namespace.addConstantDouble("kI", 0);
    private static final Supplier<Double> kD = namespace.addConstantDouble("kD", 0);
    private final PIDController pidController;

    private static final Supplier<Double> kS = namespace.addConstantDouble("kS", 0);
    private static final Supplier<Double> kV = namespace.addConstantDouble("kV", 0);
    private final FeedForwardController feedForward;

    private static final Supplier<Double> WAIT_TIME = namespace.addConstantDouble("wait time", 3);
    private static final Supplier<Double> TOLERANCE = namespace.addConstantDouble("tolerance", 0.5);
    private double lastTimeNotOnTarget;

    private final Shooter shooter;
    private static final Supplier<Double> rotationsPerSecond = namespace.addConstantDouble(
            "rotations per second", 0);

    public Shoot(Shooter shooter) {
        this.lastTimeNotOnTarget = 0;
        this.shooter = shooter;
        addRequirements(shooter);
        pidController = new PIDController(kP.get(), kI.get(), kD.get());
        feedForward = new FeedForwardController(kS.get(), kV.get(), 0, FeedForwardController.DEFAULT_PERIOD);
    }

    public static void periodic() {
        namespace.update();
    }

    @Override
    public void execute() {
        pidController.setTolerance(TOLERANCE.get());
        pidController.setPID(kP.get(), kI.get(), kD.get());
        feedForward.setGains(kS.get(), kV.get(), 0);
        shooter.setSpeed((pidController.calculate(shooter.getVelocity(),
                rotationsPerSecond.get())) + feedForward.calculate(rotationsPerSecond.get()));
    }

    @Override
    public boolean isFinished() {
        if (!pidController.atSetpoint()) {
            lastTimeNotOnTarget = Timer.getFPGATimestamp();
        }
        return Timer.getFPGATimestamp() - lastTimeNotOnTarget >= WAIT_TIME.get();
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
    }
}
