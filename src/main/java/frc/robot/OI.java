package frc.robot;

import com.revrobotics.CANSparkMax;
import com.spikes2212.util.PlaystationControllerWrapper;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Shoot;
import frc.robot.subsystems.Shooter;

public class OI {

    private static OI instance;
    private Shooter shooter;
    private final Joystick joystick  = new Joystick(0);

    public static OI getInstance() {
        if (instance == null) {
            instance = new OI(Shooter.getInstance());
        }
        return instance;
    }
    private OI(Shooter shooter) {
        this.shooter = shooter;
        JoystickButton trigger = new JoystickButton(joystick, 1);
//        trigger.whileTrue(new Shoot(shooter));
    }
}
