package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.RobotMap;

public class Piston extends DoubleSolenoidSubsystem {

    private static Piston instance;

    public static Piston getInstance() {
        if (instance == null) {
            instance = new Piston("piston", new DoubleSolenoid(PneumaticsModuleType.CTREPCM,
                    RobotMap.PCM.SOLENOID_FORWARD, RobotMap.PCM.SOLENOID_BACKWARD));
        }
        return instance;
    }

    private Piston(String namespaceName, DoubleSolenoid doubleSolenoid) {
        super(namespaceName, doubleSolenoid, false);
    }

    @Override
    public void configureDashboard() {
        namespace.putData("open", openSolenoid());
        namespace.putData("close", closeSolenoid());
    }
}
