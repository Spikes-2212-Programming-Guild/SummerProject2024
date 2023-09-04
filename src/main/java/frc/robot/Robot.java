// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.Shoot;
import frc.robot.subsystems.Piston;
import frc.robot.subsystems.Shooter;

public class Robot extends TimedRobot {

    private Shooter shooter;
    private Piston piston;

    @Override
    public void robotInit() {
        shooter = Shooter.getInstance();
        shooter.resetEncoders();
        shooter.configureDashboard();
        piston = Piston.getInstance();
        piston.configureDashboard();
    }

    @Override
    public void robotPeriodic() {
        shooter.periodic();
        Shoot.periodic();
        piston.periodic();
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void autonomousInit() {
        new Shoot(shooter).schedule();
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
    }

    @Override
    public void teleopPeriodic() {

    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void simulationInit() {
    }

    @Override
    public void simulationPeriodic() {
    }
}
