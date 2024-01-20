package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.ShooterSubsystem;

public class Shoot extends SequentialCommandGroup {
    public Shoot(
            ShooterSubsystem m_Shoot,
            int motor) {

        addCommands(new InstantCommand(() -> m_Shoot.forward(motor)));
    }
}
