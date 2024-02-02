package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.ArmSubsystem;

import static frc.robot.Constants.ArmConstants.Position;

public class MoveToPosition extends SequentialCommandGroup {
    public MoveToPosition(
            ArmSubsystem m_arm,
            Position position) {
        addCommands(
                new InstantCommand(() -> System.out.println("**START Move to " + position.label + " Position**")),
                new ParallelCommandGroup(
                        new InstantCommand(() -> m_arm.setShooterArmPositionDegrees(position.armPosition)),
                        new InstantCommand(() -> m_arm.setShooterArmPositionDegrees(position.shooterArmPosition))
            ));
    }

}
