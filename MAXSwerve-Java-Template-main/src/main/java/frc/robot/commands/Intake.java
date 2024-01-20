package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.IntakeSubsystem;

public class Intake extends SequentialCommandGroup {
    public Intake(
            IntakeSubsystem m_Intake) {

        addCommands(
                new ConditionalCommand(
                        new InstantCommand(m_Intake::stop),
                        new InstantCommand(m_Intake::forward),
                        m_Intake::NoteDetected));
    }
}
