package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.IntakeSubsystem;
public class IntakeStop extends SequentialCommandGroup{
    public IntakeStop(
        IntakeSubsystem m_Intake
    ){
    addCommands(
        new InstantCommand(() -> m_Intake.stop()));
    }
}
