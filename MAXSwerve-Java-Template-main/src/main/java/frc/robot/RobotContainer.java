// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Intake;
import frc.robot.commands.IntakeStop;
import frc.robot.commands.MoveToAmp;
import frc.robot.commands.MoveToPodium;
import frc.robot.commands.MoveToPosition;
import frc.robot.commands.MoveToSubwooferLow;
import frc.robot.commands.MoveToTrapApproach;
import frc.robot.commands.MoveToTrapClimb;
import frc.robot.commands.MoveToTrapScore;
import frc.robot.commands.MoveToTravel;

import java.util.List;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems
    private final DriveSubsystem m_drive = new DriveSubsystem();
    private final ShooterSubsystem m_shooter = new ShooterSubsystem();
    private final IntakeSubsystem m_intake = new IntakeSubsystem();
    private final ArmSubsystem m_arm = new ArmSubsystem();

    // The driver's controller
    CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
    // The operator's controller
    CommandXboxController m_operatorController = new CommandXboxController(OIConstants.kOperatorControllerPort);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();

        // Configure default commands
        m_drive.setDefaultCommand(
                // The left stick controls translation of the robot.
                // Turning is controlled by the X axis of the right stick.
                new RunCommand(() -> m_drive.drive(-MathUtil.applyDeadband(m_driverController.getLeftY(),
                        OIConstants.kDriveDeadband),
                        -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                        -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                        true,
                        true),
                        m_drive));
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     */
    private void configureButtonBindings() {
        // DRIVER controlled buttons
        final Trigger shoot = m_driverController.a();

        // OPERATOR controlled buttons
        final Trigger intake = m_operatorController.leftTrigger();
        final Trigger alternatePosition = m_operatorController.leftBumper();

        final Trigger moveToTravel = m_operatorController.y();
        final Trigger moveToSubwoofer = m_operatorController.b();
        final Trigger moveToAmp = m_operatorController.a();
        final Trigger moveToPodium = m_operatorController.x();
        final Trigger moveToTrapApproach = m_operatorController.povUp();
        final Trigger moveToTrapScore = m_operatorController.povRight();
        final Trigger moveToTrapClimb = m_operatorController.povDown();

        intake.whileTrue(new InstantCommand(m_intake::forward))
                .whileFalse(new InstantCommand(m_intake::stop));

        moveToTravel.whileTrue(new MoveToTravel(m_arm));

        moveToSubwoofer.whileTrue(new MoveToSubwooferLow(m_arm));

        moveToAmp.whileTrue(new MoveToAmp(m_arm));

        moveToPodium.whileTrue(new MoveToPodium(m_arm));

        // TODO: Test the "ENUM approach" by pressing X here. 
        moveToPodium.whileTrue(new MoveToPosition(m_arm, ArmConstants.Position.PODIUM));

        moveToTrapApproach.whileTrue(new MoveToTrapApproach(m_arm));

        moveToTrapScore.whileTrue(new MoveToTrapScore(m_arm));

        moveToTrapClimb.whileTrue(new MoveToTrapClimb(m_arm));

        /**
         * Alternate positions.
         * For these, you need to hold down the Left Bumper too.
         **/

        // Move to HIGH Podium
        moveToPodium.and(alternatePosition).whileTrue(
                new InstantCommand(() -> m_arm.setArmPositionDegrees(ArmConstants.kHighPodiumArm)));

        // Move to BACK Podium
        moveToAmp.and(alternatePosition).whileTrue(
                new InstantCommand(() -> m_arm.setArmPositionDegrees(ArmConstants.kBackPodiumArm)));

        // Move to HIGH Subwoofer
        moveToSubwoofer.and(alternatePosition).whileTrue(
                new InstantCommand(() -> m_arm.setArmPositionDegrees(ArmConstants.kHighSubwooferArm)));

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // Create config for trajectory
        TrajectoryConfig config = new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(DriveConstants.kDriveKinematics);

        // An example trajectory to follow. All units in meters.
        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(3, 0, new Rotation2d(0)),
                config);

        var thetaController = new ProfiledPIDController(
                AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                exampleTrajectory,
                m_drive::getPose, // Functional interface to feed supplier
                DriveConstants.kDriveKinematics,

                // Position controllers
                new PIDController(AutoConstants.kPXController, 0, 0),
                new PIDController(AutoConstants.kPYController, 0, 0),
                thetaController,
                m_drive::setModuleStates,
                m_drive);

        // Reset odometry to the starting pose of the trajectory.
        m_drive.resetOdometry(exampleTrajectory.getInitialPose());

        // Run path following command, then stop at the end.
        return swerveControllerCommand.andThen(() -> m_drive.drive(0, 0, 0, false, false));
    }
}
