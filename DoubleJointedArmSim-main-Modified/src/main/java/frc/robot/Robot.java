
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

/**
 * This is a sample program to demonstrate the use of arm simulation with
 * existing code.
 */
public class Robot extends TimedRobot {
  private static final int kMotorPort = 0;
  private static final int kEncoderAChannel = 0;
  private static final int kEncoderBChannel = 1;
  private static final int kJoystickPort = 0;

  // The P gain for the PID controller that drives this arm.
  private static final double kArmKp = 40.0;
  private static final double kArmKi = 0.0;

  // distance per pulse = (angle per revolution) / (pulses per revolution)
  // = (2 * PI rads) / (4096 pulses)
  private static final double kArmEncoderDistPerPulse = 2.0 * Math.PI / 4096;

  // The arm gearbox represents a gearbox containing 1 NEO Vortex motors.
  private final DCMotor m_armGearbox = DCMotor.getNeoVortex(1);

  // Standard classes for controlling our arm
  private final ProfiledPIDController m_shooterController = new ProfiledPIDController(kArmKp, kArmKi, 0,
      new TrapezoidProfile.Constraints(2, 5));
  private final ProfiledPIDController m_armController = new ProfiledPIDController(kArmKp, kArmKi, 0,
      new TrapezoidProfile.Constraints(2, 5));
  private final Encoder m_shooterEncoder = new Encoder(kEncoderAChannel, kEncoderBChannel);
  private final Encoder m_armEncoder = new Encoder(kEncoderAChannel + 2, kEncoderBChannel + 2);

  private final PWMSparkMax m_shooterMotor = new PWMSparkMax(kMotorPort);
  private final PWMSparkMax m_armMotor = new PWMSparkMax(kMotorPort + 1);
  private final Joystick m_joystick = new Joystick(kJoystickPort);

  // Simulation classes help us simulate what's going on, including gravity.
  private static final double m_armReduction = 600;
  private static final double m_arm_shooterMass = Units.lbsToKilograms(10); // Kilograms
  private static final double m_arm_shooterLengthMeters = Units.inchesToMeters(10);
  private static final double m_arm_armMass = Units.lbsToKilograms(5); // Kilograms
  private static final double m_arm_armLengthMeters = Units.inchesToMeters(19.5);

  private static final int m_arm_shooter_min_angle = -90;
  private static final int m_arm_shooter_max_angle = 180;
  private static final int m_arm_arm_min_angle = -55;
  private static final int m_arm_arm_max_angle = 43;

  // SETPOINTS FOR PRESETS MODE

  private static final int adjustmentFactor = 90;

  private static final int travelArm = 35 - adjustmentFactor;
  private static final int travelShooter = 83;

  // podium
  private static final int podiumScoreArm = 46 - adjustmentFactor;
  private static final int podiumScoreShooter = 79;

  // subwoofer
  private static final int subwooferScoreArm = 127 - adjustmentFactor;
  private static final int subwooferScoreShooter = 23;

  // amp
  private static final int ampScoreArm = 114 - adjustmentFactor;
  private static final int ampScoreShooter = -39;

  // trap approach
  private static final int trapApproachArm = 132 - adjustmentFactor;
  private static final int trapApproachShooter = -42;

  // trap climb
  private static final int trapClimbArm = 130 - adjustmentFactor;
  private static final int trapClimbShooter = -40;

  // trap score
  private static final int trapScoreArm = 130 - adjustmentFactor;
  private static final int trapScoreShooter = -55;

  // high podium
  private static final int highPodiumArm = 127 - adjustmentFactor;
  private static final int highPodiumShooter = -40;

  // back podium
  private static final int backPodiumArm = 127 - adjustmentFactor;
  private static final int backPodiumShooter = 166;

  // This shooter sim represents an arm that can travel
  private final SingleJointedArmSim m_arm_shooterSim = new SingleJointedArmSim(
      m_armGearbox,
      m_armReduction,
      SingleJointedArmSim.estimateMOI(m_arm_shooterLengthMeters, m_arm_shooterMass),
      m_arm_shooterLengthMeters,
      Units.degreesToRadians(m_arm_shooter_min_angle),
      Units.degreesToRadians(m_arm_shooter_max_angle),
      false,
      m_arm_shooterMass,
      VecBuilder.fill(kArmEncoderDistPerPulse) // Add noise with a std-dev of 1 tick
  );
  private final SingleJointedArmSim m_arm_armSim = new SingleJointedArmSim(
      m_armGearbox,
      m_armReduction,
      SingleJointedArmSim.estimateMOI(m_arm_armLengthMeters, m_arm_armMass),
      m_arm_armLengthMeters,
      Units.degreesToRadians(m_arm_arm_min_angle),
      Units.degreesToRadians(m_arm_arm_max_angle),
      true,
      m_arm_armMass,
      VecBuilder.fill(kArmEncoderDistPerPulse) // Add noise with a std-dev of 1 tick
  );
  private final EncoderSim m_shooterEncoderSim = new EncoderSim(m_shooterEncoder);
  private final EncoderSim m_armEncoderSim = new EncoderSim(m_armEncoder);
  SendableChooser<Integer> controlMode = new SendableChooser<Integer>();
  SendableChooser<Integer> presetChooser = new SendableChooser<Integer>();

  // Create a Mechanism2d display of an Arm with a fixed ArmTower and moving Arm.

  private final Mechanism2d m_mech2d = new Mechanism2d(90, 90);
  private final MechanismRoot2d gridHome = m_mech2d.getRoot("Grid Home", 25,
      0);
  // private final MechanismRoot2d dsHome = m_mech2d.getRoot("Double Substation
  // Home", 49.75, 37);
  private final MechanismRoot2d m_armPivot = m_mech2d.getRoot("ArmPivot", 32.25,
      21.75); // Changes position of the base of the arm
  private final MechanismLigament2d m_arm_arm = m_armPivot.append(
      new MechanismLigament2d(
          "Arm arm",
          Units.metersToInches(m_arm_armLengthMeters),
          -90,
          10,
          new Color8Bit(Color.kGold)));
  private final MechanismLigament2d m_arm_tower = m_armPivot
      .append(new MechanismLigament2d("ArmTower", 18, -90, 10, new Color8Bit(Color.kSilver)));

  private final MechanismLigament2d m_aframe_1 = m_armPivot
      .append(new MechanismLigament2d("aframe1", 24, -70, 10, new Color8Bit(Color.kSilver)));
  private final MechanismLigament2d m_bumper = gridHome
      .append(new MechanismLigament2d("Bumper", 33, 0, 60, new Color8Bit(Color.kRed)));
  private final MechanismLigament2d m_arm_shooter = m_arm_arm.append(
      new MechanismLigament2d(
          "Half Shooter",
          Units.metersToInches(m_arm_shooterLengthMeters),
          Units.radiansToDegrees(m_arm_shooterSim.getAngleRads()),
          10,
          new Color8Bit(Color.kPurple)));
  private final MechanismLigament2d m_shooter = m_arm_shooter.append(
      new MechanismLigament2d(
          "Shooter",
          -20,
          Units.radiansToDegrees(m_arm_shooterSim.getAngleRads()),
          10,
          new Color8Bit(Color.kPurple)));
  private final MechanismLigament2d m_shooterFront = m_arm_shooter.append(
      new MechanismLigament2d(
          "Front",
          2,
          Units.radiansToDegrees(m_arm_shooterSim.getAngleRads()),
          10,
          new Color8Bit(Color.kWhite)));

  @Override
  public void robotInit() {
    m_shooterEncoder.setDistancePerPulse(kArmEncoderDistPerPulse);
    m_armEncoder.setDistancePerPulse(kArmEncoderDistPerPulse);
    SmartDashboard.putNumber("Setpoint shooter (degrees)", 90);
    SmartDashboard.putNumber("Setpoint arm (degrees)", 90);
    controlMode.setDefaultOption("Presets (Setpoints)", 0);
    controlMode.addOption("Manual Angle Adjust", 1);

    presetChooser.setDefaultOption("Starting Config", 0);
    presetChooser.addOption("Podium", 1);
    presetChooser.addOption("Subwoofer", 2);
    presetChooser.addOption("Amp", 3);
    presetChooser.addOption("Trap Approach", 4);
    presetChooser.addOption("Trap Climb", 5);
    presetChooser.addOption("Trap Score", 6);
    presetChooser.addOption("High Podium", 7);
    presetChooser.addOption("Back Podium", 8);

    SmartDashboard.putData(controlMode);
    SmartDashboard.putData(presetChooser);
    // Put Mechanism 2d to SmartDashboard
    SmartDashboard.putData("Arm Sim", m_mech2d);

  }

  @Override
  public void simulationPeriodic() {
    // In this method, we update our simulation of what our arm is doing
    // First, we set our "inputs" (voltages)
    m_arm_shooterSim.setInput(m_shooterMotor.get() * RobotController.getBatteryVoltage());
    m_arm_armSim.setInput(m_armMotor.get() * RobotController.getBatteryVoltage());

    // Next, we update it. The standard loop time is 20ms.
    m_arm_shooterSim.update(0.020);
    m_arm_armSim.update(0.020);

    // Finally, we set our simulated encoder's readings and simulated battery
    // voltage
    m_shooterEncoderSim.setDistance(m_arm_shooterSim.getAngleRads());
    m_armEncoderSim.setDistance(m_arm_armSim.getAngleRads());
    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(
            m_arm_shooterSim.getCurrentDrawAmps() + m_arm_armSim.getCurrentDrawAmps()));

    // Update the Mechanism Arm angle based on the simulated arm angle
    m_arm_shooter.setAngle(Units.radiansToDegrees(m_arm_shooterSim.getAngleRads()));
    m_arm_arm.setAngle(Units.radiansToDegrees(m_arm_armSim.getAngleRads()));
  }

  @Override
  public void teleopPeriodic() {

    switch (controlMode.getSelected()) {
      case 1:
        double pidOutputShooter = m_shooterController
            .calculate(m_shooterEncoder.getDistance(),
                Units.degreesToRadians(MathUtil.clamp(
                    SmartDashboard.getNumber("Setpoint shooter (degrees)", 0),
                    m_arm_shooter_min_angle, m_arm_shooter_max_angle)));
        m_shooterMotor.setVoltage(pidOutputShooter);

        double pidOutputArm = m_armController.calculate(m_armEncoder.getDistance(),
            Units.degreesToRadians(MathUtil.clamp(SmartDashboard.getNumber("Setpoint arm (degrees)", 0),
                m_arm_arm_min_angle, m_arm_arm_max_angle)));
        m_armMotor.setVoltage(pidOutputArm);
        break;

      default: // also case 0
        int shooterSetpoint, armSetpoint;
        switch (presetChooser.getSelected()) {
          case 0:
            shooterSetpoint = travelShooter;
            armSetpoint = travelArm;
            break;
          case 1:
            shooterSetpoint = podiumScoreShooter;
            armSetpoint = podiumScoreArm;
            break;
          case 2:
            shooterSetpoint = subwooferScoreShooter;
            armSetpoint = subwooferScoreArm;
            break;
          case 3:
            shooterSetpoint = ampScoreShooter;
            armSetpoint = ampScoreArm;
            break;
          case 4:
            shooterSetpoint = trapApproachShooter;
            armSetpoint = trapApproachArm;
            break;
          case 5:
            shooterSetpoint = trapClimbShooter;
            armSetpoint = trapClimbArm;
            break;
          case 6:
            shooterSetpoint = trapScoreShooter;
            armSetpoint = trapScoreArm;
            break;
          case 7:
            shooterSetpoint = highPodiumShooter;
            armSetpoint = highPodiumArm;
            break;
          case 8:
            shooterSetpoint = backPodiumShooter;
            armSetpoint = backPodiumArm;
            break;
          default:
            shooterSetpoint = travelShooter;
            armSetpoint = travelArm;
            break;
        }
        // Here, we run PID control where the arm moves to the selected setpoint.
        pidOutputShooter = m_shooterController
            .calculate(m_shooterEncoder.getDistance(),
                Units.degreesToRadians(shooterSetpoint));

        m_shooterMotor.setVoltage(pidOutputShooter);
        SmartDashboard.putNumber("Setpoint arm (degrees)", armSetpoint);
        SmartDashboard.putNumber("Setpoint shooter (degrees)", shooterSetpoint);
        pidOutputArm = m_armController.calculate(m_armEncoder.getDistance(),
            Units.degreesToRadians(armSetpoint));
        m_armMotor.setVoltage(pidOutputArm);
        break;
    }

    // if (m_joystick.getTrigger()) {
    // // Here, we run PID control like normal.
    // m_arm_shooter.setAngle(Units.radiansToDegrees(m_arm_shooterSim.getAngleRads()));
    // m_arm_arm.setAngle(Units.radiansToDegrees(m_arm_armSim.getAngleRads()));
    // } else {
    // // Otherwise, we disable the motor.

    // }

  }

  @Override
  public void disabledInit() {
    // This just makes sure that our simulation code knows that the motor's off.
    m_shooterMotor.set(0.0);
  }
}
