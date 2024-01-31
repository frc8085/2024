package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;

public class VisionSubsystem {
    public double getAprilTagID() {
        // The computer fought with us :( thats why it looks so weird
        double[] id = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid")
                .getDoubleArray(new double[6]);
        return id[0];
    };

    public double[] getAprilTagLocation() {
        double[] location = NetworkTableInstance.getDefault().getTable("limelight").getEntry("targetpose_cameraspace")
                .getDoubleArray(new double[6]);
        return location;
    }
}
