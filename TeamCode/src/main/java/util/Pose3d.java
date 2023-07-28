package util;

public class Pose3d {
    public double x;
    public double y;
    public double z;
    public double rotation;
    public double yaw;

    public Pose3d(double x, double y, double z, double rotation, double yaw) {
        this.x = x;
        this.y = y;
        this.z = z;
        this.rotation = rotation;
        this.yaw = yaw;
    }
}
