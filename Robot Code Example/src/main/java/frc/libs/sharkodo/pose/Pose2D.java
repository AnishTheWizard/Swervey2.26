package frc.libs.sharkodo.pose;

import frc.libs.sharkodo.sharkodo.Exceptions;
import java.util.Objects;

public final class Pose2D implements Pose {
    private final double x;
    private final double y;

    Pose2D(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public double x() {
        return x;
    }

    public double y() {
        return y;
    }

    @Override
    public boolean equals(Object obj) {
        if (obj == this) return true;
        if (obj == null || obj.getClass() != this.getClass()) return false;
        var that = (Pose2D) obj;
        return Double.doubleToLongBits(this.x) == Double.doubleToLongBits(that.x) &&
                Double.doubleToLongBits(this.y) == Double.doubleToLongBits(that.y);
    }

    @Override
    public int hashCode() {
        return Objects.hash(x, y);
    }

    @Override
    public String toString() {
        return "Pose2D[" +
                "x=" + x + ", " +
                "y=" + y + ']';
    }

    @Override
    public double[] getPose() {
        return new double[0];
    }

    @Override
    public Pose getTransformation(Pose p) throws Exceptions.PoseTypeMismatch {
        return null;
    }

    @Override
    public void add(Pose p) throws Exceptions.PoseTypeMismatch {

    }
}
