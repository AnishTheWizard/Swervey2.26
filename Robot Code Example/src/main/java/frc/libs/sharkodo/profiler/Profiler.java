package frc.libs.sharkodo.profiler;

import frc.libs.sharkodo.sharkodo.Exceptions;
import frc.libs.sharkodo.pose.Pose;

public interface Profiler {
    public Pose poll() throws Exceptions.PoseTypeMismatch;
    public void reset();
    public void reset(double[] p);
}
