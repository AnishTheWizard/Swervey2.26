package frc.libs.sharkodo.pose;

import frc.libs.sharkodo.sharkodo.Exceptions;

public interface Pose {

    public double[] getPose();

    public Pose getTransformation(Pose p) throws Exceptions.PoseTypeMismatch;

    public void add(Pose p) throws Exceptions.PoseTypeMismatch;

    public String toString();

}
