package frc.libs.sharkodo.pose;

import frc.libs.sharkodo.sharkodo.Exceptions;

public class Twist2D implements Pose {

    private double x, y, theta, v;

    public Twist2D(double x, double y, double theta, double v) {
        this.x = x;
        this.y = y;
        this.theta = theta;
        this.v = v;
    }

    public Twist2D() {
        x = 0;
        y = 0;
        theta = 0;
        v = 0;
    }

    @Override
    public double[] getPose() {
        return new double[]{x, y, theta, v};
    }

    @Override
    public Pose getTransformation(Pose p) throws Exceptions.PoseTypeMismatch {
        Twist2D vector;
        if(p instanceof Twist2D) {
            vector = (Twist2D) p;
        }
        else {
            throw new Exceptions.PoseTypeMismatch("Expected to get pose type Twist2D");
        }
        return new Twist2D(vector.getX() - getX(),
                vector.getY() - getY(),
                vector.getTheta() - getTheta(),
                vector.getV() - getV());
    }

    @Override
    public void add(Pose p) throws Exceptions.PoseTypeMismatch {
        if(p instanceof Twist2D) {
            Twist2D addend = (Twist2D) p;
            x+=addend.getX();
            y+=addend.getY();
            theta+=addend.getTheta();
        }
        else {
            throw new Exceptions.PoseTypeMismatch("Expected to get pose type Twist2D");
        }

    }

    public Twist2D createFactoredTwist(double factor) {
        return new Twist2D(x*factor, y*factor, theta, v);
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getTheta() {
        return theta;
    }

    public double getV() {return v;}

    public void setX(double x) {
        this.x = x;
    }

    public void setY(double y) {
        this.y = y;
    }

    public void setTheta(double theta) {
        this.theta = theta;
    }

    public void setV(double v) {this.v = v;}

    public void setTwist(double[] pose) {
        this.x = pose[0];
        this.y = pose[1];
        this.theta = pose[2];
        this.v = pose[3];
    }

    public void setTwist(double x, double y, double theta, double v) {
        this.x = x;
        this.y = y;
        this.theta = theta;
        this.v = v;
    }
}
