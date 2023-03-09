package frc.libs.motionoftheocean;

public class Exceptions {

    public static class MotionOfTheOceanIsNotReady extends Exception {
        public MotionOfTheOceanIsNotReady(String msg) {
            super(msg);
        }
    }

    public static class PathFileDoesNotExist extends Exception {
        public PathFileDoesNotExist(String msg) {
            super(msg);
        }
    }
}
