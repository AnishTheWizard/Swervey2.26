package frc.libs.sharkodo.sharkodo;

public class Exceptions {
    public static class PoseTypeMismatch extends Exception {
        public PoseTypeMismatch(String e) {
            super(e);
        }
    }
}
