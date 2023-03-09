package frc.libs.motionoftheocean;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.function.Supplier;

public class State {

    private final ArrayList<String> subsystemOrder;
    double x;
    double y;
    double theta;

    double v;

    HashMap<String, Double> dynamicStates;
    HashMap<String, Boolean> binaryStates;

    HashMap<String, Supplier<Boolean>> parallelConditions;

    public State(double[] chassisMotion,
                 HashMap<String, Double> dynamicStates,
                 HashMap<String, Boolean> binaryStates,
                 HashMap<String, Supplier<Boolean>> parallelConditions,
                 ArrayList<String> subsystemOrder)
    {
        x = chassisMotion[0];
        y = chassisMotion[1];
        theta = chassisMotion[2];

        v = chassisMotion[3];

        this.dynamicStates = dynamicStates;
        this.binaryStates = binaryStates;

        this.parallelConditions = parallelConditions;

        this.subsystemOrder = subsystemOrder;
    }

    public double[] getPose() {
        return new double[]{x, y, theta, v};
    }

    public HashMap<String, Double> getDynamicStates() {
        return dynamicStates;
    }

    public HashMap<String, Boolean> getBinaryStates() {
        return binaryStates;
    }

    @Override
    public String toString(){
        double[] pose = getPose();
        String motionVec = "";

        for(double dimension : pose) {
            motionVec += dimension + ",";
        }

        StringBuilder line = new StringBuilder(motionVec);

        for(String s : subsystemOrder) {
            if(s.charAt(0) == '~')
                line.append(dynamicStates.get(s)).append(",");
            else
                line.append(binaryStates.get(s)).append(",");
        }

        System.out.println("line = " + line);
        for(String k : parallelConditions.keySet()) {
            line.append(k).append(",");
        }

        return line.toString() + "\n";
    }


}
