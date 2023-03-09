package frc.libs.motionoftheocean;


import java.io.FileNotFoundException;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class MotionOfTheOcean {

    //Subsystem commands
    private static final HashMap<String, Supplier<Double>> dynamicStateAccessors = new HashMap<>();
    private static final HashMap<String, Supplier<Boolean>> binaryStateAccessors = new HashMap<>();

    private static final HashMap<String, Consumer<Double>> dynamicExecutors = new HashMap<>();
    private static final HashMap<String, Consumer<Boolean>> binaryExecutors = new HashMap<>();

    private static final HashMap<String, Supplier<Boolean>> parallelConditions = new HashMap<>();

    private static ArrayList<String> subsystemOrder = new ArrayList<String>();

    //drivetrain commands
    private static Supplier<double[]> getChassisState = null;
    private static Consumer<double[]> toChassisState = null;

    //config params
    private static String selectedPath = "recording.csv";

    public static void addPositionFunctions(Supplier<double[]> getPose, Consumer<double[]> toPose) {
        MotionOfTheOcean.getChassisState = getPose;
        MotionOfTheOcean.toChassisState = toPose;
    }

    public static void addDynamicState(String name, Supplier<Double> state, Consumer<Double> executor) {
        dynamicStateAccessors.put(name, state);
        dynamicExecutors.put(name, executor);

    }

    public static void addBinaryState(String name, Supplier<Boolean> state, Consumer<Boolean> runnable) {
        binaryStateAccessors.put(name, state);
        binaryExecutors.put(name, runnable);
    }

    public static void addParallelCondition(String name, Supplier<Boolean> condition) {
        parallelConditions.put(name, condition);
    }

    public static void addSubsystem(String subsystemName) {
        subsystemOrder.add(subsystemName);
    }

    public static void setSelectedPath(String filename) {
        selectedPath = filename;
    }

    public static boolean isParentNotReady() {
        return (getChassisState == null || toChassisState == null);
    }

    public static class Recorder {


        private static ArrayList<State> recording = new ArrayList<>();

        private static final ScheduledExecutorService executorService = Executors.newSingleThreadScheduledExecutor();

        public enum RecordingType {
            TOTAL_SYSTEM,
            SUBSYSTEM_ONLY
        }

        private static RecordingType recordingType = RecordingType.TOTAL_SYSTEM;

        public static void resetRecorder() {
            stopRecorder();
            recording = new ArrayList<>();
        }

        public static void startRecorder() throws Exceptions.MotionOfTheOceanIsNotReady {
            if(isParentNotReady())
                throw new Exceptions.MotionOfTheOceanIsNotReady("Could not start recorder");
            executorService.scheduleAtFixedRate(MotionOfTheOcean.Recorder::update, 0, 20, TimeUnit.MILLISECONDS);
        }

        public static void stopRecorder() {
            executorService.shutdown();
        }

        public static void exportRecording() {
            try {
                String headerString = "x,y,theta,v,";
                for(String subsystem : subsystemOrder) {
                    headerString += subsystem + ",";
                }

                FileManager.export(selectedPath, recording, headerString);
            } catch (IOException e) {
                throw new RuntimeException(e);
            }
        }

        public static void update() {
            double[] chassisMotion = getChassisState.get();
            HashMap<String, Double> dynamicStates = new HashMap<String, Double>();
            HashMap<String, Boolean> binaryStates = new HashMap<String, Boolean>();

            for(String name : subsystemOrder) {
                char escapeChar = name.charAt(0);
                if(escapeChar == '~')
                    dynamicStates.put(
                            name,
                            dynamicStateAccessors.get(name).get()
                    );
                else if(escapeChar == '-')
                    binaryStates.put(
                            name,
                            binaryStateAccessors.get(name).get()
                    );
                //System.out.println("ima bitch");
            }

            recording.add(
                    new State(
                            chassisMotion,
                            dynamicStates,
                            binaryStates,
                            parallelConditions,
                            subsystemOrder));

        }
    }

    public static class Executor{

        private static ArrayList<State> executable = null;

        private static ScheduledExecutorService service = Executors.newSingleThreadScheduledExecutor();

        private static boolean isExecuting;

        private static int index;

        public static void loadFile() throws Exceptions.PathFileDoesNotExist {
            try {
                executable = FileManager.loadFile(selectedPath);
            } catch (FileNotFoundException e) {
                throw new Exceptions.PathFileDoesNotExist("Path file does not exist: " + selectedPath);
            }
        }

        public static void startExecutor() throws Exceptions.MotionOfTheOceanIsNotReady {
            if(isParentNotReady() || executable == null)
                throw new Exceptions.MotionOfTheOceanIsNotReady("Parent or executable may not be ready or configured");
            service.scheduleAtFixedRate(MotionOfTheOcean.Executor::execute, 0, 20, TimeUnit.MILLISECONDS);
            isExecuting = true;
        }

        public static void stopExecutor() {
            service.shutdown();
            isExecuting = false;
        }

        public static void resetExecutor() {
            stopExecutor();
            executable = null;
        }

        public static void execute() {
            if(index >= executable.size()) {
                stopExecutor();
            }
            else {
                State state = executable.get(index);
                toChassisState.accept(state.getPose());
                HashMap<String, Double> dynamicStates = state.getDynamicStates();
                HashMap<String, Boolean> binaryStates = state.getBinaryStates();
                for (String subsystem : subsystemOrder) {
                    if (subsystem.charAt(0) == '~') {
                        dynamicExecutors.get(subsystem).accept(dynamicStates.get(subsystem));

                    }
                    else if (subsystem.charAt(0) == '-') {
                        binaryExecutors.get(subsystem).accept(binaryStates.get(subsystem));
                    }
                }
                index++;

            }

        }
    }
}
