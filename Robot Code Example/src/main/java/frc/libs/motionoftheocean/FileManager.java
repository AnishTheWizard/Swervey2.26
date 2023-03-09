package frc.libs.motionoftheocean;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Scanner;
import java.util.function.Supplier;

public class FileManager {

    private static String directory;
    private static String[] pathNames = null;

    public static void configureWorkingDirectory(String dir) {
        directory = dir;
    }

    public static void export(String filename, ArrayList<State> recording, String headers) throws IOException {
        new File(filename).createNewFile();
        FileWriter writer = new FileWriter(filename);
        writer.write(headers + "\n");
        for(State state : recording) {
            writer.write(state.toString());
        }
        writer.flush();
        writer.close();
    }

    public static ArrayList<State> loadFile(String filename) throws FileNotFoundException {
        Scanner reader = new Scanner(new File(filename));
        ArrayList<State> file = new ArrayList<>();
        String[] headers = reader.nextLine().split(",");
        while(reader.hasNextLine()) {
            String line = reader.nextLine();
            String[] data = line.split(",");
            double[] chassisMotion = new double[]{Double.parseDouble(data[0]),
                    Double.parseDouble(data[1]),
                    Double.parseDouble(data[2]),
                    Double.parseDouble(data[3])};
            HashMap<String, Double> dynamicStates = new HashMap<>();
            HashMap<String, Boolean> binaryStates = new HashMap<>();
            for(int i = 5; i < data.length; i++) {
                String name = headers[i];
                if(name.charAt(0) == '~') {
                    dynamicStates.put(name, Double.parseDouble(data[i]));
                }
                else if(name.charAt(0) == '-') {
                    binaryStates.put(name, Boolean.parseBoolean(data[i]));
                }
            }
            ArrayList<String> subsystemOrder = new ArrayList<>(Arrays.asList(Arrays.copyOfRange(headers, 5, headers.length)));
            State state = new State(
                    chassisMotion,
                    dynamicStates,
                    binaryStates,
                    new HashMap<String, Supplier<Boolean>>(),
                    subsystemOrder
            );
            file.add(state);
        }
        return file;
    }

}
