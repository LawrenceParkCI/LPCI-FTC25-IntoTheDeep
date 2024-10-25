package org.firstinspires.ftc.teamcode.util;

import android.content.Context;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.opencv.core.CvType;
import org.opencv.core.Mat;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

/**
 * A class for loading and storing matrices on the robot control hub for transfer between OpModes
 */
public class MatLoader {
    HardwareMap hardwareMap;

    /**
     * @param hardwareMap Your OpModes hardwareMap
     */
    public MatLoader(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
    }

    /**
     * Write a Mat file to storage
     * @param mat The matrix to store
     * @param fname The file name to store it under
     * @return True if the file stores successfully
     */
    public boolean writeMatFile(Mat mat, String fname){
        //Convert the mat into a string
        StringBuilder sb = new StringBuilder();
        for(int i = 0; i < mat.rows(); i++){
            for(int j = 0; j < mat.cols(); j++){
                sb.append(mat.get(i, j)[0]).append(" ");
            }
            sb.append("\n");
        }

        //Load the app context and create a file
        Context context = hardwareMap.appContext;
        File file = new File(context.getExternalFilesDir(null), fname);

        //Write the file
        try(BufferedWriter writer = new BufferedWriter(new FileWriter(file))){
            writer.write(sb.toString());
        }catch(IOException e){
            return false; //Return false on exception
        }

        return true;
    }

    /**
     * Load a Mat file from storage
     * @param fname The file name
     * @return The loaded Mat, returns null if no mat is loaded
     */
    public Mat loadMatFile(String fname){
        Mat mat; // Output mat

        //Sizes
        int rowCount = 0;
        int colCount = 0;

        Context context = hardwareMap.appContext; //android app context for file management
        File file = new File(context.getExternalFilesDir(null), fname);
        List<Double> data = new ArrayList<>();
        try(BufferedReader reader  = new BufferedReader(new FileReader(file))){
            //Load each number from the mat file into the data list
            String line;
            while((line = reader.readLine()) != null){
                rowCount++;
                String[] nums = line.split(" ");
                colCount = nums.length;
                for (String s : nums) {
                    Double num = Double.parseDouble(s.trim());
                    data.add(num);
                }
            }

            //Create a mat with the new found mat size
            mat = new Mat(rowCount, colCount, CvType.CV_64F);

            //convert data into a flat array because Mat likes can only take primitives and .toArray() gives an object
            double[] dataArray = new double[data.size()];
            for(int i = 0; i < data.size(); i++){
                dataArray[i] = data.get(i);
            }
            mat.put(0,0, dataArray);
        }catch (IOException e){
            return null; //Return null if there are any issues retrieving mat file (if this happens restart the controller try again or its your fault)
        }

        return mat; //Hand back our newly constructed mat
    }
}
