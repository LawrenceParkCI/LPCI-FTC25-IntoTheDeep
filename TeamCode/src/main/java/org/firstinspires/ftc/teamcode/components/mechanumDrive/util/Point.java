package org.firstinspires.ftc.teamcode.components.mechanumDrive.util;

import androidx.annotation.NonNull;

public class Point {
    private double x;
    private double y;

    public Point(double x, double y){
        this.x = x;
        this.y = y;
    }

    public void setX(double x){
        this.x = x;
    }

    public double getX(){
        return x;
    }

    public void setY(double y){
        this.y = y;
    }

    public double getY(){
        return y;
    }

    @NonNull
    @Override
    public String toString(){
        return "(" + x + ", " + y + ")";
    }

    @NonNull
    @Override
    public Point clone(){
        return new Point(x, y);
    }
}
