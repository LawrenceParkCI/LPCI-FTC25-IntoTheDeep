package org.firstinspires.ftc.teamcode.components.mechanumDrive.util;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
/**
 * FTC Into The Deep 24-25 <br>
 * A Simple point class.
 * <br><br>
 * Last Updated: November 21st, 2024
 * @author Connor Feeney
 */
public class Point {
    private double x;
    private double y;

    /**
     * Point constructor.
     * @param x x value
     * @param y y value
     */
    public Point(double x, double y){
        this.x = x;
        this.y = y;
    }

    /**
     * Set the points x value.
     * @param x x value
     */
    public void setX(double x){
        this.x = x;
    }

    /**
     * Get the points x component.
     * @return the x component
     */
    public double getX(){
        return x;
    }

    /**
     * Set the points y value.
     * @param y y value
     */
    public void setY(double y){
        this.y = y;
    }

    /**
     * Get the points y component.
     * @return the y component
     */
    public double getY(){
        return y;
    }

    /**
     * Calculate distance to another point.
     * @param point Other point
     * @return Distance
     */
    public double distanceTo(Point point){
        return Math.sqrt(Math.pow(point.getX() - this.getX(), 2) + Math.pow(point.getY() - this.getY(), 2));
    }

    /**
     * Check if an other point is within a certain range of this point.
     * @param range the range
     * @param point the other point
     * @return if in range
     */
    public boolean inRange(double range, Point point){
        return Math.abs(this.distanceTo(point)) < range;
    }

    @NonNull
    @Override
    public String toString(){
        return "(" + this.x + ", " + this.y + ")";
    }

    @NonNull
    @Override
    public Point clone(){
        return new Point(x, y);
    }

    @Override
    public boolean equals(@Nullable Object obj) {
        assert obj != null;
        if(obj.getClass() == this.getClass()){
            return ((Point) obj).getX() == this.getX() && ((Point) obj).getY() == this.getY();
        }

        return false;
    }

    @Override
    public int hashCode() {
        return 31 + Double.hashCode(x) * Double.hashCode(y);
    }
}
