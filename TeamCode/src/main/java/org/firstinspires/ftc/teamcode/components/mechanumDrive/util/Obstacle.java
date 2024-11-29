package org.firstinspires.ftc.teamcode.components.mechanumDrive.util;

/**
 * FTC Into The Deep 24-25 <br>
 * Obstacle class for {@link TrajectoryPlanner TrajectoryPlanner}.
 * <br><br>
 * Last Updated: November 21st, 2024
 * @author Connor Feeney
 */
public class Obstacle {
    private static double rWidth = 0;
    private static double rLength = 0;

    private final Point position;
    private final double width;
    private final double length;

    public Obstacle(Point position, double width, double length){
        this.position = position.clone();
        this.width = width;
        this.length = length;
    }

    public static void setRobotDimensions(double rWidth, double rLength){
        Obstacle.rWidth = rWidth;
        Obstacle.rLength = rLength;
    }

    public boolean collides(Point robotPosition, Point nextPosition){
        return collidesOnBounds(robotPosition) || intersectsOnPath(robotPosition, nextPosition);
    }

    private boolean collidesOnBounds(Point robotPosition){
        //Calculate bounding box with robot radius padding
        double effectiveMinX = position.getX() - (rWidth/2);
        double effectiveMinY = position.getY() - length - (rLength/2);

        double effectiveMaxX = position.getX() + width + (rWidth/2);
        double effectiveMaxY = position.getY() + (rLength/2);

        return robotPosition.getX() >= effectiveMinX && robotPosition.getX() <= effectiveMaxX
                && robotPosition.getY() >= effectiveMinY && robotPosition.getY() <= effectiveMaxY;
    }

    private boolean intersectsOnPath(Point prev, Point next){
        double effectiveMinX = position.getX() - (rWidth/2);
        double effectiveMinY = position.getY() - length - (rLength/2);

        double effectiveMaxX = position.getX() + width + (rWidth/2);
        double effectiveMaxY = position.getY() + (rLength/2);

        return lineIntersects(prev, next, new Point(effectiveMinX, effectiveMinY), new Point(effectiveMaxX, effectiveMinY)) || //Bottom Edge
                lineIntersects(prev, next, new Point(effectiveMinX, effectiveMaxY), new Point(effectiveMaxX, effectiveMaxY)) || //Top Edge
                lineIntersects(prev, next, new Point(effectiveMinX, effectiveMinY), new Point(effectiveMaxX, effectiveMaxY)) || //Left Edge
                lineIntersects(prev, next, new Point(effectiveMaxX, effectiveMaxY), new Point(effectiveMaxX, effectiveMinY)); //Right Edge
    }

    private boolean lineIntersects(Point p1, Point q1, Point p2, Point q2){
        int d1 = direction(p1, q1, p2);
        int d2 = direction(p1, q1, q2);
        int d3 = direction(p2, q2, p1);
        int d4 = direction(p2, q2, q1);

        if(d1 != d2 && d3 != d4){
            return true;
        }

        if (d1 == 0 && onSegment(p1, p2, q1)) return true;
        if (d2 == 0 && onSegment(p1, q2, q1)) return true;
        if (d3 == 0 && onSegment(p2, p1, q2)) return true;
        if (d4 == 0 && onSegment(p2, q1, q2)) return true;

        return false;
    }

    private int direction(Point p, Point q, Point r){
        double val = (q.getY() - p.getY()) * (r.getX() - q.getX()) * (q.getX() - p.getX()) * (r.getY() - q.getY());

        if (val == 0) return 0; //Collinear
        return (val > 0) ? 1 : 2; //Clockwise or Counter Clockwise
    }

    private boolean onSegment(Point p, Point q, Point r) {
        return q.getX() <= Math.max(p.getX(), r.getX()) && q.getX() >= Math.min(p.getX(), r.getX()) &&
                q.getY() <= Math.max(p.getY(), r.getY()) && q.getY() >= Math.min(p.getY(), r.getY());
    }

}
