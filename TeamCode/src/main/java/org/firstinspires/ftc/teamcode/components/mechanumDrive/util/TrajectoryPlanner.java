package org.firstinspires.ftc.teamcode.components.mechanumDrive.util;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.Set;

/**
 * FTC Into The Deep 24-25 <br>
 * A Class for generating a trajectory for a robot to follow with obstacle avoidance.
 * <br><br>
 * Last Updated: November 21st, 2024
 * @author Connor Feeney
 */
public class TrajectoryPlanner {
    private static double step = 0.10;
    private static double angleRes = 4;

    public static void setResolution(double step, double angleRes){
        TrajectoryPlanner.step = step;
        TrajectoryPlanner.angleRes = angleRes;
    }

    private static class Node{
        private final Point pos;

        private final double gCost, hCost;

        private Node parent;

        public Node(Point pos, double gCost, double hCost){
            this.pos = pos.clone();
            this.gCost = gCost;
            this.hCost = hCost;
        }

        public Point getPos(){
            return pos.clone();
        }

        public void setParent(Node parent){
            this.parent = parent;
        }

        public Node getParent(){
            return parent;
        }

        public double fCost(){
            return hCost + gCost;
        }

        public double gCost() {
            return gCost;
        }
    }

    private static double calculateHeuristic(Point start, Point end){
        return start.distanceTo(end);
    }

    private static boolean isValid(Point point, Point prev,List<Obstacle> obstacles){
        for(Obstacle obstacle : obstacles){
            if(obstacle.collides(point, prev)) return false;
        }
        return true;
    }

    public static List<Point> generateTrajectory(Point start, Point end, List<Obstacle> obstacles){
        PriorityQueue<Node> openList = new PriorityQueue<>(Comparator.comparingDouble(Node::fCost)); //Store all nodes to check
        Set<Point> closedList = new HashSet<>(); //Store all checked nodes
        Map<Point, Node> gCostMap = new HashMap<>();

        //Set start point as first node
        Node startNode = new Node(start, 0, calculateHeuristic(start, end));
        openList.add(startNode);

        while(!openList.isEmpty()){
            //Poll a new node to traverse
            Node currentNode = openList.poll();
            assert currentNode != null;

            //Check if node is close enough to end point to return path
            if(currentNode.getPos().inRange(step, end)){
                List<Point> path = new ArrayList<>();

                //Back tracking path
                Node temp = currentNode;
                while(temp != null){
                    path.add(temp.getPos());
                    temp = temp.getParent();
                }

                //Reverse the path
                Collections.reverse(path);

                //Mark the endpoint as the final waypoint in the path
                path.add(end);

                return path;
            }

            //Mark that we have traversed the node
            closedList.add(currentNode.getPos());

            for(double angle = 0; angle < 2 * Math.PI; angle += (2 * Math.PI)/angleRes){
                //Calculate the child positions
                double nextX = currentNode.getPos().getX() + step * Math.cos(angle);
                double nextY = currentNode.getPos().getY() + step * Math.sin(angle);

                Point childPos = new Point(nextX, nextY);

                //If the closed list already contains the child node or the child node crosses and obstacle ignore it
                if(closedList.contains(childPos) || !isValid(childPos, currentNode.getPos(), obstacles)) continue;

                //Calculate the child nodes cost values
                double gCost = currentNode.gCost() + step;
                double hCost = calculateHeuristic(childPos, end);

                //Check if there is already an existing node with same position and lower cost a lower cost
                if(gCostMap.containsKey(childPos)){
                    Node existingNode = gCostMap.get(childPos);
                    assert existingNode != null;
                    if(gCost < existingNode.gCost()){
                        openList.remove(existingNode);

                        Node child = new Node(childPos, gCost, hCost);
                        child.setParent(currentNode);
                        openList.add(child);

                        gCostMap.put(childPos, child);
                    }
                }else{
                    Node child = new Node(childPos, gCost, hCost);
                    child.setParent(currentNode);
                    openList.add(child);

                    gCostMap.put(childPos, child);
                }
            }
        }

        //No path found
        return Collections.emptyList();
    }
}
