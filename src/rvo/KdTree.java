package rvo;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.apache.commons.math3.util.FastMath;
import org.apache.commons.math3.util.Pair;

import java.util.ArrayList;
import java.util.List;


class KdTree {
    private Simulator simRef;

    KdTree(Simulator sim) {
        this.simRef = sim;
    }

    private static class AgentTreeNode {
        int begin = 0;
        int end = 0;
        int left = 0;
        int right = 0;
        double maxX = 0.0;
        double maxY = 0.0;
        double minX = 0.0;
        double minY = 0.0;
    }

    private static class ObstacleTreeNode {
        Obstacle obstacle = null;
        ObstacleTreeNode left = null;
        ObstacleTreeNode right = null;
    }

    private static final int MAX_LEAF_SIZE = 10;

    private Agent[] agents = null;
    private AgentTreeNode[] agentTree = null;
    private ObstacleTreeNode obstacleTree = null;


    void buildAgentTree() {
        if (agents == null || agents.length != simRef.agents.size()) {
            agents = new Agent[simRef.agents.size()];

            for (int agentNo = 0; agentNo < agents.length; agentNo++) {
                agents[agentNo] = simRef.agents.get(agentNo);
            }

            agentTree = new AgentTreeNode[2 * agents.length];

            for (int nodeNo = 0; nodeNo < agentTree.length; nodeNo++) {
                agentTree[nodeNo] = new AgentTreeNode();
            }
        }

        if (agents.length != 0) {
            buildAgentTreeRecursive(0, agents.length, 0);
        }
    }

    void buildObstacleTree() {
        obstacleTree = new ObstacleTreeNode();

        final List<Obstacle> obstacles = new ArrayList<>(simRef.obstacles.size());

        for (int obstacleNo = 0; obstacleNo < simRef.obstacles.size(); obstacleNo++) {
            obstacles.add(simRef.obstacles.get(obstacleNo));
        }

        obstacleTree = buildObstacleTreeRecursive(obstacles);
    }

    void computeAgentNeighbors(Agent agent, double rangeSq) {
        queryAgentTreeRecursive(agent, rangeSq, 0);
    }


    void computeObstacleNeighbors(Agent agent, double rangeSq) {
        queryObstacleTreeRecursive(agent, rangeSq, obstacleTree);
    }

    boolean queryVisibility(Vector2D q1, Vector2D q2, double radius) {
        return queryVisibilityRecursive(q1, q2, radius, obstacleTree);
    }

    private void buildAgentTreeRecursive(int begin, int end, int node) {
        agentTree[node].begin = begin;
        agentTree[node].end = end;
        agentTree[node].maxX = agents[begin].position.getX();
        agentTree[node].maxY = agents[begin].position.getY();
        agentTree[node].minX = agentTree[node].maxX;
        agentTree[node].minY = agentTree[node].maxY;


        for (int i = begin + 1; i < end; i++) {
            agentTree[node].maxX = FastMath.max(agentTree[node].maxX, agents[i].position.getX());
            agentTree[node].minX = FastMath.min(agentTree[node].minX, agents[i].position.getX());
            agentTree[node].maxY = FastMath.max(agentTree[node].maxY, agents[i].position.getY());
            agentTree[node].minY = FastMath.min(agentTree[node].minY, agents[i].position.getY());
        }

        if (end - begin > MAX_LEAF_SIZE) {
            final boolean isVertical = agentTree[node].maxX - agentTree[node].minX > agentTree[node].maxY - agentTree[node].minY;
            final double splitValue = 0.5 * (isVertical ? agentTree[node].maxX + agentTree[node].minX : agentTree[node].maxY + agentTree[node].minY);

            int left = begin;
            int right = end;

            while (left < right) {
                while (left < right && (isVertical ? agents[left].position.getX() : agents[left].position.getY()) < splitValue) {
                    left++;
                }

                while (right > left && (isVertical ? agents[right - 1].position.getX() : agents[right - 1].position.getY()) >= splitValue) {
                    right--;
                }

                if (left < right) {
                    final Agent tempAgent = agents[left];
                    agents[left] = agents[right - 1];
                    agents[right - 1] = tempAgent;
                    left++;
                    right--;
                }
            }

            if (left == begin) {
                left++;
            }

            agentTree[node].left = node + 1;
            agentTree[node].right = node + 2 * (left - begin);

            buildAgentTreeRecursive(begin, left, agentTree[node].left);
            buildAgentTreeRecursive(left, end, agentTree[node].right);
        }
    }

    private ObstacleTreeNode buildObstacleTreeRecursive(List<Obstacle> obstacles) {
        if (obstacles.isEmpty()) {
            return null;
        }

        final ObstacleTreeNode node = new ObstacleTreeNode();

        int optimalSplit = 0;
        int minLeft = obstacles.size();
        int minRight = obstacles.size();

        for (int i = 0; i < obstacles.size(); i++) {
            int leftSize = 0;
            int rightSize = 0;

            final Obstacle obstacleI1 = obstacles.get(i);
            Obstacle obstacleI2 = obstacleI1.next;

            for (int j = 0; j < obstacles.size(); j++) {
                if (i == j) {
                    continue;
                }

                final Obstacle obstacleJ1 = obstacles.get(j);
                final Obstacle obstacleJ2 = obstacleJ1.next;

                final double j1LeftOfI = MathUtil.leftOf(obstacleI1.point, obstacleI2.point, obstacleJ1.point);
                final double j2LeftOfI = MathUtil.leftOf(obstacleI1.point, obstacleI2.point, obstacleJ2.point);

                if (j1LeftOfI >= -MathUtil.EPSILON && j2LeftOfI >= -MathUtil.EPSILON) {
                    leftSize++;
                } else if (j1LeftOfI <= MathUtil.EPSILON && j2LeftOfI <= MathUtil.EPSILON) {
                    rightSize++;
                } else {
                    leftSize++;
                    rightSize++;
                }

                final Pair<Integer, Integer> pair1 = new Pair<>(FastMath.max(leftSize, rightSize), FastMath.min(leftSize, rightSize));
                final Pair<Integer, Integer> pair2 = new Pair<>(FastMath.max(minLeft, minRight), FastMath.min(minLeft, minRight));

                if (!(pair1.getFirst() < pair2.getFirst() || pair1.getFirst() <= pair2.getFirst() && pair1.getSecond() < pair2.getSecond())) {
                    break;
                }
            }

            final Pair<Integer, Integer> pair1 = new Pair<>(FastMath.max(leftSize, rightSize), FastMath.min(leftSize, rightSize));
            final Pair<Integer, Integer> pair2 = new Pair<>(FastMath.max(minLeft, minRight), FastMath.min(minLeft, minRight));

            if (pair1.getFirst() < pair2.getFirst() || pair1.getFirst() <= pair2.getFirst() && pair1.getSecond() < pair2.getSecond()) {
                minLeft = leftSize;
                minRight = rightSize;
                optimalSplit = i;
            }
        }

        final List<Obstacle> leftObstacles = new ArrayList<>(minLeft);

        for (int n = 0; n < minLeft; n++) {
            leftObstacles.add(null);
        }

        final List<Obstacle> rightObstacles = new ArrayList<>(minRight);

        for (int n = 0; n < minRight; n++) {
            rightObstacles.add(null);
        }

        int leftCounter = 0;
        int rightCounter = 0;

        final Obstacle obstacleI1 = obstacles.get(optimalSplit);
        final Obstacle obstacleI2 = obstacleI1.next;

        for (int j = 0; j < obstacles.size(); j++) {
            if (optimalSplit == j) {
                continue;
            }

            final Obstacle obstacleJ1 = obstacles.get(j);
            final Obstacle obstacleJ2 = obstacleJ1.next;

            final double j1LeftOfI = MathUtil.leftOf(obstacleI1.point, obstacleI2.point, obstacleJ1.point);
            final double j2LeftOfI = MathUtil.leftOf(obstacleI1.point, obstacleI2.point, obstacleJ2.point);

            if (j1LeftOfI >= -MathUtil.EPSILON && j2LeftOfI >= -MathUtil.EPSILON) {
                leftObstacles.set(leftCounter++, obstacles.get(j));
            } else if (j1LeftOfI <= MathUtil.EPSILON && j2LeftOfI <= MathUtil.EPSILON) {
                rightObstacles.set(rightCounter++, obstacles.get(j));
            } else {
                final double t = MathUtil.det(obstacleI2.point.subtract(obstacleI1.point), obstacleJ1.point.subtract(obstacleI1.point)) / MathUtil.det(obstacleI2.point.subtract(obstacleI1.point), obstacleJ1.point.subtract(obstacleJ2.point));

                final Vector2D splitPoint = obstacleJ1.point.add(obstacleJ2.point.subtract(obstacleJ1.point).scalarMultiply(t));

                final Obstacle newObstacle = new Obstacle();
                newObstacle.point = splitPoint;
                newObstacle.previous = obstacleJ1;
                newObstacle.next = obstacleJ2;
                newObstacle.convex = true;
                newObstacle.direction = obstacleJ1.direction;

                newObstacle.id = simRef.obstacles.size();

                simRef.obstacles.add(newObstacle);

                obstacleJ1.next = newObstacle;
                obstacleJ2.previous = newObstacle;

                if (j1LeftOfI > 0.0) {
                    leftObstacles.set(leftCounter++, obstacleJ1);
                    rightObstacles.set(rightCounter++, newObstacle);
                } else {
                    rightObstacles.set(rightCounter++, obstacleJ1);
                    leftObstacles.set(leftCounter++, newObstacle);
                }
            }
        }

        node.obstacle = obstacleI1;
        node.left = buildObstacleTreeRecursive(leftObstacles);
        node.right = buildObstacleTreeRecursive(rightObstacles);

        return node;
    }


    private static double sqr(double d) {
        return d * d;
    }

    private double queryAgentTreeRecursive(Agent agent, double rangeSq, int node) {
        if (agentTree[node].end - agentTree[node].begin <= MAX_LEAF_SIZE) {
            for (int agentNo = agentTree[node].begin; agentNo < agentTree[node].end; agentNo++) {
                rangeSq = agent.insertAgentNeighbor(agents[agentNo], rangeSq);
            }
        } else {
            final double distanceSqLeft = sqr(FastMath.max(0.0, agentTree[agentTree[node].left].minX - agent.position.getX())) + sqr(FastMath.max(0.0, agent.position.getX() - agentTree[agentTree[node].left].maxX)) + sqr(FastMath.max(0.0, agentTree[agentTree[node].left].minY - agent.position.getY())) + sqr(FastMath.max(0.0, agent.position.getY() - agentTree[agentTree[node].left].maxY));
            final double distanceSqRight = sqr(FastMath.max(0.0, agentTree[agentTree[node].right].minX - agent.position.getX())) + sqr(FastMath.max(0.0, agent.position.getX() - agentTree[agentTree[node].right].maxX)) + sqr(FastMath.max(0.0, agentTree[agentTree[node].right].minY - agent.position.getY())) + sqr(FastMath.max(0.0, agent.position.getY() - agentTree[agentTree[node].right].maxY));

            if (distanceSqLeft < distanceSqRight) {
                if (distanceSqLeft < rangeSq) {
                    rangeSq = queryAgentTreeRecursive(agent, rangeSq, agentTree[node].left);

                    if (distanceSqRight < rangeSq) {
                        rangeSq = queryAgentTreeRecursive(agent, rangeSq, agentTree[node].right);
                    }
                }
            } else {
                if (distanceSqRight < rangeSq) {
                    rangeSq = queryAgentTreeRecursive(agent, rangeSq, agentTree[node].right);

                    if (distanceSqLeft < rangeSq) {
                        rangeSq = queryAgentTreeRecursive(agent, rangeSq, agentTree[node].left);
                    }
                }
            }

        }

        return rangeSq;
    }

    private static void queryObstacleTreeRecursive(Agent agent, double rangeSq, ObstacleTreeNode node) {
        if (node != null) {
            final Obstacle obstacle1 = node.obstacle;
            final Obstacle obstacle2 = obstacle1.next;

            final double agentLeftOfLine = MathUtil.leftOf(obstacle1.point, obstacle2.point, agent.position);

            queryObstacleTreeRecursive(agent, rangeSq, agentLeftOfLine >= 0.0 ? node.left : node.right);

            final double distanceSqLine = agentLeftOfLine * agentLeftOfLine / obstacle2.point.distanceSq(obstacle1.point);

            if (distanceSqLine < rangeSq) {
                if (agentLeftOfLine < 0.0) {
                    agent.insertObstacleNeighbor(node.obstacle, rangeSq);
                }

                queryObstacleTreeRecursive(agent, rangeSq, agentLeftOfLine >= 0.0 ? node.right : node.left);
            }
        }
    }

    private static boolean queryVisibilityRecursive(Vector2D q1, Vector2D q2, double radius, ObstacleTreeNode node) {
        if (node == null) {
            return true;
        }

        final Obstacle obstacle1 = node.obstacle;
        final Obstacle obstacle2 = obstacle1.next;

        final double q1LeftOfI = MathUtil.leftOf(obstacle1.point, obstacle2.point, q1);
        final double q2LeftOfI = MathUtil.leftOf(obstacle1.point, obstacle2.point, q2);
        final double invLengthI = 1.0 / obstacle2.point.distanceSq(obstacle1.point);

        if (q1LeftOfI >= 0.0 && q2LeftOfI >= 0.0) {
            return queryVisibilityRecursive(q1, q2, radius, node.left) && (q1LeftOfI * q1LeftOfI * invLengthI >= radius * radius && q2LeftOfI * q2LeftOfI * invLengthI >= radius * radius || queryVisibilityRecursive(q1, q2, radius, node.right));
        }

        if (q1LeftOfI <= 0.0 && q2LeftOfI <= 0.0) {
            return queryVisibilityRecursive(q1, q2, radius, node.right) && (q1LeftOfI * q1LeftOfI * invLengthI >= radius * radius && q2LeftOfI * q2LeftOfI * invLengthI >= radius * radius || queryVisibilityRecursive(q1, q2, radius, node.left));
        }

        if (q1LeftOfI >= 0.0 && q2LeftOfI <= 0.0) {
            return queryVisibilityRecursive(q1, q2, radius, node.left) && queryVisibilityRecursive(q1, q2, radius, node.right);
        }

        final double point1LeftOfQ = MathUtil.leftOf(q1, q2, obstacle1.point);
        final double point2LeftOfQ = MathUtil.leftOf(q1, q2, obstacle2.point);
        final double invLengthQ = 1.0 / q2.distanceSq(q1);

        return point1LeftOfQ * point2LeftOfQ >= 0.0 && point1LeftOfQ * point1LeftOfQ * invLengthQ > radius * radius && point2LeftOfQ * point2LeftOfQ * invLengthQ > radius * radius && queryVisibilityRecursive(q1, q2, radius, node.left) && queryVisibilityRecursive(q1, q2, radius, node.right);
    }
}
