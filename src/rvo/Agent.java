package rvo;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.apache.commons.math3.util.FastMath;
import org.apache.commons.math3.util.Pair;

import java.util.ArrayList;
import java.util.List;

public class Agent {
    private Simulator simRef;

    int id = 0;
    int maxNeighbors = 0;
    double maxSpeed = 0.0;
    double prefSpeed = 0.0;
    double neighborDistance = 0.0;
    double radius = 0.0;
    double timeHorizonAgents = 0.0;
    double timeHorizonObstacles = 0.0;
    int start, end;

    // 误差测量
    public int diff, goFrames;

    Vector2D goal;
    Vector2D position = Vector2D.ZERO;
    Vector2D velocity = Vector2D.ZERO;
    Vector2D newVelocity = Vector2D.ZERO;
    Vector2D preferredVelocity = Vector2D.ZERO;

    final List<Pair<Double, Agent>> agentNeighbors = new ArrayList<>();
    final List<Pair<Double, Obstacle>> obstacleNeighbors = new ArrayList<>();
    final List<Line> lines = new ArrayList<>();

    public Agent(Simulator sim) {
        this.simRef = sim;
    }
    public Agent(Agent other, Simulator newSim) {
        goal = other.goal;
        position = other.position;
        velocity = other.velocity;
        simRef = newSim;

        id = other.id;
        maxNeighbors = other.maxNeighbors;
        maxSpeed = other.maxSpeed;
        prefSpeed = other.prefSpeed;
        neighborDistance = other.neighborDistance;
        radius = other.radius;
        timeHorizonAgents = other.timeHorizonAgents;
        timeHorizonObstacles = other.timeHorizonObstacles;
        start = other.start;
        end = other.end;

        diff = 0;
        goFrames = 0;
    }

    public int getId() {
        return id;
    }
    public Vector2D getPosition() {
        return position;
    }
    public double getPrefSpeed() {
        return prefSpeed;
    }
    public double getRadius() {
        return radius;
    }
    public void setPosition(Vector2D position) {
        this.position = position;
    }

    public boolean canShowUp(int step) {
        return start <= step;
    }
    public boolean reachedGoal() {
        return position.distance(goal) < 0.1;
    }

    public void setPara(double r, double neighborDist, double maxNeighbor, double timeHorizon, double timObst, double prefS) {
        radius = r;
        neighborDistance = neighborDist;
        maxNeighbors = (int) maxNeighbor;
        timeHorizonAgents = timeHorizon;
        timeHorizonObstacles = timObst;
        prefSpeed = prefS;
        maxSpeed = prefSpeed * 2;
    }

    public void setPreferredVelocity() {
        Vector2D prefV = goal.subtract(position).normalize();
        double distanceToGoal = position.distance(goal);

        if (distanceToGoal < prefSpeed * simRef.timeStep) {
            preferredVelocity = prefV.scalarMultiply(distanceToGoal / simRef.timeStep);
        } else {
            preferredVelocity = prefV.scalarMultiply(prefSpeed);
        }
    }

    void calcDiff(Vector2D truePos, double truePosDiff) {
//        double t = truePos.distance(position) / truePosDiff;
        double t = truePos.distance(position);
        diff += t;
        goFrames++;
    }

    // 感知周围近邻
    void computeNeighbors() {
        obstacleNeighbors.clear();
        final double range = timeHorizonObstacles * maxSpeed + radius;
        simRef.kdTree.computeObstacleNeighbors(this, range * range);

        agentNeighbors.clear();

        if (maxNeighbors > 0) {
            simRef.kdTree.computeAgentNeighbors(this, neighborDistance * neighborDistance);
        }
    }

    // 决策新的速度
    void computeNewVelocity() {
        lines.clear();

        final double invTimeHorizonObstacle = 1.0 / timeHorizonObstacles;

        for (final Pair<Double, Obstacle> obstacleNeighbor : obstacleNeighbors) {
            Obstacle obstacle1 = obstacleNeighbor.getSecond();
            Obstacle obstacle2 = obstacle1.next;

            final Vector2D relativePosition1 = obstacle1.point.subtract(position);
            final Vector2D relativePosition2 = obstacle2.point.subtract(position);

            boolean alreadyCovered = false;

            for (final Line orcaLine : lines) {
                if (MathUtil.det(relativePosition1.scalarMultiply(invTimeHorizonObstacle).subtract(orcaLine.point), orcaLine.direction) - invTimeHorizonObstacle * radius >= -MathUtil.EPSILON && MathUtil.det(relativePosition2.scalarMultiply(invTimeHorizonObstacle).subtract(orcaLine.point), orcaLine.direction) - invTimeHorizonObstacle * radius >= -MathUtil.EPSILON) {
                    alreadyCovered = true;

                    break;
                }
            }

            if (alreadyCovered) {
                continue;
            }

            final double distanceSq1 = relativePosition1.getNormSq();
            final double distanceSq2 = relativePosition2.getNormSq();
            final double radiusSq = radius * radius;

            final Vector2D obstacleVector = obstacle2.point.subtract(obstacle1.point);
            final double s = -relativePosition1.dotProduct(obstacleVector) / obstacleVector.getNormSq();
            final double distanceSqLine = relativePosition1.add(s, obstacleVector).getNormSq();

            if (s < 0.0 && distanceSq1 <= radiusSq) {
                // 为凸，且与左顶点碰撞
                if (obstacle1.convex) {
                    final Vector2D direction = new Vector2D(-relativePosition1.getY(), relativePosition1.getX()).normalize();
                    lines.add(new Line(Vector2D.ZERO, direction));
                }

                continue;
            }

            if (s > 1.0 && distanceSq2 <= radiusSq) {
                // 为凸，且与右顶点碰撞
                if (obstacle2.convex && MathUtil.det(relativePosition2, obstacle2.direction) >= 0.0) {
                    final Vector2D direction = new Vector2D(-relativePosition2.getY(), relativePosition2.getX()).normalize();
                    lines.add(new Line(Vector2D.ZERO, direction));
                }

                continue;
            }

            if (s >= 0.0 && s < 1.0 && distanceSqLine <= radiusSq) {
                final Vector2D direction = obstacle1.direction.negate();
                lines.add(new Line(Vector2D.ZERO, direction));

                continue;
            }

            Vector2D leftLegDirection;
            Vector2D rightLegDirection;

            if (s < 0.0 && distanceSqLine <= radiusSq) {
                if (!obstacle1.convex) {
                    continue;
                }

                obstacle2 = obstacle1;

                final double leg1 = FastMath.sqrt(distanceSq1 - radiusSq);
                leftLegDirection = new Vector2D(relativePosition1.getX() * leg1 - relativePosition1.getY() * radius, relativePosition1.getX() * radius + relativePosition1.getY() * leg1).scalarMultiply(1.0 / distanceSq1);
                rightLegDirection = new Vector2D(relativePosition1.getX() * leg1 + relativePosition1.getY() * radius, -relativePosition1.getX() * radius + relativePosition1.getY() * leg1).scalarMultiply(1.0 / distanceSq1);
            } else if (s > 1.0 && distanceSqLine <= radiusSq) {
                if (!obstacle2.convex) {
                    continue;
                }

                obstacle1 = obstacle2;

                final double leg2 = FastMath.sqrt(distanceSq2 - radiusSq);
                leftLegDirection = new Vector2D(relativePosition2.getX() * leg2 - relativePosition2.getY() * radius, relativePosition2.getX() * radius + relativePosition2.getY() * leg2).scalarMultiply(1.0 / distanceSq2);
                rightLegDirection = new Vector2D(relativePosition2.getX() * leg2 + relativePosition2.getY() * radius, -relativePosition2.getX() * radius + relativePosition2.getY() * leg2).scalarMultiply(1.0 / distanceSq2);
            } else {
                if (obstacle1.convex) {
                    final double leg1 = FastMath.sqrt(distanceSq1 - radiusSq);
                    leftLegDirection = new Vector2D(relativePosition1.getX() * leg1 - relativePosition1.getY() * radius, relativePosition1.getX() * radius + relativePosition1.getY() * leg1).scalarMultiply(1.0 / distanceSq1);
                } else {
                    leftLegDirection = obstacle1.direction.negate();
                }

                if (obstacle2.convex) {
                    final double leg2 = FastMath.sqrt(distanceSq2 - radiusSq);
                    rightLegDirection = new Vector2D(relativePosition2.getX() * leg2 + relativePosition2.getY() * radius, -relativePosition2.getX() * radius + relativePosition2.getY() * leg2).scalarMultiply(1.0 / distanceSq2);
                } else {
                    rightLegDirection = obstacle1.direction;
                }
            }

            boolean leftLegForeign = false;
            boolean rightLegForeign = false;

            if (obstacle1.convex && MathUtil.det(leftLegDirection, obstacle1.previous.direction.negate()) >= 0.0) {
                leftLegDirection = obstacle1.previous.direction.negate();
                leftLegForeign = true;
            }

            if (obstacle2.convex && MathUtil.det(rightLegDirection, obstacle2.direction) <= 0.0) {
                rightLegDirection = obstacle2.direction;
                rightLegForeign = true;
            }

            final Vector2D leftCutOff = obstacle1.point.subtract(position).scalarMultiply(invTimeHorizonObstacle);
            final Vector2D rightCutOff = obstacle2.point.subtract(position).scalarMultiply(invTimeHorizonObstacle);
            final Vector2D cutOffVector = rightCutOff.subtract(leftCutOff);

            final double t = obstacle1 == obstacle2 ? 0.5 : velocity.subtract(leftCutOff).dotProduct(cutOffVector) / cutOffVector.getNormSq();
            final double tLeft = velocity.subtract(leftCutOff).dotProduct(leftLegDirection);
            final double tRight = velocity.subtract(rightCutOff).dotProduct(rightLegDirection);

            if (t < 0.0 && tLeft < 0.0 || obstacle1 == obstacle2 && tLeft < 0.0 && tRight < 0.0) {
                final Vector2D unitW = velocity.subtract(leftCutOff).normalize();

                final Vector2D direction = new Vector2D(unitW.getY(), -unitW.getX());
                final Vector2D point = leftCutOff.add(radius * invTimeHorizonObstacle, unitW);
                lines.add(new Line(point, direction));

                continue;
            }

            if (t > 1.0 && tRight < 0.0) {
                final Vector2D unitW = velocity.subtract(rightCutOff).normalize();

                final Vector2D direction = new Vector2D(unitW.getY(), -unitW.getX());
                final Vector2D point = rightCutOff.add(radius * invTimeHorizonObstacle, unitW);
                lines.add(new Line(point, direction));

                continue;
            }

            final double distanceSqCutOff = t < 0.0 || t > 1.0 || obstacle1 == obstacle2 ? Double.POSITIVE_INFINITY : velocity.distanceSq(leftCutOff.add(cutOffVector.scalarMultiply(t)));
            final double distanceSqLeft = tLeft < 0.0 ? Double.POSITIVE_INFINITY : velocity.distanceSq(leftCutOff.add(leftLegDirection.scalarMultiply(tLeft)));
            final double distanceSqRight = tRight < 0.0 ? Double.POSITIVE_INFINITY : velocity.distanceSq(rightCutOff.add(rightLegDirection.scalarMultiply(tRight)));

            if (distanceSqCutOff <= distanceSqLeft && distanceSqCutOff <= distanceSqRight) {
                final Vector2D direction = obstacle1.direction.negate();
                final Vector2D point = leftCutOff.add(radius * invTimeHorizonObstacle, new Vector2D(-direction.getY(), direction.getX()));
                lines.add(new Line(point, direction));

                continue;
            }

            if (distanceSqLeft <= distanceSqRight) {
                if (leftLegForeign) {
                    continue;
                }

                final Vector2D point = leftCutOff.add(radius * invTimeHorizonObstacle, new Vector2D(-leftLegDirection.getY(), leftLegDirection.getX()));
                lines.add(new Line(point, leftLegDirection));

                continue;
            }

            if (rightLegForeign) {
                continue;
            }

            final Vector2D direction = rightLegDirection.negate();
            final Vector2D point = rightCutOff.add(radius * invTimeHorizonObstacle, new Vector2D(-direction.getY(), direction.getX()));
            lines.add(new Line(point, direction));
        }

        final int numObstacleLines = lines.size();

        final double invTimeHorizon = 1.0 / timeHorizonAgents;

        for (final Pair<Double, Agent> agentNeighbor : agentNeighbors) {
            final Agent other = agentNeighbor.getSecond();

            final Vector2D relativePosition = other.position.subtract(position);
            final Vector2D relativeVelocity = velocity.subtract(other.velocity);
            final double distanceSq = relativePosition.getNormSq();
            final double combinedRadius = radius + other.radius;
            final double combinedRadiusSq = combinedRadius * combinedRadius;

            final Vector2D direction;
            final Vector2D u;

            if (distanceSq > combinedRadiusSq) {
                final Vector2D w = relativeVelocity.subtract(invTimeHorizon, relativePosition);

                final double wLengthSq = w.getNormSq();
                final double dotProduct1 = w.dotProduct(relativePosition);

                if (dotProduct1 < 0.0 && dotProduct1 * dotProduct1 > combinedRadiusSq * wLengthSq) {
                    final double wLength = FastMath.sqrt(wLengthSq);
                    final Vector2D unitW = w.scalarMultiply(1.0 / wLength);

                    direction = new Vector2D(unitW.getY(), -unitW.getX());
                    u = unitW.scalarMultiply(combinedRadius * invTimeHorizon - wLength);
                } else {
                    final double leg = FastMath.sqrt(distanceSq - combinedRadiusSq);

                    if (MathUtil.det(relativePosition, w) > 0.0) {
                        direction = new Vector2D(relativePosition.getX() * leg - relativePosition.getY() * combinedRadius, relativePosition.getX() * combinedRadius + relativePosition.getY() * leg).scalarMultiply(1.0 / distanceSq);
                    } else {
                        direction = new Vector2D(relativePosition.getX() * leg + relativePosition.getY() * combinedRadius, -relativePosition.getX() * combinedRadius + relativePosition.getY() * leg).scalarMultiply(-1.0 / distanceSq);
                    }

                    final double dotProduct2 = relativeVelocity.dotProduct(direction);
                    u = direction.scalarMultiply(dotProduct2).subtract(relativeVelocity);
                }
            } else {
                final double invTimeStep = 1.0 / simRef.timeStep;
                final Vector2D w = relativeVelocity.subtract(invTimeStep, relativePosition);

                final double wLength = w.getNorm();
                final Vector2D unitW = w.scalarMultiply(1.0 / wLength);

                direction = new Vector2D(unitW.getY(), -unitW.getX());
                u = unitW.scalarMultiply(combinedRadius * invTimeStep - wLength);
            }

            final Vector2D point = velocity.add(0.5, u);
            lines.add(new Line(point, direction));
        }

        final int lineFail = linearProgram2(lines, preferredVelocity, false);

        if (lineFail < lines.size()) {
            linearProgram3(numObstacleLines, lineFail);
        }
    }

    double insertAgentNeighbor(Agent agent, double rangeSq) {
        if (this != agent) {
            final double distSq = position.distanceSq(agent.position);

            if (distSq < rangeSq) {
                if (agentNeighbors.size() < maxNeighbors) {
                    agentNeighbors.add(new Pair<>(distSq, agent));
                }

                int i = agentNeighbors.size() - 1;

                while (i != 0 && distSq < agentNeighbors.get(i - 1).getFirst()) {
                    agentNeighbors.set(i, agentNeighbors.get(i - 1));
                    i--;
                }

                agentNeighbors.set(i, new Pair<>(distSq, agent));

                if (agentNeighbors.size() == maxNeighbors) {
                    rangeSq = agentNeighbors.get(agentNeighbors.size() - 1).getFirst();
                }
            }
        }

        return rangeSq;
    }

    void insertObstacleNeighbor(Obstacle obstacle, double rangeSq) {
        final Obstacle nextObstacle = obstacle.next;

        final double r = position.subtract(obstacle.point).dotProduct(nextObstacle.point.subtract(obstacle.point)) / nextObstacle.point.distanceSq(obstacle.point);
        final double distSq;

        if (r < 0.0) {
            distSq = position.distanceSq(obstacle.point);
        } else if (r > 1.0) {
            distSq = position.distanceSq(nextObstacle.point);
        } else {
            distSq = position.distanceSq(obstacle.point.add(nextObstacle.point.subtract(obstacle.point).scalarMultiply(r)));
        }

        if (distSq < rangeSq) {
            obstacleNeighbors.add(new Pair<>(distSq, obstacle));

            int i = obstacleNeighbors.size() - 1;

            while (i != 0 && distSq < obstacleNeighbors.get(i - 1).getFirst()) {
                obstacleNeighbors.set(i, obstacleNeighbors.get(i - 1));
                i--;
            }

            obstacleNeighbors.set(i, new Pair<>(distSq, obstacle));
        }
    }

    void update() {
        velocity = newVelocity;
        position = position.add(simRef.timeStep, velocity);
    }


    void setPos(Vector2D newPos) {
        position = newPos;
    }

    private boolean linearProgram1(List<Line> lines, int lineNo, Vector2D optimizationVelocity, boolean optimizeDirection) {
        final double dotProduct = lines.get(lineNo).point.dotProduct(lines.get(lineNo).direction);
        final double discriminant = dotProduct * dotProduct + maxSpeed * maxSpeed - lines.get(lineNo).point.getNormSq();

        if (discriminant < 0.0) {
            return false;
        }

        final double sqrtDiscriminant = FastMath.sqrt(discriminant);
        double tLeft = -sqrtDiscriminant - dotProduct;
        double tRight = sqrtDiscriminant - dotProduct;

        for (int i = 0; i < lineNo; i++) {
            final double denominator = MathUtil.det(lines.get(lineNo).direction, lines.get(i).direction);
            final double numerator = MathUtil.det(lines.get(i).direction, lines.get(lineNo).point.subtract(lines.get(i).point));

            if (FastMath.abs(denominator) <= MathUtil.EPSILON) {
                if (numerator < 0.0) {
                    return false;
                }

                continue;
            }

            final double t = numerator / denominator;

            if (denominator >= 0.0) {
                tRight = FastMath.min(tRight, t);
            } else {
                tLeft = FastMath.max(tLeft, t);
            }

            if (tLeft > tRight) {
                return false;
            }
        }

        if (optimizeDirection) {
            if (optimizationVelocity.dotProduct(lines.get(lineNo).direction) > 0.0) {
                newVelocity = lines.get(lineNo).point.add(tRight, lines.get(lineNo).direction);
            } else {
                newVelocity = lines.get(lineNo).point.add(tLeft, lines.get(lineNo).direction);
            }
        } else {
            final double t = lines.get(lineNo).direction.dotProduct(optimizationVelocity.subtract(lines.get(lineNo).point));

            if (t < tLeft) {
                newVelocity = lines.get(lineNo).point.add(tLeft, lines.get(lineNo).direction);
            } else if (t > tRight) {
                newVelocity = lines.get(lineNo).point.add(tRight, lines.get(lineNo).direction);
            } else {
                newVelocity = lines.get(lineNo).point.add(t, lines.get(lineNo).direction);
            }
        }

        return true;
    }

    private int linearProgram2(List<Line> lines, Vector2D optimizationVelocity, boolean optimizeDirection) {
        if (optimizeDirection) {
            newVelocity = optimizationVelocity.scalarMultiply(maxSpeed);
        } else if (optimizationVelocity.getNormSq() > maxSpeed * maxSpeed) {
            newVelocity = optimizationVelocity.normalize().scalarMultiply(maxSpeed);
        } else {
            newVelocity = optimizationVelocity;
        }

        for (int lineNo = 0; lineNo < lines.size(); lineNo++) {
            if (MathUtil.det(lines.get(lineNo).direction, lines.get(lineNo).point.subtract(newVelocity)) > 0.0) {
                final Vector2D tempResult = newVelocity;
                if (!linearProgram1(lines, lineNo, optimizationVelocity, optimizeDirection)) {
                    newVelocity = tempResult;

                    return lineNo;
                }
            }
        }

        return lines.size();
    }

    private void linearProgram3(int numObstacleLines, int beginLine) {
        double distance = 0.0;

        for (int i = beginLine; i < lines.size(); i++) {
            if (MathUtil.det(lines.get(i).direction, lines.get(i).point.subtract(newVelocity)) > distance) {
                final List<Line> projectedLines = new ArrayList<>(numObstacleLines);
                for (int j = 0; j < numObstacleLines; j++) {
                    projectedLines.add(lines.get(j));
                }

                for (int j = numObstacleLines; j < i; j++) {
                    final double determinant = MathUtil.det(lines.get(i).direction, lines.get(j).direction);
                    final Vector2D point;

                    if (FastMath.abs(determinant) <= MathUtil.EPSILON) {
                        if (lines.get(i).direction.dotProduct(lines.get(j).direction) > 0.0) {
                            continue;
                        }

                        point = lines.get(i).point.add(lines.get(j).point).scalarMultiply(0.5);
                    } else {
                        point = lines.get(i).point.add(lines.get(i).direction.scalarMultiply(MathUtil.det(lines.get(j).direction, lines.get(i).point.subtract(lines.get(j).point)) / determinant));
                    }

                    final Vector2D direction = lines.get(j).direction.subtract(lines.get(i).direction).normalize();
                    projectedLines.add(new Line(point, direction));
                }

                final Vector2D tempResult = newVelocity;
                if (linearProgram2(projectedLines, new Vector2D(-lines.get(i).direction.getY(), lines.get(i).direction.getX()), true) < projectedLines.size()) {
                    newVelocity = tempResult;
                }

                distance = MathUtil.det(lines.get(i).direction, lines.get(i).point.subtract(newVelocity));
            }
        }
    }
}
