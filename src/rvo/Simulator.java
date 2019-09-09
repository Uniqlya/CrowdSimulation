package rvo;

import app.RDScene;
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import java.util.ArrayList;
import java.util.List;

public class Simulator {
    final List<Agent> oriAgents = new ArrayList<>();
    final List<Obstacle> obstacles = new ArrayList<>();
    final KdTree kdTree = new KdTree(this);

    List<List<Vector2D>> truePos, trueVel;
    Agent defaultAgent = null;
    double globalTime = 0;
    double timeStep = 0;

    List<Agent> agents = new ArrayList<>();

    public Simulator() { }
    public Simulator(Simulator oriSim) {
        for (Agent a : oriSim.oriAgents)
            oriAgents.add(new Agent(a, this));

        obstacles.addAll(oriSim.obstacles);
        truePos = oriSim.truePos;
        trueVel = oriSim.trueVel;

        timeStep = oriSim.timeStep;
        processObstacles();
    }


    public List<Agent> getOriAgents() { return oriAgents; }
    public List<Agent> getAgents() { return agents; }
    public int getNumAgents() { return agents.size(); }

    public double[] getAllPara() {
        double[] res = new double[oriAgents.size()*6];
        int st = 0;
        for (Agent a : oriAgents) {
            res[st] = a.radius;
            res[st+1] = a.neighborDistance;
            res[st+2] = a.maxNeighbors;
            res[st+3] = a.timeHorizonAgents;
            res[st+4] = a.timeHorizonObstacles;
            res[st+5] = a.prefSpeed;
            st = st+6;
        }
        return res;
    }

    public void setTimeStep(double timeStep) { this.timeStep = timeStep; }

    public void setTruePosThenCalcVel(List<List<Vector2D>> truePos) {
        this.truePos = truePos;
        int frames = truePos.size();
        int numPeople = truePos.get(0).size();
        double invTimestep = 1.0 / timeStep;

        // 某一帧的真实速度应该是上一帧到这一帧的速度
        trueVel = new ArrayList<>(frames);

        // 所有第一帧的速度都是 0
        List<Vector2D> initVel = new ArrayList<>();
        for (int i = 0; i < numPeople; i++)
            initVel.add(Vector2D.ZERO);
        trueVel.add(initVel);

        // 计算其余的真实速度
        for (int i = 1; i < frames; i++) {
            List<Vector2D> prevPos = truePos.get(i - 1);
            List<Vector2D> curPos = truePos.get(i);
            List<Vector2D> curVel = new ArrayList<>();
            for (int j = 0; j < numPeople; j++) {
                Vector2D v = curPos.get(j).subtract(prevPos.get(j)).scalarMultiply(invTimestep);
                curVel.add(v);
            }
            trueVel.add(curVel);
        }
    }

    public void addAgent(int ID, Vector2D position, Vector2D goal,
                         double neighborDistance, int maxNeighbors,
                         double timeHorizonAgents, double timeHorizonObstacles,
                         double radius, double prefSpeed, double maxSpeed,
                         int start, int end, Vector2D velocity) {
        Agent agent = new Agent(this);
        agent.id = ID;
        agent.maxNeighbors = maxNeighbors;
        agent.prefSpeed = prefSpeed;
        agent.maxSpeed = maxSpeed;
        agent.neighborDistance = neighborDistance;
        agent.position = position;
        agent.goal = goal;
        agent.radius = radius;
        agent.timeHorizonAgents = timeHorizonAgents;
        agent.timeHorizonObstacles = timeHorizonObstacles;
        agent.velocity = velocity;
        agent.start = start;
        agent.end = end;
        agents.add(agent);
        oriAgents.add(agent);
    }

    public int addObstacle(List<Vector2D> vertices) {
        int numVertices = vertices.size();
        if (numVertices < 2) {
            return -1;
        }

        int obstacleId = obstacles.size();

        for (int vertexId = 0; vertexId < numVertices; vertexId++) {
            final Obstacle obstacle = new Obstacle();
            obstacle.point = vertices.get(vertexId);

            if (vertexId != 0) {
                obstacle.previous = obstacles.get(obstacles.size() - 1);
                obstacle.previous.next = obstacle;
            }

            if (vertexId == vertices.size() - 1) {
                obstacle.next = obstacles.get(obstacleId);
                obstacle.next.previous = obstacle;
            }

            int nextVertId = (vertexId+1) % numVertices;
            int prevVertId = vertexId==0 ? numVertices-1 : vertexId-1;
            obstacle.direction = vertices.get(nextVertId)
                    .subtract(vertices.get(vertexId))
                    .normalize();

            obstacle.convex = vertices.size() == 2 || MathUtil.leftOf(
                    vertices.get(prevVertId),
                    vertices.get(vertexId),
                    vertices.get(nextVertId)) >= 0.0;

            obstacle.id = obstacles.size();
            obstacles.add(obstacle);
        }

        return obstacleId;
    }

    public void processObstacles() { kdTree.buildObstacleTree(); }

    public double doStepWithGroup(int stFrame, int edFrame, List<Integer> memIDs) {
        double diff = 0;

        for (int f = stFrame; f < edFrame; f++) {
            // 只把出现的且未到达终点的放入 RVO 模拟
            agents = new ArrayList<>();
            for (Agent a : oriAgents) {
                if (a.canShowUp(f) && !a.reachedGoal())
                    agents.add(a);
            }

            // 阶段1：设置偏好速度
            for (Agent a : agents)
                a.setPreferredVelocity();

            // 阶段2：构建kd树以便查找近邻
            kdTree.buildAgentTree();

            // 阶段3：感知近邻并决策一个新速度
            for (int i = 0; i < agents.size(); i++) {
                final Agent agent = agents.get(i);
                agent.computeNeighbors();
                agent.computeNewVelocity();
            }

            // 阶段4：更新位置和速度
            for (Agent a : agents)
                a.update();

            // 更新位置后，要和真实的下一帧比较，所以是 f+1
            List<Vector2D> curStepTruePos = truePos.get(f+1);
            for (int i = 0; i < memIDs.size(); i++) {
                int id = memIDs.get(i);
                double t = curStepTruePos.get(id).distance(oriAgents.get(id).getPosition());
                diff += t;
            }
        }
        return diff;
    }

    public double doStep(int stFrame, int edFrame) {
        double diff = 0;

        for (int f = stFrame; f < edFrame; f++) {
            // 只把出现的且未到达终点的放入 RVO 模拟
            agents = new ArrayList<>();
            for (Agent a : oriAgents) {
                if (a.canShowUp(f) && !a.reachedGoal())
                    agents.add(a);
            }

            // 阶段1：设置偏好速度
            for (Agent a : agents)
                a.setPreferredVelocity();

            // 阶段2：构建kd树以便查找近邻
            kdTree.buildAgentTree();

            // 阶段3：感知近邻并决策一个新速度
            for (int i = 0; i < agents.size(); i++) {
                final Agent agent = agents.get(i);
                agent.computeNeighbors();
                agent.computeNewVelocity();
            }


            // 更新位置，同时和真实的下一帧比较，所以是 f+1
            List<Vector2D> curStepTruePos = truePos.get(f+1);
            List<Double> truePosDiff = RDScene.allTruePosDiff.get(f);
            for (Agent a : agents) {
                int id = a.getId();
                a.update();
                a.calcDiff(curStepTruePos.get(id), truePosDiff.get(id));
            }
        }

        int people = 0;
        for (Agent a : oriAgents)
            if (a.goFrames > 0) {
                people++;
                diff += (a.diff / a.goFrames);
            }

        return diff / people;
    }

    public void resetTrue(int frame) {
        // 重新设置新状态，包括：position, velocity
        List<Vector2D> curTrue = truePos.get(frame);
        List<Vector2D> curVel = trueVel.get(frame);
        for (int i = 0; i < oriAgents.size(); i++) {
            Agent a = oriAgents.get(i);
            a.position = curTrue.get(i);
            a.velocity = curVel.get(i);
            a.diff = 0;
            a.goFrames = 0;
        }
    }
}
