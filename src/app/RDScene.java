package app;

import misc.Group;
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.NodeList;
import rvo.Simulator;
import rvo.Agent;
import utilPac.Util;

import javax.xml.parsers.DocumentBuilderFactory;
import java.io.File;
import java.util.*;
import java.util.List;
import java.util.concurrent.*;

public class RDScene {
    int step = 0;
    int testStep = 0;
    int edFrame = 0;
    int numThreads;

    private int numAgents;
    List<Agent> allAgents;
    public static final Simulator OriSim = new Simulator();
    public static String dataDir, sceneFile, groupFile;
    public static List<List<Double>> allTruePosDiff;
    List<Group> allGroups;
    List<List<Vector2D>> allTruePos;
    boolean inited;
    double[][] mlabparas;
    ExecutorService es;
    ExecutorCompletionService service;



    public RDScene(int edFrame, String dataDir, String groupFile, int numThreads) {
        this.edFrame = edFrame;
        if (dataDir.contains("DML3"))
            sceneFile = dataDir+"DML3.xml";
        else
            sceneFile = dataDir + "DML2.xml";

        RDScene.dataDir = dataDir;
        RDScene.groupFile = groupFile;

        this.numThreads = numThreads;
        this.es = Executors.newFixedThreadPool(numThreads);
        this.service = new ExecutorCompletionService(this.es);
    }

    public int getTestStep() {
        return testStep;
    }

    public void setupScene() {
        allTruePosDiff = Util.matFromFile(dataDir +"allTruePosDiff.txt");

        OriSim.setTimeStep(0.1);

        // 设置真实数据
        if (groupFile.length() > 0)
            allGroups = Group.groupsFromFile(dataDir +groupFile);
        allTruePos = Util.posFromFile(dataDir +"allTruePos.txt");
        OriSim.setTruePosThenCalcVel(allTruePos);

        // 添加 agent（初始位置、设定目标）
        // 添加障碍物
        try {
            // 行人起始结束的对应帧数
            List<List<Double>> stEd = Util.matFromFile(dataDir +"startEnd.txt");

            Document doc = DocumentBuilderFactory.newInstance().newDocumentBuilder()
                    .parse(new File(sceneFile));

            NodeList xmlAgentList = doc.getElementsByTagName("crowd");
            NodeList xmlObstList = doc.getElementsByTagName("obstacles");

            for (int i = 0; i < xmlAgentList.getLength(); i++) {
                Element agent = (Element)xmlAgentList.item(i);
                Element posXML = (Element)agent.getElementsByTagName("position").item(0);
                Element goalXML = (Element)agent.getElementsByTagName("goal").item(0);

                Vector2D pos = new Vector2D(
                        Double.parseDouble(posXML.getAttribute("x")),
                        Double.parseDouble(posXML.getAttribute("y"))
                );

                Vector2D goal = new Vector2D(
                        Double.parseDouble(goalXML.getAttribute("x")),
                        Double.parseDouble(goalXML.getAttribute("y"))
                );

                int ID = Integer.parseInt(agent.getElementsByTagName("id").item(0).getTextContent());
                double prefSpeed = Double.parseDouble(agent.getElementsByTagName("preferedSpeed").item(0).getTextContent());

                int start = stEd.get(ID).get(0).intValue();
                int end = stEd.get(ID).get(1).intValue();

                prefSpeed = 1.3;

                OriSim.addAgent(ID, pos, goal, 20, 10, 0.5, 0.5, 0.2, prefSpeed, prefSpeed*2.0, start, end, Vector2D.ZERO);
            }

            numAgents = OriSim.getNumAgents();
            allAgents = OriSim.getOriAgents();

            // 从xml读取障碍物列表，注意障碍物必须是逆时针的
            for (int i = 0; i < xmlObstList.getLength(); i++) {
                Element obs = (Element) xmlObstList.item(i);
                final List<Vector2D> obst = new ArrayList<>();

                for (int j = 3; j >= 0; j--) {
                    Element vert = (Element) obs.getElementsByTagName("vertices").item(j);
                    double vertX = Double.parseDouble(vert.getAttribute("x"));
                    double vertY = Double.parseDouble(vert.getAttribute("y"));
                    obst.add(new Vector2D(vertX, vertY));
                }

                OriSim.addObstacle(obst);
            }

            OriSim.processObstacles();

        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public int numReachedGoal() {
        int n = 0;
        for (Agent a : OriSim.getAgents())
            if (a.reachedGoal())
                n++;
        return n;
    }

    public double defFeval(int stFrame, int edFrame) {
        OriSim.resetTrue(stFrame);
        return OriSim.doStep(stFrame, edFrame);
    }


    // ============================ 供 Matlab 调用的 API ============================
    public double[] getAllPara() {
        return OriSim.getAllPara();
    }

    public int getTotalFrames() {
        return allTruePos.size();
    }

    public int[] getAlive() {
        int numStep = allTruePos.size() - 1;
        int[] res = new int[numStep];
        for (int i = 0; i < numStep; i++) {
            int[][] g = null;
            for (int j = 0; j < g.length; j++)
                res[i] += g[j].length;
        }
        return res;
    }

    public void setPara(double[] para) {
        for (int j = 0; j < numAgents; j++) {
            int st = j * 6;
            allAgents.get(j).setPara(para[st], para[st + 1], para[st + 2], para[st + 3], para[st + 4], para[st + 5]);
        }
    }

    public double[] evalPartial(double[][] paras, int start, int end) {
        double[] res = new double[paras.length];

        for (int i = 0; i < paras.length; i++) {
            // Set para
            double[] para = paras[i];
            for (int j = 0; j < numAgents; j++) {
                int st = j * 6;
                allAgents.get(j).setPara(para[st], para[st + 1], para[st + 2], para[st + 3], para[st + 4], para[st + 5]);
            }

            OriSim.resetTrue(start);
            res[i] = OriSim.doStep(start, end);
        }
        return res;
    }

    public double[] evalPartialP(double[][] paras, final int start, final int end) {
        final double[] res = new double[paras.length];

        double[][][] dividedPara = Util.splitByD(paras, numThreads);
        int rb = dividedPara.length;
        int be = dividedPara[0].length;

        for (int i = 0; i < rb; i++) {
            final double[][] pslice = dividedPara[i];
            final int st = i * be;

            Callable<Void> ind = new Callable<Void>() {
                @Override
                public Void call() throws Exception {
                    Simulator sim = new Simulator(OriSim);
                    List<Agent> simOriAgents = sim.getOriAgents();

                    for (int k = 0; k < pslice.length; k++) {
                        double[] p = pslice[k];
                        for (int m = 0; m < numAgents; m++) {
                            int st = m * 6;
                            simOriAgents.get(m).setPara(p[st], p[st + 1], p[st + 2], p[st + 3], p[st + 4], p[st + 5]);
                        }

                        sim.resetTrue(start);
                        res[st+k] = sim.doStep(start, end);
                    }
                    return null;
                }
            };
            service.submit(ind);
        }

        try {
            for (int i = 0; i < rb; i++)
                service.take().get();

            return res;
        } catch (Exception e) {
            e.printStackTrace();
            return null;
        }
    }

    public void destroyExecutor() {
        es.shutdownNow();
    }

    public double[] FevalResetInteval(double[] para, int interval) {
        for (int j = 0; j < numAgents; j++) {
            int st = j * 6;
            allAgents.get(j).setPara(para[st], para[st + 1], para[st + 2], para[st + 3], para[st + 4], para[st + 5]);
        }

        int totalStep = allTruePos.size() - 1;
        int times = totalStep / interval;
        int rem = totalStep % interval == 0 ? 0 : 1;
        double[] res = new double[times + rem];
        for (int i = 0; i < times; i++) {
            OriSim.resetTrue(i);
            res[i] = OriSim.doStep(i, i+interval);
        }
        return res;
    }

// =====================   18.11.22 新增 ================
    public int getGroupSt(int g) { return allGroups.get(g).st; }
    public int getGroupEd(int g) { return allGroups.get(g).ed; }
    public int[] getGroupMemIdx(int g) { return allGroups.get(g).allMembers2Matlab(); }
    public int getNumGroups() { return allGroups.size(); }
    public int getNumAgents() { return numAgents; }

    public double step() {
        if (step == allTruePos.size()-1)
            return -1;
        double diff = OriSim.doStep(step, step+1);
        step++;
        return diff;
    }

    public double stepTrue() {
        if (step == allTruePos.size()-1)
            return -1;
        step++;
        List<Vector2D> curTrue = allTruePos.get(step);
        for (int i = 0; i < allAgents.size(); i++)
            allAgents.get(i).setPosition(curTrue.get(i));
        return 0;
    }

    public List<Agent> getAliveAgents(int s) {
        List<Agent> res = new ArrayList<>();
        for (Agent a : allAgents)
            if (a.canShowUp(s) && !a.reachedGoal())
                res.add(a);
        return res;
    }

    public int[] getNumAlive() {
        OriSim.resetTrue(0);
        step = 0;
        int[] res = new int[allTruePos.size()-1];
        for (int i = 0; i < allTruePos.size() - 1; i++) {
            for (Agent a : allAgents)
                if (a.canShowUp(step) && !a.reachedGoal())
                    res[i] += 1;
            stepTrue();
        }
        return res;
    }

    public static void main(String[] args) {
        double[][] popR = Util.labmatFromFile("predParas.txt");

        int st = 600;
        int end = 900;
        RDScene rd = new RDScene(end, "DML2/", "groupF/lfmodeInfo300.txt", 4);
        rd.setupScene();

        double rest = rd.defFeval(st, end);
        System.out.println(rest);

        double[] res = rd.evalPartial(popR, st, end);
        System.out.println(res[0]);

        rd.destroyExecutor();
    }
}