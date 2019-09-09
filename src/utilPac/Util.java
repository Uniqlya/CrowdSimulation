package utilPac;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;

import java.io.*;
import java.lang.reflect.Method;
import java.nio.charset.Charset;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Vector;
import java.util.zip.DeflaterOutputStream;

public class Util {
    public static final String dir = "C:\\Users\\aly\\Documents\\Data\\";


    public static List<String> getlines(String fname) {
        List<String> res = new ArrayList<>();
        try {
            File f = new File(fname);
            BufferedReader br = new BufferedReader(new FileReader(fname));
            String line = null;
            while ((line = br.readLine()) != null)
                res.add(line);
            br.close();
        } catch (Exception e) {
            e.printStackTrace();
        }
        return res;
    }


    public static <T> List<T> parseLine(String line, String dimRegex, Class<T> type) {
        try {
            Method parse = type.getMethod("valueOf", String.class);
            List<T> res = new ArrayList<>();
            for (String s : line.split(dimRegex)) {
                res.add((T)parse.invoke(null, s));
            }
            return res;
        } catch (Exception e) {
            e.printStackTrace();
            return null;
        }
    }

    public static void matToFile(String fname, List<List<Double>> mat) {
        try {
            PrintWriter pw = new PrintWriter(fname);
            StringBuilder sb = new StringBuilder();
            for (List<Double> l : mat) {
                for (Double d : l)
                    sb.append(d).append(" ");
                sb.append('\n');
            }
            pw.print(sb.toString());
            pw.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public static double[][] labmatFromFile(String fname) {
        double[][] res = null;
        try {
            List<String> lines = Files.readAllLines(Paths.get(fname), Charset.defaultCharset());
            res = new double[lines.size()][];
            for (int j = 0; j < lines.size(); j++) {
                String[] s = lines.get(j).split(",");
                double[] t = new double[s.length];

                for (int i = 0; i < s.length; i++)
                    t[i] = Double.parseDouble(s[i]);
                res[j] = t;
            }
        } catch (Exception e) {
            e.printStackTrace();
        }

        return res;
    }

    public static List<Double> toList(double[] in) {
        List<Double> res = new ArrayList<>();
        for (double d : in)
            res.add(d);
        return res;
    }

    public static double sumDiff(double[] a, double[] b) {
        assert(a.length == b.length);
        double res = 0;
        for (int i = 0; i < a.length; i++) {
            res += (a[i] - b[i]);
        }
        return res;
    }

    public static List<List<Double>> matFromFile(String fname) {
        List<List<Double>> res = new ArrayList<>();
        try {
            List<String> lines = Files.readAllLines(Paths.get(fname), Charset.defaultCharset());
            for (String l : lines) {
                List<Double> t = new ArrayList<>();
                String[] s = l.split("\\s+");

                for (int i = 0; i < s.length; i++)
                    t.add(Double.parseDouble(s[i]));
                res.add(t);
            }
        } catch (Exception e) {
            e.printStackTrace();
        }

        return res;
    }
    public static List<int[][]> groupsFromFile(String fname) {
        List<int[][]> res = new ArrayList<>();
        try {
            List<String> lines = Files.readAllLines(Paths.get(fname), Charset.defaultCharset());
            int stFrame = 0;
            List<int[]> cg = new ArrayList<>();
            for (String l : lines) {
                String[] str = l.split("\\s+");
                int[] t = new int[str.length-2];    // 减掉帧数和follower数量信息
                int lineFrame = Integer.parseInt(str[0]);
                t[0] = Integer.parseInt(str[1]);
                if (t[0] == -1)                 // 去掉 Dead Group
                    continue;

                for (int i = 0; i < str.length - 3; i++)
                    t[i+1] = Integer.parseInt(str[i+3]);

                if (stFrame == lineFrame)
                    cg.add(t);
                else {
                    res.add(cg.toArray(new int[cg.size()][]));
                    cg = new ArrayList<>();
                    cg.add(t);
                    stFrame = lineFrame;
                }
            }
            res.add(cg.toArray(new int[cg.size()][]));
        } catch (Exception e) {
            e.printStackTrace();
        }

        return res;
    }


    public static void vecToFile(String fname, List<Double> data) {
        try {
            PrintWriter pw = new PrintWriter(fname);
            for (Double d : data)
                pw.println(d);
            pw.close();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public static List<Integer> intVecFromFile(String fname) {
        List<Integer> res = new ArrayList<>();
        List<String> lines = getlines(fname);
        for (String s : lines)
            res.add(Integer.parseInt(s));
        return res;
    }

    public static void posToFile(String fname, List<List<Vector2D>> pos) {
        try {
            PrintWriter pw = new PrintWriter(fname);
            StringBuilder sb = new StringBuilder();
            for (List<Vector2D> cpos : pos) {
                for (Vector2D p : cpos)
                    sb.append(p.getX()).append(" ").append(p.getY()).append(" ");
                sb.append('\n');
            }
            pw.print(sb.toString());
            pw.close();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }


    public static List<List<Vector2D>> posFromFile(String fname) {
        List<List<Vector2D>> res = new ArrayList<>();
        try {
            List<String> lines = Files.readAllLines(Paths.get(fname), Charset.defaultCharset());
            for (String l : lines) {
                List<Vector2D> t = new ArrayList<>();
                String[] s = l.split("\\s+");

                assert (s.length % 2 == 0);
                for (int i = 0; i < s.length; i+=2)
                    t.add(new Vector2D(Double.parseDouble(s[i]), Double.parseDouble(s[i+1])));
                res.add(t);
            }
        } catch (Exception e) {
            e.printStackTrace();
        }

        return res;
    }


    public static void calcPosDiff() {
        List<List<Vector2D>> allTruePos = posFromFile(dir+"DML2\\allTruePos.txt");

        List<List<Double>> res = new ArrayList<>();
        for (int i = 0; i < allTruePos.size() - 1; i++) {
            List<Vector2D> cur = allTruePos.get(i);
            List<Vector2D> next = allTruePos.get(i+1);
            List<Double> rt = new ArrayList<>();

            for (int j = 0; j < cur.size(); j++)
                rt.add(next.get(j).distance(cur.get(j)));
            res.add(rt);
        }

        matToFile(dir+"DML2\\allTruePosDiff.txt", res);
    }


    public static double[][][] splitByD(double[][] paras, int b) {
        int be = (int)Math.ceil(paras.length / (double)b);
        int rb = (int)Math.ceil(paras.length / (double)be);
        double[][][] res = new double[rb][][];

        int st = 0;
        for (int i = 0; i < rb; i++) {
            int rest = (paras.length-st) >= be ? be : (paras.length-st);
            res[i] = new double[rest][];
            for (int j = 0; j < rest; j++) {
                res[i][j] = paras[st++];
            }
        }
        return res;
    }


    public static void main(String[] args) {
        calcPosDiff();
    }
}
