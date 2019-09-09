package misc;

import utilPac.Util;

import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;

public class Group {
    public int st, ed;
    private List<Integer> memberIDs;

    Group(List<Integer> memIDs, int st, int ed) {
        this.memberIDs = memIDs;
        this.st = st;
        this.ed = ed;
    }

    public static List<Group> groupsFromFile(String groupFile) {
        try {
            List<Group> res = new ArrayList<>();
            List<String> lines = Files.readAllLines(Paths.get(groupFile));
            for (String l : lines) {
                int t = l.indexOf("]");
                List<Integer> mems = Util.parseLine(l.substring(1, t).trim(), "[^0-9]+", Integer.class);
                List<Integer> time = Util.parseLine(l.substring(t+1).trim(), "\\s+", Integer.class);

                Group g = new Group(mems, time.get(0), time.get(1));
                res.add(g);
            }
            return res;
        } catch (Exception e) {
            e.printStackTrace();
            return null;
        }
    }


    public List<Integer> getAllMembers() {
        return memberIDs;
    }

    public int[] allMembers2Matlab() {
        List<Integer> mem = getAllMembers();
        int numMember = mem.size();
        int[] t = new int[numMember * 6];
        for (int j = 0; j < numMember; j++)
            for (int k = 0; k < 6; k++) {
                t[j * 6 + k] = mem.get(j) * 6 + k + 1; // Matlab 索引必须加一
            }
        return t;
    }
}
