// Alexandria Bennett(22969368), Kristof Kovacs (22869854)

import java.util.Array;

public class MyProject implements Project {
  public boolean allDevicesConnected(int[][] adjlist) {
    // Note: should we use Bellman-Ford here instead of BFS?
    int size = adjList.length;
    boolean[] connected = new boolean[size];
    connected[0] = true;
   
    for (int i = 0; i < size; i++) {
      if(!connected[i]) {
        int size2 = adjList[i].length;
        for (int j = 0; j < size2; j++) {
          adjList[i][j] = connected;
        }
      }
    }

    for(int x : connected) {
      if(!x) {return false;}
    }
    return true;
  }

  public int numPaths(int[][] adjlist, int src, int dst) {
    // TODO
    return 0;
  }

  public int[] closestInSubnet(int[][] adjlist, short[][] addrs, int src, short[][] queries) {
    // TODO
    return null;
  }

  public int maxDownloadSpeed(int[][] adjlist, int[][] speeds, int src, int dst) {
    // TODO
    return 0;
  }
}
