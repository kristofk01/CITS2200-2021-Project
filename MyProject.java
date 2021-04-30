// Alexandria Bennett(22969368), Kristof Kovacs (22869854)

import java.util.*;

public class MyProject implements Project {
  private PriorityQueue<Integer> queue;

  public MyProject () {
    queue = new PriorityQueue<>();
  }

  /**
   * checks whether all of the devices in the network are connected.
   * @param adjList is the adjaceny matrix of the graph being checked.
   */
  public boolean allDevicesConnected(int[][] adjlist) {
    // Note: This is O(n^2), our goal is O(n)
    int size = adjlist.length;
    boolean[] connected = new boolean[size];

    connected[0] = true;    //initial node
   
    for (int i = 0; i < size; i++) {
      for (int j = 0; j < adjlist[i].length; j++) {
        connected[adjlist[i][j]] = true;
      }
    }

    for (boolean x : connected) {
      if (!x) return false;
    }
    return true;
  }

  public int numPaths(int[][] adjlist, int src, int dst) {
    int count = 0;
    boolean[] visited = new boolean[adjlist.length];

    queue.add(src);
    while (!queue.isEmpty()) {
      int current = queue.remove();
      if (current == dst) {
        count++;
      }

      for (int i = 0; i < adjlist[current].length; i++) {
        if (!visited[i]) {
          queue.add(adjlist[current][i]);
          visited[i] = true;
        }
      }
    }

    return count;
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
