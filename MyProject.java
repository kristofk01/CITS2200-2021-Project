// Alexandria Bennett(22969368), Kristof Kovacs (22869854)

import java.util.*;

public class MyProject implements Project {
  /**
   * Checks whether all of the devices in the network are connected using BFS.
   * 
   * @param adjList the adjacency matrix of the graph being checked.
   */
  public boolean allDevicesConnected(int[][] adjlist) {
    Queue<Integer> queue = new LinkedList<>();
    boolean[] visited = new boolean[adjlist.length];

    // An arbitrary starting vertex
    queue.add(0);
    visited[0] = true;

    while (!queue.isEmpty()) {
      int current = queue.remove();
      for (int vertex : adjlist[current]) {
        if (!visited[vertex]) {
          queue.add(vertex);
          visited[vertex] = true;
        }
      }
    }

    for (boolean v : visited)
      if (!v) return false;

    return true;
  }

  /* 
   * NOTE: fails when there are two (or possibly more) paths between the
   * source and destination nodes.
   * 
   * This might be helpful for when we want to make it faster:
   * https://www.cs.princeton.edu/~rs/talks/PathsInGraphs07.pdf
   */
  public int numPaths(int[][] adjlist, int src, int dst) {
    Queue<Integer> queue = new LinkedList<>();
    boolean[] visited = new boolean[adjlist.length];
    int count = 0;

    queue.add(src);
    visited[src] = true;

    while (!queue.isEmpty()) {
      int current = queue.remove();
      if (current == dst)
        count++;

      for (int vertex : adjlist[current]) {
        if (!visited[vertex]) {
          queue.add(vertex);
          visited[vertex] = true;
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
