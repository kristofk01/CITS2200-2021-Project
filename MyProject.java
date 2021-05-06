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
        if (visited[vertex]) continue{
          queue.add(vertex);
          visited[vertex] = true;
        }
      }
    }

    return count;
  }

  public int[] closestInSubnet(int[][] adjlist, short[][] addrs, int src, short[][] queries) {
    //Dijkstra's algorithm with binary heap has the complexity we need (wikipedia lmao)
    /*
     *addrs[i][] = an ip adress of 4 vals from 1-255 for device i
     *queries[i][] = ip address prefix? eg for {198, 34, 1, 1} it would be {198, 34} but not {198, 34, 2}
     *probably going to want to take the address from the source and go to eaach node 1 index at a time
     *eg) 
     *get.addrs[src]. for each query, is queries[i][0] = addrs[src][0] if so move on to queries[i][1] etc
     *i think the queries arrays will not be the same length as addrs so that will need to be checked. 
     *returning the number of hops to get to a subnet (from queries)
     */
    return null;
  }

  public int maxDownloadSpeed(int[][] adjlist, int[][] speeds, int src, int dst) {
   //Bellman Ford or Flyod Warshall depending on the complexity (VE vs V^3)
    /* speeds[i][j] relates to the device at adjList[i][j]
     * instead of smallest path, we need biggest path eg MAX speed
     * can travel multiple paths at once and can be asymetric down a link -> check both ways?
     *if (src == dst){return -1}
     *dijkstra again (or similar) to grab the max total speed from the source to destination.
     */
    return 0;
  }
}
