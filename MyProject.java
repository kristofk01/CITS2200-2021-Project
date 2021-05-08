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
        if (visited[vertex]) {
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

    
    // This stack should contain all devices in the specified subnet
    //Stack<Integer> stack = new Stack<>();
    
    int deviceCount = adjlist.length;

    ///////////////////
    // Might not need a stack, we just set the devices that aren't in the
    // subnet to visited=true so that Dijkstra ignores them from the start.
    boolean[] visited = new boolean[deviceCount];
    ///////////////////
    
    // NOTE: probably broken asf
    for (int i = 0; i < deviceCount; i++) {
      //int device = adjlist[i][0];
      short[] device_address = addrs[i];

      // The device is in the subnet
      boolean isInSubnet = true;
      for (int j = 0; j < queries[i].length; j++) {
        short[] subnet = queries[i];
        // Not in subnet
        if (subnet[j] < device_address[j]) {
          isInSubnet = false;
          break;
        }
      }
      if (!isInSubnet) {
        visited[i] = true;
      } // else { stack.push(device); }
    }
    
    //key returns null :/

    // Run Dijkstra's on the devices in the subnet
    PriorityQueue<Node> queue = new PriorityQueue<>();
    int[] key = new int[deviceCount];
    Arrays.fill(key, -1);

    key[src] = 0;
    queue.add(new Node(src, key[src]));

    while (!queue.isEmpty()) {
      Node current = queue.remove();
      if (!visited[current.vertex]) {
        visited[current.vertex] = true;
        key[current.vertex] = current.priority;

        // deviceCount is wrong, java.lang.ArrayIndexOutOfBoundsException for i
        for (int i = 0; i < deviceCount; i++) {
          if (!visited[i] && adjlist[current.vertex][i] >= 1) {
            queue.remove();
            queue.add(new Node(i, adjlist[current.vertex][i] + current.priority));
          }
        }
      }
    }

    return key;
  }

  
  public int maxDownloadSpeed(int[][] adjlist, int[][] speeds, int src, int dst) {
   //Bellman Ford or Floyd Warshall depending on the complexity (VE vs V^3)
    /* speeds[i][j] relates to the device at adjList[i][j]
     * instead of smallest path, we need biggest path eg MAX speed
     * can travel multiple paths at once and can be asymetric down a link -> check both ways?
     *if (src == dst){return -1}
     *dijkstra again (or similar) to grab the max total speed from the source to destination.
     */


    // completely different, just writing down psuedo code for floyd warshall
     // NEED TO MAKE A SECOND MATRIX CALLED prevSpeed SO WE CAN DO THIS
     int[][] prevSpeed = new int[speeds.length][speeds.length];

     for(k = 1; k < length; k++){

     for(int i = 0; i < length; i++){
      for(int j = 0; j < length; j++){
        maxSpeed = max/*?*/(prev max, prevspeed through k vals added.)
      }
     }
    }


     if(/*i maybe? */ == dst){return maxSpeed;}
  }


    //maximuum value method. dont know if needed.
    private int maxVal(int a, int b){
      if(a > b) {return a}
      if(b > a){return b}
    }


  /**
   * Inner-class that allows for the priority queue to store
   * a given vertex with a priority.
   */
  private class Node implements Comparable<Node> {
    int vertex, priority;

    Node (int vertex, int priority) {
      this.vertex = vertex;
      this.priority = priority;
    }

    public int compareTo (Node other) {
      return Integer.compare(this.priority, other.priority);
    }
  }
}
