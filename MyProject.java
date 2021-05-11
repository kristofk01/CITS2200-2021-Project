// Alexandria Bennett(22969368), Kristof Kovacs (22869854)

import java.util.*;

public class MyProject implements Project {
  /**
   * Checks whether all of the devices in the network are connected using BFS.
   * 
   * @param adjlist the adjacency list of the graph being checked.
   */
  public boolean allDevicesConnected(int[][] adjlist) {

    // Linear time implementation
    // https://courses.cs.vt.edu/~cs4104/murali/Fall09/lectures/lecture-06-linear-time-graph-algorithms.pdf

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
    
    //from last node in other direction (incase 2 cycles?)
    // NOTE(Kris): not 100% sure what u mean, but this causes an infinite loop
    //queue.add(adjlist.length -1);

    while (!queue.isEmpty()) {
      int current = queue.remove();
      for (int vertex : adjlist[current]) {
        if (visited[vertex]) {
          queue.add(vertex);
          visited[vertex] = true;
        }
      }
    }

    for (boolean v : visited)
      if (!v) return false;

    return true;
  }

  /**
   * Computes (using BFS) all possible paths between two vertices in the graph.
   * 
   * @param adjlist the adjacency list of the graph
   * @param src the source vertex
   * @param dst the target vertex
   * 
   * @return the number of paths
   */
  public int numPaths(int[][] adjlist, int src, int dst) {
    // This might be helpful for when we want to make it faster:
    // https://www.cs.princeton.edu/~rs/talks/PathsInGraphs07.pdf
    if (src == dst) { return 1; }

    Queue<Integer> queue = new LinkedList<>();
    boolean[] visited = new boolean[adjlist.length];
    int count = 0;

    queue.add(src);
    visited[src] = true;

    while (!queue.isEmpty()) {
      int current = queue.remove();
      for (int vertex : adjlist[current]) {
        if (vertex == dst) {
          count++;
        }
        else if (!visited[vertex]) {
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

      // Edmonds–Karp algorithm is best complexity
      //https://en.wikipedia.org/wiki/Edmonds%E2%80%93Karp_algorithm

    // make speeds array and just return distance at dst?
    return 0;
    /*
    int maxSpeed = 0;
    int prevSrc = src;
    while(prevSrc != dst){
      for(int i = 0; i < speeds[prevSrc].length; i++){
        int speed1 = speeds[prevSrc][i];
        //which speed in each list is fastest/
        if(speed1 > maxSpeed){
          
        }
        prevSrc = adjlist[prevSrc][i];
        }
    }
    if(src == dst){return -1;}
    return maxSpeed;

    //psuedo codeish
    q = queue();
    int[] parent = int[adjList.length];
    while(!q.isEmpty()){
      current = q.poll()
      for(each edge)
        if(parent[edges] = null and edge != src){
          parent[edge] = edge;
          push edge
        }
          second half lszkhg;kergvb;zbhv
    }
    */
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
