//----------------------------------------------------------------------
// NAME: S. Tornquist
// FILE: graph.cpp
// DATE: Spring 2021
// DESC: implementation of functions from graph.h
//----------------------------------------------------------------------

#ifndef GRAPH_CPP
#define GRAPH_CPP

#include "graph.h"
#include <list>
#include <queue>
#include <stack>
#include <vector>
#include <algorithm>
#include <set>

  //----------------------------------------------------------------------
  // HW-3 graph operations
  //----------------------------------------------------------------------
  
  //----------------------------------------------------------------------
  // Breadth-first search from a given source vertex. 
  //
  // Inputs: 
  //   dir -- true if directed
  //   src -- the source vertex
  // Outputs:
  //   tree -- search tree that maps vertices found during bfs from the
  //           source to their parent vertices
  //---------------------------------------------------------------------
void Graph::bfs(bool dir, int src, Map& tree) const{
     tree.clear();

     // marking all vertices in v as unexplored
     bool discovered[vertex_count()] = {0};

     //mark S as discovered
     discovered[src] = true;

     //setting parent of S to -1 to denote root
     tree[src] = -1;

     //creating Queue data structure intialized with S
     std::queue<int> Q;
     Q.push(src);
     
     //while the Q is not empty
     while(Q.size() > 0){
          int u = Q.front(); // getting the top item from the Q
          Q.pop();//removing u from he Q
          std::list<int> vertices;
          if(dir == true){ //if directed graph
               connected_to(u, vertices); // getting all vertices u goes to
          }
          if(dir == false){ //if undirected graph
               adjacent(u, vertices); // getting all vertices u goes to
          }
          while(vertices.size() > 0){//for all v adjacent to u
               int v = vertices.front(); //setting v
               vertices.pop_front();//removing v from the list
               if(discovered[v] == false){ // if v is undiscovered
                    discovered[v] = true;//set v to discovered
                    tree[v] = u; //setting v's parent to u
                    Q.push(v); //adding v to the end of Q
               }
               
          }
     }
}
  
  //----------------------------------------------------------------------
  // Shortest path length from source to destination vertices.
  // Conditions:
  //   The source and destination vertices must be unique.  
  // Inputs:
  //   dir -- true if directed
  //   src -- the vertex starting the path
  //   dst -- the vertex ending the path
  // Outputs:
  //   path -- sequence of nodes that define the shortest path
  //----------------------------------------------------------------------
void Graph::shortest_path_length(bool dir, int src, int dst, std::list<int>& path) const{
     if((0 <= dir) && (dir < vertex_count()) && (0 <= src) && (src < vertex_count())){ // making sure dir and src are valid vertices
          std::unordered_map<int,int> m;//intializing map
          bfs(dir,src,m);//breadth-first search from the source vertex
          if(m.count(dst) == 0){//if the destination is not in the returned tree, ie there is not a path
               path.clear(); //returning empty path
               return;
          }
          int search = dst; //setting search to the destination vertex parent
          path.push_front(dst); // adding destination vertex to the path
          while(search != src){ // iterating until the search finds the source vertex
               search = m[search];//setting search to it's parent to trace back through the tree
               path.push_front(search); // adding  vertex to the path
          }
     }
}

  //----------------------------------------------------------------------
  // Find connected components based on breadth-first search.
  //
  // Conditions:
  //   Finds strongly connected components in an undirected graph and
  //   weakly-connected components in a directed graph.
  // Inputs:
  //   None
  // Outputs: 
  //   components -- mapping from each graph vertex to its corresponding
  //                 component number where component numbers range from
  //                 0 to c-1 (for c components)
  //----------------------------------------------------------------------
void Graph::bfs_connected_components(Map& components) const{
     components.clear();

     int component= 0; //component count

     // marking all vertices in v as unexplored
     bool discovered[vertex_count()] = {0};

     int last_undiscovered=0;//last unfound node;

     while(last_undiscovered < vertex_count()){
          //mark S as discovered
          discovered[last_undiscovered] = true;
          components[last_undiscovered] = component;

          //creating Queue data structure intialized with S
          std::queue<int> Q;
          Q.push(last_undiscovered);


          //while the Q is not empty
          while(Q.size() > 0){
               int u = Q.front(); // getting the top item from the Q
               Q.pop();//removing u from he Q
               std::list<int> vertices;
               adjacent(u, vertices); // getting all connected vertices adjacent, wealky connected for
               while(vertices.size() > 0){//for all v adjacent to u
                    int v = vertices.front(); //setting v
                    vertices.pop_front();//removing v from the list
                    if(discovered[v] == false){ // if v is undiscovered
                         discovered[v] = true;//set v to discovered
                         components[v] = component; //setting v's parent to component
                         Q.push(v); //adding v to the end of Q
                    }

               }
          }
          component++;//new component
          while(discovered[last_undiscovered] == true){//finding the next component
               last_undiscovered++;
               if(last_undiscovered >= vertex_count()){ //if we have gone over the vertex count, break
                    break;
               }
          }
     }

}

  //----------------------------------------------------------------------
  // Determine if the graph is bipartite (i.e., 2-colorable)
  //
  // Inputs:
  //   None
  // Outputs:
  //   returns  -- true if the graph is bipartite, false otherwise
  //   coloring -- mapping from each graph vertex to its corresponding
  //               color (either 0 or 1) if graph is bipartite
  //----------------------------------------------------------------------
bool Graph::bipartite_graph(Map& coloring) const{
     coloring.clear();

     // marking all vertices in v as unexplored
     bool discovered[vertex_count()] = {0};

     int last_undiscovered=0;//last unfound node;

     while(last_undiscovered < vertex_count()){
          //mark S as discovered
          discovered[last_undiscovered] = true;
          coloring[last_undiscovered] = 0;//coloring blue

          //creating Queue data structure intialized with S
          std::queue<int> Q;
          Q.push(last_undiscovered);

          //while the Q is not empty
          while(Q.size() > 0){
               int u = Q.front(); // getting the top item from the Q
               Q.pop();//removing u from he Q
               std::list<int> vertices;
               adjacent(u, vertices); // getting all connected vertices adjacent, wealky connected for
               while(vertices.size() > 0){//for all v adjacent to u
                    int v = vertices.front(); //setting v
                    vertices.pop_front();//removing v from the list
                    if(discovered[v] == false){ // if v is undiscovered
                         discovered[v] = true;//set v to discovered
                         
                         //setting v to the opposite of u
                         if((coloring.count(v) > 0) && (coloring[v] == coloring[u])){//if v is colored and it is the same as u
                              return false;
                         }
                         else if(coloring[u] == 1){
                              coloring[v] = 0;
                         }
                         else if(coloring[u] == 0){
                              coloring[v] = 1;
                         }

                         std::list<int> vertices2;
                         adjacent(v, vertices2); // getting all connected vertices adjacent, wealky connected for directed graph
                         while(vertices2.size() > 0){//for all w adjacent to v
                              int w = vertices2.front(); //setting w
                              vertices2.pop_front();//removing w from the list
                              if((coloring.count(w) > 0) && (coloring[v] == coloring[w])){//if w is colored and it is the same as v
                                   return false;
                              }
                              else if(coloring[v] == 1){
                                   coloring[w] = 0;
                              }
                              else if(coloring[v] == 0){
                                   coloring[w] = 1;
                              }
                         }
                         Q.push(v); //adding v to the end of Q
                    }

               }
          }
          while(discovered[last_undiscovered] == true){//finding the next component
               last_undiscovered++;
               if(last_undiscovered >= vertex_count()){ //if we have gone over the vertex count, break
                    break;
               }
          }
     }
     return true;
     
}

 //----------------------------------------------------------------------
  // HW-4 graph operations
  //----------------------------------------------------------------------
  
  //----------------------------------------------------------------------
  // Depth-first search from a given source vertex.
  //
  // Inputs: 
  //   dir -- if true assumes graph is directed
  //   src -- the source vertex
  // Outputs:
  //   tree -- search tree that maps vertices found during dfs to their
  //           corresponding parent vertex.
  //----------------------------------------------------------------------
void Graph::dfs(bool dir, int src, Map& tree) const{
     tree.clear();

     // marking all vertices in v as unexplored
     bool discovered[vertex_count()] = {0};

     //setting parent of S to -1 to denote root
     tree[src] = -1;

     //creating Queue data structure intialized with S
     std::stack<int> stak;
     stak.push(src);
     
     //while the Q is not empty
     while(stak.size() > 0){
          int u = stak.top(); // getting the top item from the Q
          //Q.pop();//removing the top item from the Q
          if(discovered[u] == false){
               discovered[u] = true;
          }
          std::list<int> vertices;
          if(dir == true){ //if directed graph
               connected_to(u, vertices); // getting all vertices u goes to
          }
          if(dir == false){ //if undirected graph
               adjacent(u, vertices); // getting all vertices u goes to
          }
          bool dead_end = true;
               while(0 < vertices.size()){ // for all the vertices adajcent to u
                    if(discovered[vertices.front()] == false ){ // check if undiscovered
                         if(vertices.front() != u){// catching self edges
                              tree[vertices.front()] = u; // set parent to u
                              stak.push(vertices.front()); // add to stak
                              dead_end = false; // mark as not dead end
                         }
                    }
                    vertices.pop_front();
               }
          if(dead_end == true){ // if it is a dead end we know that u is the the top of the stak
               stak.pop(); // removing dead end
          }
          
     }
}

  //----------------------------------------------------------------------
  // Determine if the graph is acyclic or not.
  //
  // Inputs:
  //   dir -- if true assumes graph is directed
  // Outputs:
  //   returns -- true if acyclic
  //----------------------------------------------------------------------
bool Graph::acyclic(bool dir) const{
     Map coloring; 
     // White = Undiscovered = 0
     // Gray = Discovered = 1
     // Black =  Dead end = 2
     

     Map tree; 
     //iterating over all the vertices
     for(int k=0; k< vertex_count(); k++){
          tree.clear();
          //setting each vertex to white
          for(int i =0 ; i < vertex_count(); i ++){
               coloring[i] = 0;
          }

          //setting parent of S to -1 to denote root
          tree[k] = -1;

          //creating Queue data structure intialized with S
          std::stack<int> stak;
          stak.push(k);
          
          //while the Q is not empty
          while(stak.size() > 0){

               //creating the stack
               int u = stak.top(); // getting the top item from the Q
               if(coloring[u] == 0){ // if node is white
                    //coloring u gray
                    coloring[u] = 1;
               }

               //getting all adjacent vetices
               std::list<int> vertices;
               if(dir == true){ //if directed graph
                    connected_to(u, vertices); // getting all vertices u goes to
               }
               if(dir == false){ //if undirected graph
                    adjacent(u, vertices); // getting all vertices u goes to
               }

               //iterating through all vetices adjacent to u
               bool dead_end = true;
               while(0 < vertices.size()){ // for all the vertices adajcent to u 
                    
                    // for all the vertices adajcent to u
                    if(coloring[vertices.front()] == 0 ){ // check if v is white

                         tree[vertices.front()] = u; // set parent to u
                         stak.push(vertices.front()); // add to stak
                         dead_end = false; // mark as not dead end
                    }
                    else if(coloring[vertices.front()] == 1){ // if coloring is gray

                         if(dir == false && (tree[u] != vertices.front())){ // if undirected, checking if the vertex is the parent
                              return false; // there is a cycle
                         }
                         else if(dir == true){
                              return false;
                         }
                    }
                    vertices.pop_front();
               }
               if(dead_end == true){ // if it is a dead end we know that u is the the top of the stak
                    coloring[stak.top()] = 2;// coloring black to denote dead end
                    stak.pop(); // removing dead end
               }
               
          }
     }

     return true;
     
     
}

  //----------------------------------------------------------------------
  // Brute force implementation to compute the transitive closure of
  // the current graph without consideration of edge weights.
  //
  // Conditions: Assumes that the given graph (the closed_graph) is a
  //             copy of the current graph prior to the call.
  // 
  // Inputs:
  //   dir -- if true assumes graph is directed
  // Outputs:
  //   closed_graph -- the transitively closed graph, where added
  //                   edges have
  //----------------------------------------------------------------------
void Graph::unweighted_transitive_closure(bool dir, Graph& closed_graph) const{
     for(int i =0; i< vertex_count(); i++){ // for all the vertexes
          Map tree;
          dfs(dir,i,tree); // run dfs
          for(int k =0; k<vertex_count(); k++){// for all the vertexes
               if(tree.count(k) > 0){ // if the key was found  by DFS, then it is reachable from vertex i
                    int fake_edge;
                    if(dir == true){ // directed
                         if(!closed_graph.get_edge(i,k,fake_edge) && k != i){//if the edge does not already exist, and the k is not i
                              closed_graph.set_edge(i, 0, k); //adding transitive edge to G+ with weight 0
                         }
                    }
                    else if(dir == false){ // if undirected
                         if(!closed_graph.get_edge(i,k,fake_edge) && !closed_graph.get_edge(k,i,fake_edge)&& k != i){//if the edge does not already exist either way, and the k is not i
                              closed_graph.set_edge(i, 0, k); //adding transitive edge to G+ with weight 0
                         }
                    }
               }
          }
     }
}

  //----------------------------------------------------------------------
  // Computes a topological sort of the current graph based on dfs.
  //
  // Conditions: Assumes the graph is directed.
  // Outputs:
  //
  //   vertex_ordering -- a map from vertex to its corresponding
  //                      order in the topological sort (where nodes
  //                      are ordered from 1 to n)
  //----------------------------------------------------------------------
bool Graph::dfs_topological_sort(Map& vertex_ordering) const{
     Map coloring; 
     // White = Undiscovered = 0
     // Gray = Discovered = 1
     // Black =  Dead end = 2
     

     Map tree; 
     //iterating over all the vertices
     for(int k=0; k< vertex_count(); k++){
          tree.clear();
          //setting each vertex to white
          for(int i =0 ; i < vertex_count(); i ++){
               coloring[i] = 0;
          }

          //setting parent of S to -1 to denote root
          tree[k] = -1;

          //creating Queue data structure intialized with S
          std::stack<int> stak;
          stak.push(k);
          int count =vertex_count();
          
          //while the Q is not empty
          while(stak.size() > 0){

               //creating the stack
               int u = stak.top(); // getting the top item from the Q
               if(coloring[u] == 0){ // if node is white
                    //coloring u gray
                    coloring[u] = 1;
               }

               //getting all adjacent vetices
               std::list<int> vertices;
               //always directed graph
               connected_to(u, vertices); // getting all vertices u goes to

               //iterating through all vetices adjacent to u
               bool dead_end = true;
               while(0 < vertices.size()){ // for all the vertices adajcent to u
                    
                    // for all the vertices adajcent to u
                    if(coloring[vertices.front()] == 0 ){ // check if v is white

                         tree[vertices.front()] = u; // set parent to u
                         stak.push(vertices.front()); // add to stak
                         dead_end = false; // mark as not dead end
                    }
                    else if(coloring[vertices.front()] == 1){ // if coloring is gray
                         return false;
                    }
                    vertices.pop_front();
               }
               if(dead_end == true){ // if it is a dead end we know that u is the the top of the stak
                    vertex_ordering[u] = count;
                    count--;
                    coloring[stak.top()] = 2;// coloring black to denote dead end
                    stak.pop(); // removing dead end
               }
               
          }
     }

};

 //----------------------------------------------------------------------
  // HW-5 graph operations
  //----------------------------------------------------------------------

  //----------------------------------------------------------------------
  // Computes the strongly connected components.
  //
  // Inputs:
  //   none
  // Outputs: 
  //   components -- mapping from each graph vertex to its corresponding
  //                 component number where component numbers range from
  //                 0 to c-1 (for c components)
  //----------------------------------------------------------------------
void Graph::strongly_connected_components(Map& components)const {
     // finding sink for each new graph
     int comp_number =0;
     components.clear();
          //reverse graph
          int i =0; //starting from 0

          Map pre_time; //declaring maps to store times and clock
          Map post_time;

          int clock = 0;

          // marking all vertices in v as unexplored
          bool discovered[vertex_count()] = {0};
          
          while(post_time.size() < vertex_count()){ //going until all vertices mapped

               //creating Queue data structure intialized with S
               std::stack<int> stak;
               stak.push(i);
               
               //while the Q is not empty
                    while(stak.size() > 0){
                         int u = stak.top(); // getting the top item from the Q
                         if(discovered[u] == false){
                              discovered[u] = true;
                              pre_time[u] = clock;
                              clock++;
                         }
                         std::list<int> vertices;// getting all vertices u goes to, allways directed graph, 
                         connected_from(u, vertices);  // calling connected from to get the reverse graph
                         bool dead_end = true;
                              while(0 < vertices.size()){ // for all the vertices adajcent to u
                                   if(discovered[vertices.front()] == false ){ // check if undiscovered
                                        if(vertices.front() != u){// catching self edges
                                             stak.push(vertices.front()); // add to stak
                                             dead_end = false; // mark as not dead end
                                        }
                                   }
                                   vertices.pop_front();
                              }
                         if(dead_end == true){ // if it is a dead end we know that u is the the top of the stak
                              stak.pop(); // removing dead end
                              post_time[u] = clock;
                              clock++;
                         }
                         
                    }
               if(post_time.size() < vertex_count()){ //checking if all vertices have been found
                    for(int k=0; k<vertex_count(); k++){ //chekcin which vertices have not been found yes
                         if(post_time.count(k) == 0){ //lowest vertex not found is the new starting point
                              i =k;
                              break;
                         }
                    }
                    
               }
          }
          // now that graph is mapped

          //finding the largest post time
          int largest_post=-1;
          int largest_post_val = -1;
          for(i =0; i<vertex_count(); i++){
               if(post_time[i] >  largest_post_val){
                    largest_post_val = post_time[i];
                    largest_post = i;
               }
          }
          //now that sink is found
          // running dfs on the normal graph from the largest post time
     
          // marking all vertices in v as unexplored
          bool discovered1[vertex_count()] = {0};

     while(components.size() <= vertex_count()-1){
          //creating Queue data structure intialized with S
          std::stack<int> stak;
          stak.push(largest_post);
          
          //while the Q is not empty
          while(stak.size() > 0){
               int u = stak.top(); // getting the top item from the Q
               //Q.pop();//removing the top item from the Q
               if(discovered1[u] == false){
                    discovered1[u] = true;
               }
               std::list<int> vertices;
               connected_to(u, vertices); // always directed graph
               bool dead_end = true;
                    while(0 < vertices.size()){ // for all the vertices adajcent to u
                         if(discovered1[vertices.front()] == false ){ // check if undiscovered
                              if(vertices.front() != u){// catching self edges
                                   stak.push(vertices.front()); // add to stak
                                   dead_end = false; // mark as not dead end
                              }
                         }
                         vertices.pop_front();
                    }
               if(dead_end == true){ // if it is a dead end we know that u is the the top of the stak
                    stak.pop(); // removing dead end
               }
          }

          for(int k=0; k<vertex_count(); k++){
               if(discovered1[k] == true){
                    if(components.count(k) < 1){//if the component number has not been set already
                         components[k] = comp_number;
                         //removing all vertices goind to and from that vertice
                         //for(int i =0; i<vertex_count(); i++){
                              //remove_edge(k,i);
                              //remove_edge(i,k);
                         post_time[k] = -2; //removing from post time 
                    }
               }
          }
          comp_number++;

          largest_post=-1;
          largest_post_val = -1;
          for(i =0; i<vertex_count(); i++){
               if(post_time[i] >  largest_post_val){
                    largest_post_val = post_time[i];
                    largest_post = i;
               }
          }

     }

     
     
};

  //----------------------------------------------------------------------
  // Computes the transitive reduction.
  //
  // Conditions: Assumes that the given graph (the closed_graph) has
  //             the same number of nodes as the current graph. But
  //             does not have any edges prior to the call
  //
  // Inputs:
  //   none
  // Outputs:
  //   reduced_graph -- the reduced edges are added to the reduced graph
  //----------------------------------------------------------------------
void Graph::transitive_reduction(Graph& reduced_graph) const{
     //1. compute SCC's
     Map components;
     strongly_connected_components(components);

     //2. for each component(until each vertex is mapped)
     int count =0;
     int component_number =0;
     while(count < vertex_count()){
          //get every vertex in the current component
          int last_vertex =-1;
          int first_vertex = -1;
          Map bridges; //map storing bridges between components
          for(int i=0; i<vertex_count(); i++){
               if(components[i] == component_number){
                    //3.
                    if(first_vertex == -1){ //starting the cycle
                         first_vertex = i;
                    }
                    else{
                         reduced_graph.set_edge(last_vertex,0,i); //3. creating simple cycle for each component
                         
                    }
                    last_vertex = i;
                    count++; //recording vertex as mapped

                    //4. checking if bridges exist to other components
                    std::list<int> vertices;
                    connected_to(i,vertices);//getting everthing current vertex is adjacent to
                    while(0 < vertices.size()){
                         int new_comp_number = components[vertices.front()];
                         if(new_comp_number != component_number){ //if it is a different component number
                              if(bridges.count(new_comp_number) == 0){ //if the bridge does not already exist
                                   //4. keeping one edge per component bridge
                                   reduced_graph.set_edge(i,0,vertices.front());
                                   bridges[new_comp_number] = 0; //adding to keep track of bridges
                              }
                              
                         }
                         vertices.pop_front();
                    }
               }
          }
          if(last_vertex != -1 && first_vertex != -1 && last_vertex !=  first_vertex){ // making sure it is  correct cycle
               reduced_graph.set_edge(last_vertex,0,first_vertex); //closing cycle
          }
          component_number++;
     }

     //5. computing irreducable kernel
     
     //1. let E- = E
     //Graph E = reduced_graph;

     //2. for each (u,v) in E
     for(int i =0; i<vertex_count(); i++){
          std::list<int> vertices;
          reduced_graph.connected_to(i,vertices);
          for(int j=0; j<vertices.size(); j++){
               int v = vertices.front();
               vertices.pop_front(); 
               //3. E - (u,v)
               reduced_graph.remove_edge(i,v);
               std::list<int> path;
               reduced_graph.shortest_path_length(true,i,v,path);
               if(path.size() ==0){ //4. if no other path was found
                    reduced_graph.set_edge(i,0,v);//put the edge back
               }//5. otherwise, leave it deleted
          }
     }
     

};

  //----------------------------------------------------------------------
  // Check if an eulerian exists in a directed graph, and if so,
  // return one.
  //
  // Conditions: Assumes the graph is not disconnected.
  //
  // Inputs:
  //   none
  // Outputs:
  //   path -- the path as an ordered list of vertices
  //----------------------------------------------------------------------
bool Graph::directed_eulerian_path(std::list<int>& path) const{
     Map out_edges_num;
     //1. checking if EP exists

     int sources =0;
     int source_val =-1;
     int sinks =0;
     int sink_val =-1;
     for(int i =0; i<vertex_count(); i++){

          //gatheing number of connections in and out
          std::list<int> vertices_to;
          std::list<int> vertices_from;
          connected_to(i,vertices_to);
          connected_from(i,vertices_from);

          //checking if greater than 1 connection difference
          if((vertices_to.size() > vertices_from.size()+1) || (vertices_from.size() > vertices_to.size()+1)){
               path.clear();
               return false;
          }

          //checking if source
          if((vertices_to.size() == vertices_from.size()+1)){
               if(sources >0){ //too many sources
                    return false;
               }
               sources++;
               source_val = i;
          }

          //checking if sink
          if((vertices_from.size() == vertices_to.size()+1)){
               if(sinks > 0){ //too many sources
                    return false;
               }
               sinks++;
               sink_val = i;
          }
          //otherwise it must have equal sources and sinks

          //adding the number of out edges to the map, used later in modified DFS
          out_edges_num[i] = vertices_to.size();
     }

     //2. Computing path

     path.clear();

     // marking all vertices in v as unexplored
     bool discovered[vertex_count()] = {0};


     //creating Queue data structure intialized with S
     std::stack<int> stak;

     

     if(sources == 1){ //if a source exists, start there
          stak.push(source_val);
     }
     else{ //if not, start at any vertex that is not the sink
          stak.push(sink_val-1);
     }
     
     //while the Q is not empty
     while(stak.size() > 0){
          int u = stak.top(); // getting the top item from the Q
          if(discovered[u] == false){
               discovered[u] = true;
               std::stack<int> stak2;
               stak2.push(u); 
               while(0 < stak.size()){
                    int c = stak.top();
                    if(c != u){
                       stak2.push(c);  
                    }
                    stak.pop();
               }
               while(0 < stak2.size()){
                    stak.push(stak2.top());  
                    stak2.pop();
               }
          }
          std::list<int> vertices;
          connected_to(u, vertices); // getting all vertices u goes to
          bool dead_end = true;
          bool include = false;
               while(0 < vertices.size()){ // for all the vertices connencted to u
                    if(out_edges_num[u] > 0){ //if it has out vertices untraveled
                         for(int i = out_edges_num[u]; i<vertices.size(); i++){
                              vertices.pop_front(); //removing edges already visited
                         }
                         stak.push(vertices.back()); // adding unvisited edge to stak
                         dead_end = false; // mark as not dead end
                    }
                    if(vertices.back() == path.front()){ //if it is connected to the last part of the path
                         include = true;
                    }
                    vertices.pop_back();
               }
          if(dead_end == true ){ // if it is a dead end we know that u is the the top of the stak
               stak.pop(); // removing dead end
               if (include == true || path.size() ==0){ //if it is connected to last path or is the first path
                    path.push_front(u);//adding dead end to the path
               }
          }
          else{
               out_edges_num[u]--;
          }
     }

     return true;
};

//----------------------------------------------------------------------
  // HW-6 graph operations
  //----------------------------------------------------------------------

  //----------------------------------------------------------------------
  // Returns a Hamiltonian path if one exists in the current graph.
  //
  // Conditions: Treats the graph as directed.
  //
  // Outputs:
  //   path -- the Hamiltonian path
  //
  // Returns:
  //   true if a Hamiltonian path exists, false otherwise.
  //----------------------------------------------------------------------
bool Graph::directed_hamiltonian_path(std::list<int>& path) const{
     //iterate through all of the vertices
     for(int i = 0; i<vertex_count(); i++){
          bool discovered[vertex_count()] = {0};
          path.push_back(i);
            discovered[i] = true;
          if(directed_hamiltonian_rec(i,path, discovered)){
               return true;
          }
          path.pop_back();
        discovered[i] = false;
     }
     return false;

};

  //----------------------------------------------------------------------
  // Find a maximum matched graph using the augmenting paths algorithm
  // from the textbook.
  //
  // Conditions: Finds a matching only if the current graph is bipartite
  //
  // Output:
  //   max_matched_graph -- A graph with same vertices as original
  //                        graph, containing the edges in the
  //                        matching. The output graph is assumed to
  //                        be initialized with the same number of
  //                        vertices, but without any edges.
  // Returns:
  //   true if the current graph is bipartite, false otherwise
  //----------------------------------------------------------------------
bool Graph::bipartite_graph_matching(Graph& max_matched_graph) const{

     //intializing 
     Map coloring;

     //checking if bipartite
     if(bipartite_graph(coloring)){
          //intializing
          Map M;
          Map V;
          Map U;
          int i1 = -1;
          int i2 = -1;
          std::queue<int> Q;

          // assigning vertices to v1 and v2
          for(int k = 0; k < vertex_count(); k++){
               if(coloring[k] == 0){
                    V[k] =0;
                    Q.push(k); //creating Queue data structure intialized with v1
               }
               else{
                    U[k]=0;
               }
          }

               //traversing an augmented path from i1 to i2 using BFS

               // marking all vertices in v as unexplored
               bool discovered[vertex_count()] = {0};

               //mark S as discovered
               discovered[i1] = true;

               //labels building an augmenting path
               std::vector<int> labels;
               labels.assign(vertex_count(),-1);
               //while the Q is not empty
               while(Q.size() > 0){
                    int w = Q.front(); // getting the top item from the Q
                    Q.pop();//removing u from the Q
                    if(V.count(w) ==1){ //if u is in V
                         std::list<int> vertices;
                         adjacent(w, vertices); // getting all vertices u goes to
                         while(vertices.size() > 0){//for all v adjacent to u
                              int u = vertices.front(); //setting v with no label
                              vertices.pop_front();//removing v from the list
                              if(M.count(u) == 0){ //if u is free
                                   //augment
                                   M[w] = u;
                                   M[u] = w;
                                   max_matched_graph.set_edge(w,0,u);
                                   int v = w; //making v = w 
                                   while(labels[v] >= 0){ //while v is labled
                                        u = labels[v];
                                        //deleting (v,u) from M
                                        if(M[u] == v){
                                             M.erase(u);
                                        }
                                        if(M[v] == u){
                                             M.erase(v);
                                        }
                                        max_matched_graph.remove_edge(v,u);
                                        v = labels[u];
                                        M[u] = v;
                                        M[v] = u;
                                        max_matched_graph.set_edge(v,0,u);
                                   }
                                   labels.clear();
                                   labels.assign(vertex_count(),-1);
                                   //clearing the Q
                                   while(Q.size() >0){
                                        Q.pop();
                                   }
                                   //reinitalizing Q with all free vertices in V
                                   for(int i=0; i<vertex_count(); i++){
                                        if(V.count(i)==1 && M.count(i)==0){
                                             Q.push(i);
                                        }
                                   }
                                   break;
                              }
                              else{ //u is matched
                                   if((labels[u] == -1) && (M.count(w) == 0 || M[w] != u)&&(M.count(u) == 0 || M[u] != w)){ //if (w,u) DNE in M and u is unlabled
                                        labels[u] = w;
                                        Q.push(u);
                                   }
                              }
                         }
                              
                    }
                    else{ //w is in U and matched
                         int v = M[w]; //mate of w
                         labels[v] = w; //labeling mate v of w with w
                         Q.push(v);
                    }
               }
          M.count(0);
          return true;
     }
};

  //----------------------------------------------------------------------
  // Finds all (maximal) cliques in the graph using the Bron-Kerbosch
  // algorithm.
  //
  // Conditions: Assumes the graph is undirected.
  //
  // Output:
  //   cliques -- a list of list of vertices denoting a maximal clique
  //----------------------------------------------------------------------
void Graph::cliques(std::list<Set>& cliques) const{
     std::set<int> P;
     for(int i =0; i<vertex_count(); i++){
          P.insert(i);
     }
     std::set<int> R;
     std::set<int> X;
     cliques_rec(P,R,X,cliques);

};
  

//private:

  // helper function for directed hamiltonian recursive function
bool Graph::directed_hamiltonian_rec(int v, std::list<int>& path, bool discovered[]) const{
     if(path.size() == vertex_count()){
          return true;
     }
     else{
          std::list<int> vertices;
          connected_to(v,vertices);
          while(0<vertices.size()){
               int u = vertices.front();
               if(discovered[u] == 0){
                    path.push_back(u);
                    discovered[u] = true;
                    if(directed_hamiltonian_rec(u,path,discovered)){
                         return true;
                    }
                    else{
                         path.pop_back();
                         discovered[u] = false;
                    }
               }
               vertices.pop_front();
          }
          return false;
     }
     //return false;
};

  // helper function for finding all cliques
void Graph::cliques_rec(Set& p, Set& r, Set& x, std::list<Set>& cliques) const{
     if(p.size() == 0 && x.size() ==0 ){ //if yes R is maximal
          //cliques = cliuqes and R
          cliques.push_front(r);
          return;
     }
     //std::set<int>::iterator it = p.begin();
     //int v;
     //std::set<int>::iterator it = p.begin();
     for(int i =0; i<vertex_count(); i++){
          if(p.count(i) >0){
               int v = i;

               //creating P and adj(v)
               std::list<int> vertices;
               adjacent(v,vertices);
               std::list<int> tmp;
               set_intersection(p.begin(), p.end(), vertices.begin(), vertices.end(), back_inserter(tmp));
               std::set<int> P(tmp.begin(), tmp.end());

               //creating X and adj(v)
               adjacent(v,vertices);
               tmp.clear();
               set_intersection(x.begin(), x.end(), vertices.begin(), vertices.end(), back_inserter(tmp));
               std::set<int> X(tmp.begin(), tmp.end());

                //creating R and v
               std::set<int> R =r;
               R.insert(v);

               cliques_rec(P,R,X,cliques);
               p.erase(v);
               x.insert(v);
          }
     }
     return;
};

//----------------------------------------------------------------------
  // HW-7 graph operations
  //----------------------------------------------------------------------

  //----------------------------------------------------------------------
  // Single-source shortest paths from the given source using
  // Dijkstra's algorithm.
  //
  // Conditions: Assumes graph is directed and maximum weight is
  //             numeric_limits<int>::max()
  // 
  // Input:
  //  src -- the source vertex
  //
  // Output:
  //  path_costs -- the minimum path cost from src to each vertex v
  //                given as path_costs[v].
  //----------------------------------------------------------------------
void Graph::dijkstra_shortest_path(int src, Map& path_costs) const{
    //intializing all distances to infinite
    for(int i =0; i<vertex_count();i++){
        path_costs[i] = std::numeric_limits<int>::max();  
    }
    path_costs[src] =0; //path to src is 0

    // marking all vertices in v as unexplored
     bool discovered[vertex_count()] = {0};

     //mark S as discovered
     discovered[src] = true;

     //creating Queue data structure intialized with S
     std::queue<int> Q;
     Q.push(src);
     
     //while the Q is not empty
     while(Q.size() > 0){
          int u = Q.front(); // getting the top item from the Q
          Q.pop();//removing u from he Q
          std::list<int> vertices;
        connected_to(u, vertices); // getting all vertices u goes to
          while(vertices.size() > 0){//for all v adjacent to u
               int v = vertices.front(); //setting v
               vertices.pop_front();//removing v from the list
               int edge = -1;
                get_edge(u,v,edge); //getting the edge from u to v
               if(discovered[v] == false){ // if v is undiscovered
                    discovered[v] = true;//set v to discovered
                    path_costs[v] = edge + path_costs[u];
                    Q.push(v); //adding v to the end of Q
               }
               else if((edge + path_costs[u]) < path_costs[v]){
                   path_costs[v] = edge + path_costs[u];
                   Q.push(v); //adding v to the end of Q
               }
               
          }
     }

}
  
  //----------------------------------------------------------------------
  // Compute a minimum spanning tree using Prim's algorithm.
  //
  // Conditions: Assumes a connected, undirected graph. The spanning
  //             tree is represented as a graph, which is initialized
  //             with the same vertices as the current graph, but with
  //             no edges (on input).
  //
  // Output:
  //  spanning-tree -- A graph containing the minimum spanning tree
  //                   edges.
  //
  //----------------------------------------------------------------------
void Graph::prim_min_spanning_tree(Graph& spanning_tree) const{

    //adding all edges to a set
    std::set<std::pair<int,int>> edges_added;

    std::set<int> vertices_added;;
    vertices_added.insert(0);//adding starting vertex to X

    while(vertices_added.size() < vertex_count()){ //while U exists in X and v does not

        //getting all frontier edges
        std::list<std::pair<int,int>> frontier_edges; 
        for(std::set<int>::iterator it = vertices_added.begin(); it != vertices_added.end(); it++){ //iterating over all the added vertices;
            int u = *it;
            std::list<int> vertices;
            adjacent(u,vertices); //undirected graph
            while(vertices.size() > 0){
                int v = vertices.front();
                vertices.pop_front();
                std::pair<int,int> p;
                p =std::make_pair(u,v);
                if(edges_added.count(p) < 1){//if the edge is not already added to the tree
                    frontier_edges.push_front(p); //adding 
                }
            }
        }

        //iterating over all frontier edges
        std::pair<int,int> smallest_edge;
        int smallest_edge_weight = std::numeric_limits<int>::max();
        while(frontier_edges.size()>0){
            //getting edge_weight
            int edge_weight=-1;
            int u = std::get<0>(frontier_edges.front());
            int v = std::get<1>(frontier_edges.front());
            get_edge(u,v,edge_weight);
            if(edge_weight == -1){
                get_edge(v,u,edge_weight);
            }
            //finding smallest edgeweight of the frontier edges
            if(edge_weight < smallest_edge_weight && vertices_added.count(v) <1){
                smallest_edge_weight = edge_weight;
                smallest_edge= std::make_pair(u,v);
            }
            frontier_edges.pop_front();
        }

        //updateing tree
        int u = std::get<0>(smallest_edge);
        int v = std::get<1>(smallest_edge);
        std::pair<int,int> p;
        p = std::make_pair(u,v);
        std::pair<int,int> p1;
        p1 = std::make_pair(v,u);

        spanning_tree.set_edge(u,smallest_edge_weight,v);

        //updating vertices and edges added
        vertices_added.insert(v);
        edges_added.insert(p);
        edges_added.insert(p1);
    }

}

//helper functions

void Graph::merge_edges_by_weight(std::vector<std::tuple<int,int,int>> &A, int start,int mid, int end){
     if(start < end && mid >=start && mid <= end){ // if the bounds passed in are correct
        std::tuple<int,int,int> MT(0,0,0);
          std::vector<std::tuple<int,int,int>> tmp(edge_count(),MT);//[(end-start)+1]; //creating temporary array to sort 2 halves
          int first1 = start;
          int first2 = mid+1;
          int i = 0;
          while((first1 <= mid) && (first2 <= end)){ //combining the left half and right half into temp
               if(std::get<1>(A[first1]) < std::get<1>(A[first2])){
                    tmp[i] = A[first1];
                    first1++;
                    i++;
               }
               else{
                    tmp[i++] = A[first2++];
               }
          }
          while(first1 <= mid){ // one of their while loops will execute to finish sorting array
               tmp[i++] = A[first1++];
          }
          while(first2 <= end){
               tmp[i++] = A[first2++];
          }
          for(int k = start; k<=end; k++ ){ // moving items back from temporary array into orginal
               A[k] = tmp[k-start];
          }
          int t =0;

     }




}

void Graph::split_edges_by_weight(std::vector<std::tuple<int,int,int>> &A, int start, int end){
    if(start < end){
    size_t mid = (start + end)/2; //spliting phase
    split_edges_by_weight(A, start, mid); //left side

    split_edges_by_weight(A, mid+1, end); //right side

    merge_edges_by_weight(A,start,mid,end); //merging
    }



}


  //----------------------------------------------------------------------
  // Compute a minimum spanning tree using Kruskal's algorithm.
  //
  // Conditions: Assumes a connected, undirected graph. The spanning
  //             tree is represented as a graph, which is initialized
  //             with the same vertices as the current graph, but with
  //             no edges (on input).
  //
  // Output:
  //  spanning-tree -- A graph containing the minimum spanning tree
  //                   edges.
  //
  //----------------------------------------------------------------------
void Graph::kruskal_min_spanning_tree(Graph& spanning_tree){
//getting an array of edges
std::vector<std::tuple<int,int,int>> edges;

for(int i=0; i< vertex_count();i++){
    std::list<int> vertices;
    connected_to(i,vertices);
    while(vertices.size() > 0){
        int v = vertices.front();
        int edge_weight;
        get_edge(i,v,edge_weight);
        std::tuple<int,int,int> t (i,edge_weight,v);
        edges.push_back(t);
        vertices.pop_front();
    }
}

//sorting egdes with merge sort
split_edges_by_weight(edges, 0, edges.size()-1);

for(int i =0; i<edges.size(); i++){
    int u = std::get<0>(edges[i]);
    int v = std::get<2>(edges[i]);
    int w = std::get<1>(edges[i]);
    spanning_tree.set_edge(u,w,v);
    if(!spanning_tree.acyclic(false)){
        spanning_tree.remove_edge(u,v);
    }
}

}

//----------------------------------------------------------------------
  // HW-8 graph operations
  //----------------------------------------------------------------------

  //----------------------------------------------------------------------
  // Single-source shortest paths from the given source using
  // Bellman-Ford's algorithm.
  //
  // Conditions: Assumes graph is directed and maximum weight is
  //             numeric_limits<int>::max()
  // 
  // Input:
  //  src -- the source vertex
  //
  // Output:
  //  path_costs -- the minimum path cost from src to each vertex v
  //                given as path_costs[v].
  //
  // Returns: true if there is not a negative cycle, and false
  //          otherwise
  //----------------------------------------------------------------------
bool Graph:: bellman_ford_shortest_path(int src, Map& path_costs) const{

    //intializing all distances to infinite
    for(int i =0; i<vertex_count();i++){
        path_costs[i] = std::numeric_limits<int>::max();  
    }
    path_costs[src] =0; //path to src is 0


    //creating list of all edges
    std::vector<std::tuple<int,int,int>> edges;

    for(int i = 0; i<vertex_count(); i++){
        std::list<int> vertices;
        connected_to(i,vertices);
        while(vertices.size() >0){
            int u = vertices.front();
            vertices.pop_front();
            int edge_weight =std::numeric_limits<int>::max();  
            get_edge(i,u,edge_weight);
            std::tuple<int,int,int> tmp;
            tmp = std::make_tuple(i,edge_weight,u);
            edges.push_back(tmp);
        }
    }

    //nested for loop going over all edges
    for(int i =0; i<vertex_count()-1; i++){
        //updating each edges with corresponding shortest path
        for(int j =0; j<edges.size(); j++){
            int u = std::get<0>(edges[j]);
            int w = std::get<1>(edges[j]);
            int v = std::get<2>(edges[j]);
            if(path_costs[v] > path_costs[u]+ w ){
                if(path_costs[u] != std::numeric_limits<int>::max()){
                    path_costs[v] = path_costs[u]+ w;
                }
            }
        }
        int k =0;
    }

    //checking for negative cycles
    for(int i =0; i<edges.size(); i++){
        int u = std::get<0>(edges[i]);
        int w = std::get<1>(edges[i]);
        int v = std::get<2>(edges[i]);
        if(path_costs[v] > path_costs[u] + w){
            return false;
        }
    }
    return true;
}


  //----------------------------------------------------------------------
  // HW-9 graph operations
  //----------------------------------------------------------------------

  //----------------------------------------------------------------------
  // Finds a maximum-weight independent set for a path graph using
  // dynamic programming.
  //
  // Conditions: Assumes the graph is a path-graph starting at vertex 0 and
  //             ending at vertex n-1, that is, E = {(v0,v1), (v1,v2), ...,
  //             v(n-1,vn)}
  //
  // Input:
  //  vertex_weights -- weights for each of the n vertices in the graph
  //
  // Output:
  //  S -- set of vertices representing the maximum independent set
  //
  // Returns: true if the graph is a path graph, false otherwise
  //----------------------------------------------------------------------
bool Graph:: path_max_independent_set(int vertex_weights[], Set& S) const{
    //checking for path graph
    if(edge_count() > vertex_count() -1){
        return false;
    }
    
    int ends =0;
    for(int i =0; i<vertex_count(); i++){
        std::list<int> vertices;
        adjacent(i,vertices);
        if(vertices.size() ==1){
            ends++;
            if(ends >2){
                return false;
            }
        }
        else if(vertices.size() !=2){
            return false;
        }
    }

    //creating array A
    int A [vertex_count()] ={0}; // array length n+1
    A[0] =vertex_weights[0];
    A[1] = vertex_weights[1];
    for(int i =2; i<vertex_count(); i++){
        if(A[i-1] > A[i-2]+ vertex_weights[i]){
            A[i] = A[i-1];
        }
        else{
            A[i] = A[i-2]+ vertex_weights[i];
        }
        
    }
    //reconstructing S from A
    int start =0;
    if(A[vertex_count()-1] > A[vertex_count() -2]){
        S.insert(vertex_count()-1);
        start =vertex_count()-1;
    }
    else{
        S.insert(vertex_count()-2);
        start =vertex_count()-2;
    }
     while(start>2){
        if(A[start-2] >A[start-3]){
            start -=2;
            S.insert(start);
        }
        else{
            start -=3;
            S.insert(start);
        }
    }
    //corner case
    if(start == 2){
        S.insert(0);
    }

    //returning
    return true;
}

  //----------------------------------------------------------------------
  // Finds all-pairs shortest paths using the Floyd-Warshall algorithm.
  //
  // Conditions: Assumes weights is an uninitialized "n x n" matrix
  //             represented as a vector of vectors (to be completely
  //             filled in with all-pairs shortest path weights)
  //
  // Output:
  //   weights -- shortest path weights from u to v given by weights[u][v]
  //              for all u,v in the graph's vertices
  //
  // Returns: true if there is not a negative cycle, and false
  //          otherwise
  //----------------------------------------------------------------------
bool Graph:: all_pairs_shortest_paths(std::vector<std::vector<int>>& weights) const{

    
    //making 3d matrix
    std::vector<int> D1(vertex_count(), std::numeric_limits<int>::max());

    std::vector<std::vector<int>> D2 (vertex_count(), D1);

    std::vector<std::vector<std::vector<int>>> A(vertex_count()+1, D2);

    weights = D2;

    //covering base cases
    for(int u =0; u<vertex_count(); u++){
        for(int v = 0; v<vertex_count(); v++){
            int edge = std::numeric_limits<int>::max();
            if(u == v){ //if it is same vertex
                A[0][u][v] =0;
            }
            else if(get_edge(u,v,edge)){ //if the edge exists
                A[0][u][v] = edge;
            }
            else{
                A[0][u][v] = std::numeric_limits<int>::max();
            }
            
        }
    }

    //iterativley solveing subproblems
    for(int k=1; k<vertex_count(); k++){
        for(int u =0; u<vertex_count(); u++){
            for(int v = 0; v<vertex_count(); v++){
                int tmp = 0;
                if(A[k-1][u][k] == std::numeric_limits<int>::max()){
                    tmp = std::numeric_limits<int>::max();
                }
                else if(A[k-1][k][v] == std::numeric_limits<int>::max()){
                    tmp = std::numeric_limits<int>::max();
                }
                else{
                    tmp = A[k-1][u][k]+ A[k-1][k][v];
                }
                if(A[k-1][u][v] <= tmp){
                    A[k][u][v] = A[k-1][u][v];
                }
                else{
                    A[k][u][v] = tmp;
                }
            }
        }
    }
    
    //check for negative cycles
    for(int u =0; u<vertex_count(); u++){
        if(A[vertex_count()-1][u][u] < 0){
            return false;
        }
    }

    //set the distance values
    for(int u =0; u<vertex_count(); u++){
        for(int v =0; v<vertex_count(); v++){
            weights[u][v] = A[vertex_count()-1][u][v];
        }
    }
    return true;
}
  


  

#endif