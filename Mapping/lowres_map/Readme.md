# LowResolutionMap Update 

## Brief:  
LRM is composed of a set of lowresolution nodes, which are updated according to the precise map.   

## Input:  
robot pos, current observed point cloud.  

## Process Results:  
Get the local traversable(TS) and untraversable nodes(NTS). The NTS nodes will be marked as *Xnode*, which will not be updated in few seconds.  
If updated **normally**, the interconnected nodes inside a bounding box will be updated.  
If updated **topologically**, nodes to robot within a certain L1 dist or inside the current FOV will be updated using the Djkstra algorithm. Whats more, the Djkstra paths can be record for the DTG maintenance.

## Process(Topo):  
```
1. Set Xnodes according to pointcloud.
2. Set current FOV.
3. Clear last local nodes.
#  Expand Topological map
3. Push the node of current robot pos into TS proirity set or NTS priority set.
4. Two Djkstra(TS and NTS) search are processed in turn:   
-------------------------------------------------------#NTS Djkstra  
    1. Pop front node in the NTS priority set and check the status of the node.
    2. If the status is "occupied" or "unknown"(may be set fronmtier in the future), it will be set as "Xnode".
    3. If the status is "free", it will be set as "local" and push its neighbours into TS proirity set or NTS priority set. If the node is connected by some hnodes, those hnodes are recorded.
-------------------------------------------------------   
-------------------------------------------------------#TS Djkstra  
    1. Pop front node in the TS priority set.
    2. The node will be set as "local" and push its neighbours into TS proirity set or NTS priority set. If the node is connected by some hnodes, those hnodes are recorded.
-------------------------------------------------------   
5. Clear expired Xnode. Clear dead blocks. Update topological relationships.
```

