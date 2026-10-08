"""Native sparse BFS with the original cardinal-neighbour and queue order."""
import numpy as np
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import breadth_first_order


def shortest_paths(traversable,start):
    h,w=traversable.shape
    n=h*w
    distance=np.full(n,-1,dtype=np.int32)
    parent=np.full(n,-1,dtype=np.int32)
    if not traversable[start]:
        return distance.reshape(h,w),parent.reshape(h,w)
    nodes=np.flatnonzero(traversable)
    rows,cols=np.divmod(nodes,w)
    neighbours=nodes[:,None]+np.asarray([-w,w,-1,1])[None,:]
    valid=np.stack([rows>0,rows<h-1,cols>0,cols<w-1],axis=1)
    valid &= traversable.ravel()[np.clip(neighbours,0,n-1)]
    counts=np.zeros(n,dtype=np.int32)
    counts[nodes]=valid.sum(axis=1)
    indptr=np.empty(n+1,dtype=np.int32);indptr[0]=0
    np.cumsum(counts,out=indptr[1:])
    indices=neighbours[valid].astype(np.int32)
    # csgraph's dtype conversion sorts CSR indices for integer data, changing
    # parent ties. Native float64 avoids that conversion and preserves order.
    graph=csr_matrix((np.ones(len(indices),dtype=np.float64),indices,indptr),shape=(n,n))
    origin=start[0]*w+start[1]
    order,predecessors=breadth_first_order(graph,origin,directed=True,return_predecessors=True)
    distance[origin]=0
    for node in order[1:]:
        p=predecessors[node]
        parent[node]=p
        distance[node]=distance[p]+1
    return distance.reshape(h,w),parent.reshape(h,w)
