import Stack from "../../Wolfie2D/DataTypes/Collections/Stack";
import Vec2 from "../../Wolfie2D/DataTypes/Vec2";
import NavigationPath from "../../Wolfie2D/Pathfinding/NavigationPath";
import NavPathStrat from "../../Wolfie2D/Pathfinding/Strategies/NavigationStrategy";
import GraphUtils from "../../Wolfie2D/Utils/GraphUtils";

class PriorityQueue<T> {
  private heap: T[];
  private compareFn: (a: T, b: T) => number;
  constructor(compareFn: (a: T, b: T) => number) {
    this.heap = []; this.compareFn = compareFn;
  }
  public enqueue(item: T): void {
    this.heap.push(item);
    this.bubbleUp(this.heap.length - 1);
  }
  public dequeue(): T | undefined {
    const root = this.heap[0]; const last = this.heap.pop();
    if (this.heap.length > 0) {
      this.heap[0] = last!;
      this.bubbleDown(0);
    }
    return root;
  }
  public get size(): number { return this.heap.length; }
  private bubbleUp(index: number): void {
    while (index > 0) {
      const parentIndex = Math.floor((index - 1) / 2);
      if (this.compareFn(this.heap[index], this.heap[parentIndex]) >= 0) break;
      [this.heap[index], this.heap[parentIndex]] = [this.heap[parentIndex], this.heap[index]];
      index = parentIndex;
    }
  }
  private bubbleDown(index: number): void {
    while (true) {
      const leftChildIndex = 2 * index + 1;
      const rightChildIndex = 2 * index + 2;
      let smallestChildIndex = index;
      if (leftChildIndex < this.heap.length && this.compareFn(this.heap[leftChildIndex], this.heap[smallestChildIndex]) < 0) smallestChildIndex = leftChildIndex;
      if (rightChildIndex < this.heap.length && this.compareFn(this.heap[rightChildIndex], this.heap[smallestChildIndex]) < 0) smallestChildIndex = rightChildIndex;
      if (smallestChildIndex === index) break;
      [this.heap[index], this.heap[smallestChildIndex]] = [this.heap[smallestChildIndex], this.heap[index],];
      index = smallestChildIndex;
    }
  }
}

/**
 * The AstarStrategy class is an extension of the abstract NavPathStrategy class. For our navigation system, you can
 * now specify and define your own pathfinding strategy. Originally, the two options were to use Djikstras or a
 * direct (point A -> point B) strategy. The only way to change how the pathfinding was done was by hard-coding things
 * into the classes associated with the navigation system. 
 * 
 * - Peter
 */
export default class AstarStrategy extends NavPathStrat {
    public getNeighbors(nodeIndex: number): number[] {
        const neighbors = []; let graph = this.mesh.graph;
        let edge = graph.getEdges(nodeIndex);
        while (edge) {
            neighbors.push(edge.y);
            edge = edge.next;
        }
        for (const nb of neighbors.slice()) {
            let edge = graph.getEdges(nb);
            while (edge) {
                if (graph.getNodePosition(nodeIndex).y == graph.getNodePosition(edge.y).y || graph.getNodePosition(nodeIndex).x == graph.getNodePosition(edge.y).x) { 
                    edge = edge.next; continue;
                }
                if (!neighbors.includes(edge.y)) neighbors.push(edge.y);
                edge = edge.next;
            }
        }
        /* console.log(`Neighbors of ${graph.getNodePosition(nodeIndex)}: ${neighbors.map(nb => graph.getNodePosition(nb))}`); */
        return neighbors;
    }
    /**
     * @see NavPathStrat.buildPath()
     */
    public buildPath(to: Vec2, from: Vec2): NavigationPath {
        /* console.log("Creating path using A*"); */
        /* console.log(`From: ${from}, To: ${to}`); */
        from = this.mesh.graph.getNodePosition(this.mesh.graph.snap(from)); to = this.mesh.graph.getNodePosition(this.mesh.graph.snap(to));
        let openSet = [{node: from, g: 0}], closedSet = new Map(), scores = new Map();

        function reconstructPath(node: Vec2, scores: Map<Vec2, {g: number, parent: Vec2}>, graph): Stack<Vec2> {
            let stack = new Stack<Vec2>(1000); /* console.log("Reconstructing path..."); */
            while (node) {
                stack.push(node); /* console.log(`Adding ${node} to path`); */
                node = scores.get(graph.snap(node))?.parent;
            }
            return stack;
        }

        while (openSet.length) {
            /* const {node: tempNode, g: currentGScore} = openSet.reduce((a, b) => (a.g+a.node.distanceTo(to)) <= (b.g+b.node.distanceTo(to)) ? a : b, openSet[0]);
            const currentNode: Vec2 = new Vec2(tempNode.x, tempNode.y);
            openSet.splice(openSet.findIndex(({node}) => node.x === currentNode.x && node.y === currentNode.y), 1); */
            openSet.sort((a, b) => (a.g+a.node.distanceTo(to)) - (b.g+b.node.distanceTo(to))); /* Sort the open set by f score */
            const {node: currentNode, g: currentGScore} = openSet.shift();

            if (currentNode.x == to.x && currentNode.y == to.y) return new NavigationPath(reconstructPath(currentNode, scores, this.mesh.graph)); /* Reached the end node */
            closedSet.set(currentNode.toString(), true); /* Add the current node to the closed set */
            for (const neighbor of this.getNeighbors(this.mesh.graph.snap(currentNode))){
                const nb = this.mesh.graph.getNodePosition(neighbor);
                if (closedSet.has(nb.toString())) continue;
                let tentativeGScore = currentGScore + (currentNode.distanceTo(nb) * ((currentNode.x == nb.x || currentNode.y == nb.y) ? 1 : 1.414));

                /* Neighbor is not in the open set, add it and calculate its h score */
                if (!openSet.some(({ node }) => node.toString() === nb.toString())) openSet.push({ node: nb, g: tentativeGScore });
                else if (tentativeGScore >= scores.get(neighbor).g) continue; /* This is not a better path */

                /* Update the neighbor's g score and set its parent to the current node */
                scores.set(neighbor, {g: tentativeGScore, parent: currentNode});
                /* openSet[openSet.length-1].g = tentativeGScore; */ /* Update the neighbor's f score in the open set */
            }
        }
        console.log("No path found!"); return new NavigationPath(new Stack());
    }
}
