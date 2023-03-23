import Stack from "../../Wolfie2D/DataTypes/Collections/Stack";
import Vec2 from "../../Wolfie2D/DataTypes/Vec2";
import NavigationPath from "../../Wolfie2D/Pathfinding/NavigationPath";
import NavPathStrat from "../../Wolfie2D/Pathfinding/Strategies/NavigationStrategy";
import GraphUtils from "../../Wolfie2D/Utils/GraphUtils";

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
        let edge = graph.edges[nodeIndex];
        edge = graph.getEdges(nodeIndex);
        while (edge) {
            neighbors.push(edge.y);
            edge = edge.next;
        }
        for (const nb of neighbors.slice()) {
            let edge = graph.edges[nb];
            edge = graph.getEdges(nb);
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
        from = this.mesh.graph.getNodePosition(this.mesh.graph.snap(from)); to = this.mesh.graph.getNodePosition(this.mesh.graph.snap(to));
        let openSet = [{node: from, f: 0 + from.distanceTo(to)}], closedSet = new Map(), scores = new Map();

        function reconstructPath(node: Vec2, scores: Map<Vec2, {g: number, h: number, parent: Vec2}>, graph): Stack<Vec2> { /* Reconstruct path using parents */
            let stack = new Stack<Vec2>(1000); /* console.log("Reconstructing path..."); */
            while (node) {
                stack.push(node); /* console.log(`Adding ${node} to path`); */
                node = scores.get(graph.snap(node))?.parent;
            }
            return stack;
        }

        while (openSet.length > 0) {
            openSet.sort((a, b) => a.f - b.f); /* Get the node with the lowest f score from the open set */
            const {node: currentNode, f: currentFScore} = openSet.shift();

            if (currentNode == to) return new NavigationPath(reconstructPath(currentNode, scores, this.mesh.graph)); /* Reached the end node */
            closedSet.set(currentNode.toString(), true); /* Add the current node to the closed set */
            for (const neighbor of this.getNeighbors(this.mesh.graph.snap(currentNode))){
                const nb = this.mesh.graph.getNodePosition(neighbor);
                if (closedSet.size < 50) console.log(`Evaluating neighbor of ${currentNode}: ${nb}`);
                if (closedSet.has(nb.toString())) continue;
                /* console.log(`Evaluating neighbor of ${currentNode}: ${nb}`); */
                let tentativeGScore = currentFScore - currentNode.distanceTo(to) + (currentNode.distanceTo(nb) * ((currentNode.x == nb.x || currentNode.y == nb.y) ? 1 : 1.414));

                /* Neighbor is not in the open set, add it and calculate its h score */
                if (!openSet.some(({ node }) => node.toString() === nb.toString())) openSet.push({ node: nb, f: tentativeGScore + nb.distanceTo(to) });
                else if (tentativeGScore >= scores.get(neighbor).g) continue; /* This is not a better path */
                if (closedSet.size < 50) console.log(`FScore: ${tentativeGScore + nb.distanceTo(to)}`);

                /* Update the neighbor's g score and set its parent to the current node */
                scores.set(neighbor, {g: tentativeGScore, h: nb.distanceTo(to), parent: currentNode});
                openSet[openSet.length-1].f = tentativeGScore + scores.get(neighbor).h; /* Update the neighbor's f score in the open set */
            }
        }
        /* console.log("No path found!"); */ return new NavigationPath(new Stack());
    }
}
