import Stack from "../../Wolfie2D/DataTypes/Collections/Stack";
import Vec2 from "../../Wolfie2D/DataTypes/Vec2";
import NavigationPath from "../../Wolfie2D/Pathfinding/NavigationPath";
import NavPathStrat from "../../Wolfie2D/Pathfinding/Strategies/NavigationStrategy";
import GraphUtils from "../../Wolfie2D/Utils/GraphUtils";

// TODO Construct a NavigationPath object using A*

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
        const neighbors = [];
        let edge = this.mesh.graph.edges[nodeIndex];
        while (edge) {
            neighbors.push(edge.y);
            edge = edge.next;
        }
        return neighbors;
    }
    /**
     * @see NavPathStrat.buildPath()
     */
    public buildPath(to: Vec2, from: Vec2): NavigationPath {
        console.log("Creating path using A*");
        let openSet = [];
        let closedSet = new Map();
        let scores = new Map(); // Map of nodes to their g and h scores
        let sfrom = this.mesh.graph.getNodePosition(this.mesh.graph.snap(from)); let sto = this.mesh.graph.getNodePosition(this.mesh.graph.snap(to));
        openSet.push({node: sfrom, f: 0 + sfrom.distanceTo(sto)}); /* console.log(`From: ${sfrom}, To: ${sto}`); */

        function reconstructPath(node: Vec2, scores, graph) { /* Reconstruct path using parent pointers */
            let stack = new Stack<Vec2>(1000); /* console.log("Reconstructing path..."); */
            while (node != null) {
                stack.push(node); /* console.log(`Adding ${node} to path`); */
                try { node = scores.get(graph.snap(node)).parent; } catch { break; } 
            }
            return stack;
        }

        while (openSet.length > 0) {
            openSet.sort((a, b) => a.f - b.f); /* Get the node with the lowest f score from the open set */
            let currentNode: Vec2 = openSet[0].node; let currentFScore: number = openSet[0].f; openSet.shift();

            if (currentNode == sto) return new NavigationPath(reconstructPath(currentNode, scores, this.mesh.graph)); /* Reached the end node */
            closedSet.set(currentNode.toString(), true); /* Add the current node to the closed set */
            for (const neighbor of this.getNeighbors(this.mesh.graph.snap(currentNode))){
                const nb = this.mesh.graph.getNodePosition(neighbor);
                if (closedSet.has(nb.toString())) continue;
                /* console.log(`Evaluating neighbor of ${currentNode}: ${nb}`); */
                let tentativeGScore = currentFScore - currentNode.distanceTo(sto) + currentNode.distanceTo(nb);

                /* Neighbor is not in the open set, add it and calculate its h score */
                if (!openSet.some((element) => element.node.toString() === nb.toString())) openSet.push({node: nb, f: tentativeGScore + nb.distanceTo(sto)});
                else if (tentativeGScore >= scores.get(neighbor).g) continue;

                /* Update the neighbor's g score and set its parent to the current node */
                scores.set(neighbor, {g: tentativeGScore, h: nb.distanceTo(sto), parent: currentNode});
                openSet[openSet.length-1].f = tentativeGScore + scores.get(neighbor).h; /* Update the neighbor's f score in the open set */
            }
        }
        /* console.log("No path found!"); */ return new NavigationPath(new Stack());
    }
}
