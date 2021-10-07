package org.heigit.ors.routing.graphhopper.extensions.core;

import com.graphhopper.routing.DijkstraBidirectionRef;
import com.graphhopper.routing.Path;
import com.graphhopper.routing.QueryGraph;
import com.graphhopper.routing.RoutingAlgorithm;
import com.graphhopper.routing.ch.PreparationWeighting;
import com.graphhopper.routing.util.*;
import com.graphhopper.routing.weighting.ShortestWeighting;
import com.graphhopper.routing.weighting.TurnWeighting;
import com.graphhopper.routing.weighting.Weighting;
import com.graphhopper.storage.*;
import com.graphhopper.storage.index.QueryResult;
import com.graphhopper.util.HelperORS;
import org.heigit.ors.fastisochrones.ToyGraphCreationUtil;
import org.heigit.ors.routing.graphhopper.extensions.edgefilters.core.TurnRestrictionsCoreEdgeFilter;
import org.heigit.ors.util.DebugUtility;
import org.junit.Before;
import org.junit.Test;

import java.util.ArrayList;

import static org.junit.Assert.assertEquals;

public class CoreRoutingTest {
    private final CarFlagEncoder carEncoder = new CarFlagEncoder(5,5.0D,1);
    private final EncodingManager encodingManager = EncodingManager.create(carEncoder);
    private final Weighting weighting = new ShortestWeighting(carEncoder);
    private final TraversalMode tMode = TraversalMode.NODE_BASED;
    private Directory dir;

    GraphHopperStorage createGHStorage() {
        return new GraphBuilder(encodingManager).setCHProfiles(new ArrayList<>()).setCoreGraph(weighting).withTurnCosts(true).create();
    }

    void addRestrictedTurn(GraphHopperStorage graph, int fromEdge, int viaNode, int toEdge) {
        TurnCostExtension turnCostExtension = HelperORS.getTurnCostExtensions(graph.getExtension());
        turnCostExtension.addTurnInfo(fromEdge, viaNode, toEdge, carEncoder.getTurnFlags(true, 0));
    }

    @Before
    public void setUp() {
        dir = new GHDirectory("", DAType.RAM_INT);
    }

    private CHGraph contractGraph(GraphHopperStorage g, EdgeFilter restrictedEdges) {
        CHProfile chProfile = new CHProfile(weighting, tMode, TurnWeighting.INFINITE_U_TURN_COSTS, "core");
        CHGraph lg = g.getCHGraph(chProfile);
        PrepareCore prepare = new PrepareCore(g, chProfile, restrictedEdges);

        //FIXME
        // set contraction parameters to prevent test results from changing when algorithm parameters are tweaked
        //prepare.setPeriodicUpdates(20);
        //prepare.setLazyUpdates(10);
        //prepare.setNeighborUpdates(20);
        //prepare.setContractedNodes(100);

        prepare.doWork();

        if (DebugUtility.isDebug()) {
            for (int i = 0; i < lg.getNodes(); i++)
                System.out.println("nodeId " + i + " level: " + lg.getLevel(i));
            AllCHEdgesIterator iter = lg.getAllEdges();
            while (iter.next()) {
                System.out.print(iter.getBaseNode() + " -> " + iter.getAdjNode() + " via edge " + iter.getEdge());
                if (iter.isShortcut())
                    System.out.print(" (shortcut)");
                System.out.println(" [weight: " + (new PreparationWeighting(weighting)).calcWeight(iter, false, -1) +"]");
            }
        }

        return lg;
    }

    private Path findCorePath(GraphHopperStorage graphHopperStorage, int from, int to) {
        EdgeFilter coreEdgeFilter = new TurnRestrictionsCoreEdgeFilter(carEncoder, graphHopperStorage);
        CHGraph g = contractGraph(graphHopperStorage, coreEdgeFilter);

        QueryGraph queryGraph = new QueryGraph(g);
        queryGraph.lookup(new ArrayList<QueryResult>());// mock call neccessary for proper initialization
        Weighting turnWeighting = new TurnWeighting(weighting, HelperORS.getTurnCostExtensions(graphHopperStorage.getExtension()), -1);
        CoreDijkstra algo = new CoreDijkstra(queryGraph, turnWeighting);
        // append any restriction filters after node level filter
        CoreDijkstraFilter levelFilter = new CoreDijkstraFilter(g);
        algo.setEdgeFilter(levelFilter);

        return algo.calcPath(from, to);
    }

    private Path findDijkstraPath(GraphHopperStorage graphHopperStorage, int from, int to) {
        QueryGraph queryGraph = new QueryGraph(graphHopperStorage);
        queryGraph.lookup(new ArrayList<QueryResult>());// mock call neccessary for proper initialization
        Weighting turnWeighting = new TurnWeighting(weighting, HelperORS.getTurnCostExtensions(graphHopperStorage.getExtension()), -1);
        RoutingAlgorithm algo = new DijkstraBidirectionRef(queryGraph, turnWeighting, TraversalMode.EDGE_BASED);
        return algo.calcPath(from, to);
    }

    @Test
    public void testRouteAvoidingTurnRestriction() {
        GraphHopperStorage graphHopperStorage = ToyGraphCreationUtil.createTwoWayGraph(createGHStorage());
        addRestrictedTurn(graphHopperStorage, 2, 3, 3);

        Path path = findCorePath(graphHopperStorage, 7, 5);
        assertEquals(13.0, path.getDistance(), 0);
    }

    @Test
    public void testTurnRestriction() {
        GraphHopperStorage graphHopperStorage = ToyGraphCreationUtil.createGraphForIssue1074(createGHStorage());
        addRestrictedTurn(graphHopperStorage, 1, 2, 5);

        Path path = findDijkstraPath(graphHopperStorage, 0,6);
        assertEquals(7.0, path.getDistance(), 0);

        // core routing fails because of #1074
        path = findCorePath(graphHopperStorage, 0,6);
        assertEquals(7.0, path.getDistance(), 0);
    }
}
