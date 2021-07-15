/*  This file is part of Openrouteservice.
 *
 *  Openrouteservice is free software; you can redistribute it and/or modify it under the terms of the
 *  GNU Lesser General Public License as published by the Free Software Foundation; either version 2.1
 *  of the License, or (at your option) any later version.

 *  This library is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 *  without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *  See the GNU Lesser General Public License for more details.

 *  You should have received a copy of the GNU Lesser General Public License along with this library;
 *  if not, see <https://www.gnu.org/licenses/>.
 */

package org.heigit.ors.routing.graphhopper.extensions.core;

import com.graphhopper.routing.ch.NodeContractor;
import com.graphhopper.routing.util.DefaultEdgeFilter;
import com.graphhopper.routing.util.EdgeFilter;
import com.graphhopper.routing.util.FlagEncoder;
import com.graphhopper.routing.weighting.Weighting;
import com.graphhopper.storage.*;
import com.graphhopper.util.CHEdgeExplorer;

/**
 * This is a modified copy of the original code from GraphHopper GmbH.
 *
 * @author Andrzej Oles
 */

abstract class CoreContractor implements NodeContractor {
    final CHGraph prepareGraph;
    final FlagEncoder encoder;
    CHEdgeExplorer inEdgeExplorer;
    CHEdgeExplorer outEdgeExplorer;
    private final DataAccess originalEdges;
    int maxLevel;
    private int maxEdgesCount;

    EdgeFilter restrictionFilter;

    public CoreContractor(CHGraph prepareGraph, Weighting weighting) {
        this.prepareGraph = prepareGraph;
        this.encoder = weighting.getFlagEncoder();
        originalEdges = new GHDirectory("", DAType.RAM_INT).find("");
        originalEdges.create(1000);
    }

    @Override
    public void initFromGraph() {
        inEdgeExplorer = prepareGraph.createEdgeExplorer(DefaultEdgeFilter.inEdges(encoder));
        outEdgeExplorer = prepareGraph.createEdgeExplorer(DefaultEdgeFilter.outEdges(encoder));
        maxLevel = prepareGraph.getNodes() + 1;
        maxEdgesCount = prepareGraph.getOriginalEdges();
    }

    @Override
    public void close() {
        originalEdges.close();
    }

    boolean isContracted(int node) {
        return prepareGraph.getLevel(node) < maxLevel;
    }

    void setOrigEdgeCount(int edgeId, int value) {
        edgeId -= maxEdgesCount;
        if (edgeId < 0) {
            // ignore setting as every normal edge has original edge count of 1
            if (value != 1)
                throw new IllegalStateException("Trying to set original edge count for normal edge to a value = " + value
                        + ", edge:" + (edgeId + maxEdgesCount) + ", max:" + maxEdgesCount + ", graph.max:" +
                        prepareGraph.getEdges());
            return;
        }

        long tmp = (long) edgeId * 4;
        originalEdges.ensureCapacity(tmp + 4);
        originalEdges.setInt(tmp, value);
    }

    int getOrigEdgeCount(int edgeId) {
        edgeId -= maxEdgesCount;
        if (edgeId < 0)
            return 1;

        long tmp = (long) edgeId * 4;
        originalEdges.ensureCapacity(tmp + 4);
        return originalEdges.getInt(tmp);
    }

    abstract boolean isEdgeBased();

    public void setRestrictionFilter(EdgeFilter filter){
        this.restrictionFilter = filter;
    }
}
