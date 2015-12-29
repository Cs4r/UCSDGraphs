package roadgraph;

import geography.GeographicPoint;

public class EnrichedGeographicPoint {

	public final GeographicPoint vertex;
	public final RoadInfo label;
	
	public EnrichedGeographicPoint(GeographicPoint g, RoadInfo label){
		this.vertex = g;
		this.label = label;
	}

}
