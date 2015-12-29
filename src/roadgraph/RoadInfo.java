package roadgraph;

public class RoadInfo {
	
	public final String roadName;
	public final String roadType; 
	public final double length;
	
	public RoadInfo(String roadName,
			String roadType, double length){
		this.roadName = roadName;
		this.roadType = roadType;
		this.length = length;
	}

}
