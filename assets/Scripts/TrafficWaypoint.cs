using Godot;

public partial class TrafficWaypoint : Node3D
{
	[Export] public TrafficWaypoint[] NextWaypoints;
	[Export] public bool IsSpawnPoint;
	[Export] public bool IsStopLine;
	[Export] public TrafficLight StopLight;
	[Export] public string LaneId;
}
