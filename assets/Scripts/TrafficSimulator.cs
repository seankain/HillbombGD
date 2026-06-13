using Godot;
using System.Collections.Generic;

public partial class TrafficSimulator : Node
{
	[Export]
	public float MinFollowDistance = 6.0f;

	private readonly HashSet<NpcCar> _activeCars = new();

	public void Register(NpcCar car) => _activeCars.Add(car);
	public void Unregister(NpcCar car) => _activeCars.Remove(car);

	public bool IsBlocked(NpcCar car)
	{
		var pos = car.GlobalPosition;
		var forward = -car.GlobalTransform.Basis.Z;
		foreach (var other in _activeCars)
		{
			if (other == car) continue;
			var toOther = other.GlobalPosition - pos;
			if (toOther.Dot(forward) > 0 && toOther.Length() < MinFollowDistance)
				return true;
		}
		return false;
	}

	public bool CanSpawnAt(Vector3 position)
	{
		foreach (var car in _activeCars)
		{
			if (car.GlobalPosition.DistanceTo(position) < MinFollowDistance)
				return false;
		}
		return true;
	}
}
