using Godot;
using System;
using System.Collections.Generic;

public partial class TrafficSimulator : Node
{
	[Export]
	public float MinFollowDistance = 6.0f;

	private readonly Dictionary<Path3D, List<NpcCar>> _carsByPath = new();

	public void RegisterCar(NpcCar car, Path3D path)
	{
		if (!_carsByPath.ContainsKey(path))
			_carsByPath[path] = new List<NpcCar>();
		_carsByPath[path].Add(car);
	}

	public void UnregisterCar(NpcCar car, Path3D path)
	{
		if (_carsByPath.TryGetValue(path, out var list))
		{
			list.Remove(car);
			if (list.Count == 0)
				_carsByPath.Remove(path);
		}
	}

	public bool CanSpawnOnPath(Path3D path, float atProgress)
	{
		if (!_carsByPath.TryGetValue(path, out var cars))
			return true;

		foreach (var car in cars)
		{
			float dist = Mathf.Abs(car.CurrentProgress - atProgress);
			if (dist < MinFollowDistance)
				return false;
		}
		return true;
	}

	public bool IsBlockedByTraffic(NpcCar car, Path3D path, float progress)
	{
		if (!_carsByPath.TryGetValue(path, out var cars))
			return false;

		foreach (var other in cars)
		{
			if (other == car)
				continue;

			float otherProgress = other.CurrentProgress;
			float gap = otherProgress - progress;

			if (gap > 0 && gap < MinFollowDistance)
				return true;
		}
		return false;
	}
}
