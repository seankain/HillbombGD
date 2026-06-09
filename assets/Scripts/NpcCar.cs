using Godot;
using System;

public delegate void CarReachedSinkEventHandler(NpcCar car);

public partial class NpcCar : AnimatableBody3D
{
	public HillChunk CurrentChunk { get; private set; }
	public event CarReachedSinkEventHandler CarReachedSink;
	public float CurrentProgress => _pathFollow?.Progress ?? 0f;
	public Path3D CurrentPath { get; private set; }

	private PathFollow3D _pathFollow;
	private float _speed;
	private bool _active;
	private TrafficSimulator _simulator;
	private TrafficLightController _lightController;

	public override void _Ready()
	{
		_pathFollow = new PathFollow3D();
		_pathFollow.Loop = false;
		_pathFollow.RotationMode = PathFollow3D.RotationModeEnum.Oriented;
		Disable();
	}

	public override void _PhysicsProcess(double delta)
	{
		if (!_active)
			return;

		if (ShouldStopAtLight())
		{
			GlobalTransform = _pathFollow.GlobalTransform;
			return;
		}

		float proposedProgress = _pathFollow.Progress + _speed * (float)delta;

		if (_simulator != null && _simulator.IsBlockedByTraffic(this, CurrentPath, proposedProgress))
		{
			GlobalTransform = _pathFollow.GlobalTransform;
			return;
		}

		_pathFollow.Progress = proposedProgress;
		GlobalTransform = _pathFollow.GlobalTransform;

		if (_pathFollow.ProgressRatio >= 1.0f)
		{
			if (TryTransitionToNextPath())
				return;

			CarReachedSink?.Invoke(this);
		}
	}

	public void Activate(Path3D path, HillChunk chunk, float speed, float startOffset = 0f)
	{
		CurrentChunk = chunk;
		CurrentPath = path;
		_speed = speed;
		_active = true;

		path.AddChild(_pathFollow);
		_pathFollow.Progress = startOffset;

		_simulator?.RegisterCar(this, path);

		GlobalTransform = _pathFollow.GlobalTransform;
	}

	public void SetSimulator(TrafficSimulator simulator)
	{
		_simulator = simulator;
	}

	public void SetLightController(TrafficLightController controller)
	{
		_lightController = controller;
	}

	public void Reset()
	{
		if (_simulator != null && CurrentPath != null)
			_simulator.UnregisterCar(this, CurrentPath);

		_active = false;
		_speed = 0f;
		CurrentChunk = null;
		CurrentPath = null;
		_lightController = null;

		if (_pathFollow.GetParent() != null)
		{
			_pathFollow.GetParent().RemoveChild(_pathFollow);
		}
		_pathFollow.Progress = 0f;
	}

	public void Disable()
	{
		_active = false;
		Visible = false;
		ProcessMode = ProcessModeEnum.Disabled;
		SetPhysicsProcess(false);
		SetCollisionEnabled(false);
	}

	public void Enable()
	{
		Visible = true;
		ProcessMode = ProcessModeEnum.Always;
		SetPhysicsProcess(true);
		SetCollisionEnabled(true);
	}

	private bool ShouldStopAtLight()
	{
		if (_lightController == null || CurrentPath == null)
			return false;

		var state = _lightController.GetStateForPath(CurrentPath);
		if (state == TrafficLightState.Red || state == TrafficLightState.Yellow)
		{
			if (_pathFollow.ProgressRatio < 0.3f)
				return true;
		}
		return false;
	}

	private bool TryTransitionToNextPath()
	{
		if (CurrentChunk == null)
			return false;

		string pathName = CurrentPath.Name;
		bool isStraightThru = pathName.Contains("ThruPath");

		if (isStraightThru)
			return TryTransitionToNextChunk();

		return TryTransitionToThruPath();
	}

	private bool TryTransitionToThruPath()
	{
		if (CurrentChunk == null)
			return false;

		var endPos = _pathFollow.GlobalPosition;
		Path3D bestPath = null;
		float bestDist = 8.0f;

		foreach (var child in CurrentChunk.GetChildren())
		{
			if (child is Path3D candidate && candidate != CurrentPath && candidate.Name.Contains("ThruPath"))
			{
				var curveStart = candidate.GlobalPosition + candidate.Curve.GetPointPosition(0);
				var curveEnd = candidate.GlobalPosition +
					candidate.Curve.GetPointPosition(candidate.Curve.PointCount - 1);

				float distToStart = endPos.DistanceTo(curveStart);
				float distToEnd = endPos.DistanceTo(curveEnd);
				float dist = Mathf.Min(distToStart, distToEnd);

				if (dist < bestDist)
				{
					bestDist = dist;
					bestPath = candidate;
				}
			}
		}

		if (bestPath != null)
		{
			TransferToPath(bestPath, CurrentChunk);
			return true;
		}
		return false;
	}

	private bool TryTransitionToNextChunk()
	{
		var cycler = GetChunkCycler();
		if (cycler == null)
			return false;

		string pathName = CurrentPath.Name;
		TravelDirection direction = pathName.Contains("OnComing")
			? TravelDirection.Inbound
			: TravelDirection.Outbound;

		if (cycler.TryGetNeighborChunk(CurrentChunk, direction, out HillChunk neighbor))
		{
			foreach (var child in neighbor.GetChildren())
			{
				if (child is Path3D candidate && candidate.Name == pathName)
				{
					TransferToPath(candidate, neighbor);
					return true;
				}
			}
		}
		return false;
	}

	private void TransferToPath(Path3D newPath, HillChunk newChunk)
	{
		if (_simulator != null && CurrentPath != null)
			_simulator.UnregisterCar(this, CurrentPath);

		if (_pathFollow.GetParent() != null)
			_pathFollow.GetParent().RemoveChild(_pathFollow);

		CurrentPath = newPath;
		CurrentChunk = newChunk;

		newPath.AddChild(_pathFollow);
		_pathFollow.Progress = 0f;

		_simulator?.RegisterCar(this, newPath);

		GlobalTransform = _pathFollow.GlobalTransform;
	}

	private ChunkCycler GetChunkCycler()
	{
		if (CurrentChunk?.GetParent() is ChunkCycler cycler)
			return cycler;
		return null;
	}

	private void SetCollisionEnabled(bool enabled)
	{
		foreach (var child in GetChildren())
		{
			if (child is CollisionShape3D shape)
				shape.Disabled = !enabled;
		}
	}
}
