using Godot;
using System;

public delegate void CarReachedSinkEventHandler(NpcCar car);

public partial class NpcCar : AnimatableBody3D
{
	public HillChunk CurrentChunk { get; private set; }
	public TrafficWaypoint CurrentWaypoint { get; private set; }
	public TrafficWaypoint TargetWaypoint { get; private set; }
	public event CarReachedSinkEventHandler CarReachedSink;

	private float _speed;
	private bool _active;
	private float _segmentProgress;
	private float _segmentLength;
	private TrafficSimulator _simulator;
	private RandomNumberGenerator _rng;
	private bool _stoppedAtStopLine;
	private Quaternion _currentRotation;
	private Quaternion _targetRotation;

	private static readonly Quaternion IdentityQuat = new Quaternion(0, 0, 0, 1);

	public override void _Ready()
	{
		_rng = new RandomNumberGenerator();
		_rng.Randomize();
		_currentRotation = IdentityQuat;
		_targetRotation = IdentityQuat;
		Disable();
	}

	public override void _PhysicsProcess(double delta)
	{
		if (!_active || TargetWaypoint == null)
			return;

		if (_stoppedAtStopLine)
		{
			if (CurrentWaypoint.StopLight == null ||
				CurrentWaypoint.StopLight.State == TrafficLightState.Green)
			{
				_stoppedAtStopLine = false;
			}
			else
			{
				return;
			}
		}

		if (_simulator != null && _simulator.IsBlocked(this))
			return;

		float dt = (float)delta;
		_segmentProgress += (_speed / _segmentLength) * dt;

		if (_segmentProgress >= 1.0f)
		{
			_segmentProgress = 1.0f;
			GlobalTransform = new Transform3D(ComputeSmoothedBasis(1.0f), TargetWaypoint.GlobalPosition);
			ArriveAtWaypoint();
			return;
		}

		var pos = CurrentWaypoint.GlobalPosition.Lerp(
			TargetWaypoint.GlobalPosition, _segmentProgress);
		GlobalTransform = new Transform3D(ComputeSmoothedBasis(_segmentProgress), pos);
	}

	public void Activate(TrafficWaypoint startWaypoint, HillChunk chunk, float speed)
	{
		CurrentChunk = chunk;
		CurrentWaypoint = startWaypoint;
		_speed = speed;
		_active = true;
		_stoppedAtStopLine = false;
		_currentRotation = IdentityQuat;
		_targetRotation = IdentityQuat;

		_simulator?.Register(this);

		if (!AdvanceToNextWaypoint())
		{
			GD.PrintErr($"NpcCar: no valid NextWaypoints on {startWaypoint.Name}, returning to pool");
			CarReachedSink?.Invoke(this);
			return;
		}

		ComputeTargetRotation();
		_currentRotation = _targetRotation;

		var spawnPos = startWaypoint.GlobalPosition;
		GlobalTransform = new Transform3D(new Basis(_targetRotation), spawnPos);
	}

	public void SetSimulator(TrafficSimulator simulator)
	{
		_simulator = simulator;
	}

	public void Reset()
	{
		_simulator?.Unregister(this);

		_active = false;
		_speed = 0f;
		CurrentChunk = null;
		CurrentWaypoint = null;
		TargetWaypoint = null;
		_segmentProgress = 0f;
		_segmentLength = 0f;
		_stoppedAtStopLine = false;
		_currentRotation = IdentityQuat;
		_targetRotation = IdentityQuat;
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

	private void ArriveAtWaypoint()
	{
		CurrentWaypoint = TargetWaypoint;

		if (CurrentWaypoint.IsStopLine && CurrentWaypoint.StopLight != null)
		{
			var state = CurrentWaypoint.StopLight.State;
			if (state == TrafficLightState.Red || state == TrafficLightState.Yellow)
			{
				_stoppedAtStopLine = true;
				return;
			}
		}

		if (!AdvanceToNextWaypoint())
		{
			if (!TryCrossChunkTransition())
			{
				CarReachedSink?.Invoke(this);
			}
		}
	}

	private bool AdvanceToNextWaypoint()
	{
		if (CurrentWaypoint.NextWaypoints == null || CurrentWaypoint.NextWaypoints.Length == 0)
			return false;

		var previousRotation = _targetRotation;
		TrafficWaypoint nextWp = null;

		if (CurrentWaypoint.NextWaypoints.Length == 1)
		{
			nextWp = CurrentWaypoint.NextWaypoints[0];
		}
		else
		{
			var candidates = CurrentWaypoint.NextWaypoints;
			int startIdx = _rng.RandiRange(0, candidates.Length - 1);
			for (int i = 0; i < candidates.Length; i++)
			{
				var candidate = candidates[(startIdx + i) % candidates.Length];
				if (candidate != null)
				{
					nextWp = candidate;
					break;
				}
			}
		}

		if (nextWp == null)
		{
			GD.PrintErr($"NpcCar: NextWaypoints on {CurrentWaypoint.Name} contains only null refs");
			return false;
		}

		TargetWaypoint = nextWp;
		_segmentProgress = 0f;
		_segmentLength = CurrentWaypoint.GlobalPosition.DistanceTo(TargetWaypoint.GlobalPosition);
		if (_segmentLength < 0.01f)
			_segmentLength = 0.01f;

		_currentRotation = previousRotation;
		ComputeTargetRotation();

		return true;
	}

	private bool TryCrossChunkTransition()
	{
		if (string.IsNullOrEmpty(CurrentWaypoint.LaneId) || CurrentChunk == null)
			return false;

		var cycler = GetChunkCycler();
		if (cycler == null)
			return false;

		TravelDirection direction = CurrentWaypoint.LaneId.Contains("Oncoming")
			? TravelDirection.Outbound
			: TravelDirection.Inbound;

		if (cycler.TryGetNeighborChunk(CurrentChunk, direction, out HillChunk neighbor))
		{
			var entryWp = cycler.FindEntryWaypoint(neighbor, CurrentWaypoint.LaneId);
			if (entryWp != null)
			{
				CurrentChunk = neighbor;
				CurrentWaypoint = entryWp;

				if (!AdvanceToNextWaypoint())
				{
					CarReachedSink?.Invoke(this);
					return false;
				}
				ComputeTargetRotation();
				GlobalTransform = new Transform3D(new Basis(_targetRotation), entryWp.GlobalPosition);
				return true;
			}
		}
		return false;
	}

	private void ComputeTargetRotation()
	{
		if (TargetWaypoint == null || CurrentWaypoint == null)
			return;

		var direction = (TargetWaypoint.GlobalPosition - CurrentWaypoint.GlobalPosition).Normalized();
		if (direction.LengthSquared() < 0.001f)
			return;

		var lookBasis = Basis.LookingAt(direction, Vector3.Up);
		_targetRotation = lookBasis.GetRotationQuaternion();
	}

	private Basis ComputeSmoothedBasis(float t)
	{
		var smoothT = Mathf.Min(t * 3.0f, 1.0f);
		var rotation = _currentRotation.Slerp(_targetRotation, smoothT);
		return new Basis(rotation);
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
