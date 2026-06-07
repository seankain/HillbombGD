using Godot;
using System;

public delegate void CarReachedSinkEventHandler(NpcCar car);

public partial class NpcCar : AnimatableBody3D
{
	public HillChunk CurrentChunk { get; private set; }
	public event CarReachedSinkEventHandler CarReachedSink;

	private PathFollow3D _pathFollow;
	private float _speed;
	private bool _active;

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

		_pathFollow.Progress += _speed * (float)delta;
		GlobalTransform = _pathFollow.GlobalTransform;

		if (_pathFollow.ProgressRatio >= 1.0f)
		{
			CarReachedSink?.Invoke(this);
		}
	}

	public void Activate(Path3D path, HillChunk chunk, float speed, float startOffset = 0f)
	{
		CurrentChunk = chunk;
		_speed = speed;
		_active = true;

		path.AddChild(_pathFollow);
		_pathFollow.Progress = startOffset;

		GlobalTransform = _pathFollow.GlobalTransform;
	}

	public void Reset()
	{
		_active = false;
		_speed = 0f;
		CurrentChunk = null;

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

	private void SetCollisionEnabled(bool enabled)
	{
		foreach (var child in GetChildren())
		{
			if (child is CollisionShape3D shape)
				shape.Disabled = !enabled;
		}
	}
}
