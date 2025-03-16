using Godot;
using System;

public partial class CameraController : Node3D
{

	[Export]
	public Node3D TrackedNode;

	[Export]
	public float PercentPerFrame = 0.3f;

	// Called when the node enters the scene tree for the first time.
	public override void _Ready()
	{
	}

	// Called every frame. 'delta' is the elapsed time since the previous frame.
	public override void _Process(double delta)
	{
	}

	public override void _PhysicsProcess(double delta)
	{
		this.Position = LerpVector3(this.Position, TrackedNode.Position, PercentPerFrame);
	}

	/// <summary>
	/// TODO move into extension method or smomething
	/// </summary>
	public static Vector3 LerpVector3(Vector3 First, Vector3 Second, float Amount)
	{
		float retX = Mathf.Lerp(First.X, Second.X, Amount);
		float retY = Mathf.Lerp(First.Y, Second.Y, Amount);
		float retZ = Mathf.Lerp(First.Z, Second.Z, Amount);
		return new Vector3(retX, retY, retZ);
	}
}
