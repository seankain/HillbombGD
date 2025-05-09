using Godot;
using System;
using System.ComponentModel;

public partial class vehicle_board : VehicleBody3D
{

	public PlayerRespawnedEventHandler PlayerRespawned;
	[Export]
	public float MaxSteeringAngle = 30f;
	private bool respawnPressed = false;

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
		// Get the input direction and handle the movement/deceleration.
		// As good practice, you should replace UI actions with custom gameplay actions.
		Vector2 inputDir = Input.GetVector("Left", "Right", "Forward", "Backward");
		Vector3 direction = (Transform.Basis * new Vector3(inputDir.X, 0, inputDir.Y)).Normalized();
		this.Steering = Mathf.MoveToward(this.Steering, Input.GetAxis("Left", "Right"), (float)delta * MaxSteeringAngle);
		base._PhysicsProcess(delta);
	}

	public override void _IntegrateForces(PhysicsDirectBodyState3D state)
	{
		base._IntegrateForces(state);
		if (respawnPressed)
		{
			Respawn();
			respawnPressed = false;
		}

	}

	public override void _UnhandledInput(InputEvent @event)
	{
		if (@event is InputEventKey eventKey)
		{
			if (eventKey.Pressed && eventKey.Keycode == Key.Escape)
			{
				GetTree().Quit();
			}
			if (eventKey.Pressed && eventKey.Keycode == Key.R)
			{
				respawnPressed = true;
			}
		}
	}

	private void Respawn()
	{
		var respawn = GetTree().GetNodesInGroup("Respawn")[0] as Node3D;
		this.AngularVelocity = Vector3.Zero;
		this.LinearVelocity = Vector3.Zero;
		this.GlobalTransform = new Transform3D(respawn.GlobalTransform.Basis, respawn.GlobalPosition);
		this.PlayerRespawned?.Invoke(this, new PlayerRespawnArgs());


	}
}
